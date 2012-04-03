#include <uquad_error_codes.h> // DEBUG is defined here
#if DEBUG // The following define will affect includes
#define TIMING 0
#define TIMING_KALMAN 0
#define TIMING_IMU 0
#define TIMING_IO 0
#define LOG_W 1
#define LOG_W_CTRL 1
#define LOG_IMU_RAW 1
#define LOG_IMU_DATA 1
#define LOG_IMU_AVG 1
#define DEBUG_X_HAT 1
#define DEBUG_KALMAN_INPUT 1
#endif

#define USE_GPS 0

#include <uquad_types.h>
#include <macros_misc.h>
#include <uquad_aux_io.h>
#include <imu_comm.h>
#include <mot_control.h>
#include <uquad_kalman.h>
#include <control.h>
#include <path_planner.h>
#include <uquad_logger.h>
#if USE_GPS
#include <uquad_gps_comm.h>
#endif

#include <sys/signal.h>   // for SIGINT and SIGQUIT
#include <unistd.h>       // for STDIN_FILENO

#define UQUAD_HOW_TO   "./main <imu_device>"
#define MAX_ERRORS     20
#define STARTUP_RUNS   200
#define STARTUP_KALMAN 200
#define FIXED          3

/**
 *
 * Sampling time within [TS_MIN,TS_MAX] will be used for
 * kalman filtering. If out of range, using it within the kalman filter
 * would force a violent reaction, and the stabilization time would
 * be unacceptable.
 *
 */
#define TS_JITTER          2000L      // Max jitter accepted
#define TS_MAX             (TS_DEFAULT_US + TS_JITTER)
#define TS_MIN             (TS_DEFAULT_US - TS_JITTER)
#define TS_JITTER_RATE_MAX 1.0 // Warning display threshold

#define LOG_W_NAME         "w"
#define LOG_W_CTRL_NAME    "w_ctrl"

#define LOG_IMU_RAW_NAME   "imu_raw"
#define LOG_IMU_DATA_NAME  "imu_data"
#define LOG_IMU_AVG_NAME   "imu_avg"
#define LOG_X_HAT_NAME     "x_hat"
#define LOG_KALMAN_IN_NAME "kalman_in"
#define LOG_TV_NAME        "tv"

/**
 * Frequency at which motor controller is updated
 * Must be at least MOT_UPDATE_MAX_US
 *
 */
#define MOT_UPDATE_T MOT_UPDATE_MAX_US

/// Global structs
static imu_t *imu;
static kalman_io_t *kalman;
static uquad_mot_t *mot;
static io_t *io;
static ctrl_t *ctrl;
static path_planner_t *pp;
#if USE_GPS
static gps_t *gps;
#endif
/// Global var
uquad_mat_t *w, *wt;
uquad_mat_t *x;
imu_data_t imu_data;
/// Logs
#if DEBUG
#if LOG_IMU_RAW
FILE *log_imu_raw;
#endif //LOG_IMU_RAW
#if LOG_IMU_DATA
FILE *log_imu_data;
#endif //LOG_IMU_DATA
#if LOG_IMU_AVG
FILE *log_imu_avg;
#endif //LOG_IMU_AVG
#if LOG_W
FILE *log_w = NULL;
#endif //LOG_W
#if LOG_W
FILE *log_w_ctrl = NULL;
#endif //LOG_W
#if DEBUG_X_HAT
FILE *log_x_hat = NULL;
uquad_mat_t *x_hat_T = NULL;
#endif //DEBUG_X_HAT
#if DEBUG_KALMAN_INPUT
FILE *log_kalman_in = NULL;
#endif //DEBUG_KALMAN_INPUT
#endif //DEBUG
FILE *log_tv = NULL;

/** 
 * Clean up and close
 * 
 */
void quit()
{
    int retval;
    /// IO manager
    retval = io_deinit(io);
    if(retval != ERROR_OK)
    {
	err_log("Could not close IO correctly!");
    }

    /// IMU
    retval = imu_comm_deinit(imu);
    if(retval != ERROR_OK)
    {
	err_log("Could not close IMU correctly!");
    }

#if USE_GPS
    /// GPS
    gps_comm_deinit(gps);
#endif

    /// Kalman
    kalman_deinit(kalman);

    /// Motors
    retval = mot_deinit(mot);
    if(retval != ERROR_OK)
    {
	err_log("Could not close motor driver correctly!");
    }

    /// Control module
    control_deinit(ctrl);

    /// Path planner module
    pp_deinit(pp);

    /// Global vars
    uquad_mat_free(w);
    uquad_mat_free(wt);
    uquad_mat_free(x);
    uquad_mat_free(imu_data.acc);
    uquad_mat_free(imu_data.gyro);
    uquad_mat_free(imu_data.magn);

    // Logs
#if DEBUG
#if LOG_IMU_RAW
    uquad_logger_remove(log_imu_raw);
#endif //LOG_IMU_RAW
#if LOG_IMU_DATA
    uquad_logger_remove(log_imu_data);
#endif //LOG_IMU_DATA
#if LOG_IMU_AVG
    uquad_logger_remove(log_imu_avg);
#endif //LOG_IMU_AVG
#if LOG_W
    uquad_logger_remove(log_w);
#endif //LOG_W
#if LOG_W_CTRL
    uquad_logger_remove(log_w_ctrl);
#endif //LOG_W_CTRL
#if DEBUG_X_HAT
    uquad_logger_remove(log_x_hat);
    uquad_mat_free(x_hat_T);
#endif //DEBUG_X_HAT
#if DEBUG_KALMAN_INPUT
    uquad_logger_remove(log_kalman_in);
#endif
#endif //DEBUG
    uquad_logger_remove(log_tv);

    //TODO deinit everything?
    exit(retval);
}

#define IDLE_TIME_MS 1000
#define RETRY_IDLE_WAIT_MS 100
#define SLOW_LAND_STEP_MS 200
#define SLOW_LAND_STEP_W 10
void slow_land(void)
{
    int i, retval = ERROR_OK;
    double dtmp;
    w->m_full[0] = MOT_W_HOVER;
    w->m_full[1] = MOT_W_HOVER;
    w->m_full[2] = MOT_W_HOVER;
    w->m_full[3] = MOT_W_HOVER;
    for(i = 0; i < 5; ++i)
    {
	retval = mot_set_vel_rads(mot, w);
	if(retval != ERROR_OK)
	    break;
	else
	    sleep_ms(RETRY_IDLE_WAIT_MS);
    }
    sleep_ms(IDLE_TIME_MS);
    for(dtmp = MOT_W_HOVER;dtmp > MOT_W_IDLE;dtmp -= SLOW_LAND_STEP_W)
    {
	retval = mot_set_vel_rads(mot, w);
	if(retval != ERROR_OK)
	{
	    err_log("Failed to set speed when landing...");
	}
	sleep_ms(SLOW_LAND_STEP_MS);
    }
    retval = mot_stop(mot);
    quit();
}

void uquad_sig_handler(int signal_num){
    err_log_num("Caught signal:",signal_num);
    quit();
}

int main(int argc, char *argv[]){
    int retval = ERROR_OK, i;
    char * device_imu;
    double ts_jitter_rate = 0;
    imu_raw_t imu_frame;

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    if(argc<2)
    {
	err_log(UQUAD_HOW_TO);
	exit(1);
    }
    else
    {
	device_imu = argv[1];
    }

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Init
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// IO manager
    io = io_init();
    if(io==NULL)
    {
	quit_log_if(ERROR_FAIL,"io init failed!");
    }

    /// IMU
    imu = imu_comm_init(device_imu);
    if(imu == NULL)
    {
	quit_log_if(ERROR_FAIL,"imu init failed!");
    }

#if USE_GPS
    /// GPS
    gps = gps_comm_init();
    if(gps == NULL)
    {
	err_log("WARN: GPS not available!");
	//	quit_log_if(ERROR_FAIL,"gps init failed!");
    }
#endif

    /// Kalman
    kalman = kalman_init();
    if(kalman == NULL)
    {
	quit_log_if(ERROR_FAIL,"kalman init failed!");
    }

    /// Motors
    mot = mot_init();
    if(mot == NULL)
    {
	quit_log_if(ERROR_FAIL,"mot init failed!");
    }

    /// Control module
    ctrl = control_init();
    if(ctrl == NULL)
    {
	quit_log_if(ERROR_FAIL,"control init failed!");
    }

    /// Path planner module
    pp = pp_init();
    if(pp == NULL)
    {
	quit_log_if(ERROR_FAIL,"path planner init failed!");
    }

    /// Global vars
    w = uquad_mat_alloc(4,1);        // Current angular speed [rad/s]
    wt = uquad_mat_alloc(1,4);        // tranpose(w)
    x = uquad_mat_alloc(1,12);   // State vector
    imu_data.acc = uquad_mat_alloc(3,1);
    imu_data.gyro = uquad_mat_alloc(3,1);
    imu_data.magn = uquad_mat_alloc(3,1);

    if( x == NULL || w == NULL || wt == NULL ||
	imu_data.acc == NULL || imu_data.gyro == NULL || imu_data.magn == NULL)
    {
	err_log("Cannot run without x or w, aborting...");
	quit();
    }

    /// Logs
#if DEBUG
#if LOG_IMU_RAW
    log_imu_raw = uquad_logger_add(LOG_IMU_RAW_NAME);
    if(log_imu_raw == NULL)
    {
	err_log("Failed to open log_imu_raw!");
	quit();
    }
#endif //LOG_IMU_RAW
#if LOG_IMU_DATA
    log_imu_data = uquad_logger_add(LOG_IMU_DATA_NAME);
    if(log_imu_data == NULL)
    {
	err_log("Failed to open log_imu_data!");
	quit();
    }
#endif //LOG_IMU_DATA
#if LOG_IMU_AVG
    log_imu_avg = uquad_logger_add(LOG_IMU_AVG_NAME);
    if(log_imu_avg == NULL)
    {
	err_log("Failed to open log_imu_avg!");
	quit();
    }
#endif //LOG_IMU_AVG
#if LOG_W
    log_w = uquad_logger_add(LOG_W_NAME);
    if(log_w == NULL)
    {
	err_log("Failed to open log_w!");
	quit();
    }
#endif //LOG_W
#if LOG_W_CTRL
    log_w_ctrl = uquad_logger_add(LOG_W_CTRL_NAME);
    if(log_w_ctrl == NULL)
    {
	err_log("Failed to open log_w_ctrl!");
	quit();
    }
#endif //LOG_W_CTRL
#if DEBUG_X_HAT
    log_x_hat = uquad_logger_add(LOG_X_HAT_NAME);
    if(log_x_hat == NULL)
    {
	err_log("Failed to open x_hat!");
	quit();
    }
    x_hat_T = uquad_mat_alloc(1,12);
    if(x_hat_T == NULL)
    {
	err_log("Failed alloc x_hat_T!");
	quit();
    }
#endif //DEBUG_X_HAT
#if DEBUG_KALMAN_INPUT
    log_kalman_in = uquad_logger_add(LOG_KALMAN_IN_NAME);
    if(log_kalman_in == NULL)
    {
	err_log("Failed open kalman_in log!");
	quit();
    }
#endif
#endif //DEBUG
    log_tv = uquad_logger_add(LOG_TV_NAME);
    if(log_tv == NULL)
    {
	err_log("Failed to open tv_log!");
	quit();
    }

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Register IO devices
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // imu
    int imu_fds;
    retval = imu_comm_get_fds(imu,& imu_fds);
    quit_log_if(retval,"Failed to get imu fds!!");
    retval = io_add_dev(io,imu_fds);
    quit_log_if(retval,"Failed to add imu to dev list");
#if USE_GPS
    // gps
    int gps_fds;
    if(gps != NULL)
    {
	gps_fds = gps_comm_get_fd(gps);
	retval = io_add_dev(io,gps_fds);
	quit_log_if(retval,"Failed to add gps to dev list");
    }
#endif
    // stdin
    retval = io_add_dev(io,STDIN_FILENO);
    quit_log_if(retval, "Failed to add stdin to io list");

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Startup your engines...
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    retval = mot_set_idle(mot);
    quit_log_if(retval, "Failed to startup motors!");

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Poll n read loop
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    uquad_bool_t
	read = false,
	write = false,
	imu_update = false,
	reg_stdin = true;
    uquad_bool_t gps_update = false;
#if USE_GPS
    uquad_bool_t reg_gps = (gps == NULL)?false:true;
    struct timeval tv_gps_diff;
#endif
    int runs_imu = 0, runs_kalman = 0;
    int err_imu = ERROR_OK, err_gps = ERROR_OK;
    unsigned char tmp_buff[2];
    struct timeval
	tv_tmp, tv_diff,
	tv_last_m_cmd,
	tv_last_kalman,
	tv_last_imu,
	tv_gps_last,
	tv_start;
    gettimeofday(&tv_gps_last,NULL);
    gettimeofday(&tv_start,NULL);
#if TIMING
    struct timeval
	tv_pgm,
	tv_last_io_ok;
    gettimeofday(&tv_last_io_ok,NULL);
    gettimeofday(&tv_pgm,NULL);
#endif
#if TIMING && TIMING_IMU
    struct timeval tv_last_imu_read, tv_imu_start, tv_imu_diff;
    gettimeofday(&tv_last_imu_read,NULL);
#endif
    int count_err = 0, count_ok = FIXED;
    retval = ERROR_OK;
    //    poll_n_read:
    while(1){
	if((runs_imu > STARTUP_RUNS) &&
	   (retval != ERROR_OK  ||
	    err_imu != ERROR_OK ||
	    err_gps != ERROR_OK))
	{
	    count_ok = 0;
	    if(count_err++ > MAX_ERRORS)
	    {
		err_log("Too many errors! Aborting...");
		slow_land();
		/// program ends here
	    }
	}
	else
	{
	    if(count_ok < FIXED)
	    {
		count_ok++;
	    }
	    else if(count_err > 0)
	    {
		// forget abour error
		err_log_num("Recovered! Errors:",count_err);
		count_err = 0;
	    }
	}
	// reset error/update indicators
	imu_update = false;
	err_imu = ERROR_OK;
	gps_update = false;
	err_gps = ERROR_OK;
	retval = ERROR_OK;

	retval = io_poll(io);
	quit_log_if(retval,"io_poll() error");

	/// -- -- -- -- -- -- -- --
	/// Check IMU updates
	/// -- -- -- -- -- -- -- --
	retval = io_dev_ready(io,imu_fds,&read,&write);
	quit_log_if(retval,"io_dev_ready() error");
	if(read)
	{
#if TIMING && TIMING_IO
	    gettimeofday(&tv_tmp,NULL);
	    err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_io_ok);
	    if(err_imu < 0)
	    {
		err_log("Timing error!");
	    }
	    gettimeofday(&tv_last_io_ok,NULL);
	    gettimeofday(&tv_pgm,NULL);
	    printf("IO:\t%ld\t\t%ld.%06ld\n", tv_diff.tv_usec,
		   tv_pgm.tv_sec - tv_start.tv_sec,
		   tv_pgm.tv_usec);
#endif
#if TIMING && TIMING_IMU
	    gettimeofday(&tv_imu_start,NULL);
#endif

	    err_imu = imu_comm_read(imu, &imu_update);
	    log_n_jump(err_imu,end_imu,"imu_comm_read() failed!");
	    if(!imu_update)
	    {
		goto end_imu;
	    }
	    imu_update = false; // data may not be of direct use, may be calib

#if LOG_IMU_RAW || LOG_IMU_DATA
	    err_imu = imu_comm_get_raw_latest(imu,&imu_frame);
	    log_n_jump(err_imu,end_imu,"could not get new frame...");
#if LOG_IMU_RAW
	    err_imu= imu_comm_print_raw(&imu_frame, log_imu_raw);
	    log_n_jump(err_imu,end_imu,"could not print new raw frame...");
#endif // LOG_IMU_RAW
#if LOG_IMU_DATA
	    err_imu = imu_comm_raw2data(imu, &imu_frame, &imu_data);
	    log_n_jump(err_imu,end_imu,"could not convert new raw...");
	    err_imu = imu_comm_print_data(&imu_data, log_imu_data);
	    log_n_jump(err_imu,end_imu,"could not print new data...");
#endif // LOG_IMU_DATA
#endif // LOG_IMU_RAW || LOG_IMU_DATA

#if TIMING && TIMING_IMU
	    gettimeofday(&tv_tmp,NULL);
	    err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_imu_read);
	    if(err_imu < 0)
	    {
		err_log("Timing error!");
	    }
	    err_imu = uquad_timeval_substract(&tv_imu_diff,tv_tmp,tv_imu_start);
	    if(err_imu < 0)
	    {
		err_log("Timing error!");
	    }
	    gettimeofday(&tv_last_imu_read,NULL);
	    gettimeofday(&tv_pgm,NULL);
	    printf("IMU:\t%ld\tDELAY:\t%ld\t\t%ld.%06ld\n",
		   tv_diff.tv_usec, tv_imu_diff.tv_usec,
		   tv_pgm.tv_sec - tv_start.tv_sec,
		   tv_pgm.tv_usec);
#endif

	    /// discard first samples
	    if(!(runs_imu > STARTUP_RUNS))
	    {
		++runs_imu;
		if(runs_imu == STARTUP_RUNS)
		{
		    gettimeofday(&tv_last_imu,NULL);
		    err_imu = uquad_timeval_substract(&tv_diff,tv_last_imu,tv_start);
		    if(err_imu < 0)
		    {
			log_n_jump(err_imu,end_imu,"Absurd IMU startup time!");
		    }
		    err_imu = ERROR_OK; // clear timing info
		    err_log_tv("IMU startup completed in ", tv_diff);
		    ++runs_imu; // so re-entry doesn't happen
		}
		goto end_imu;
	    }

	    /// check calibration status
	    if(imu_comm_get_status(imu) == IMU_COMM_STATE_CALIBRATING)
		// if calibrating, then data should not be used.
		goto end_imu;
	    else if(!imu_comm_calib_estim(imu))
	    {
		// if no calibration estim exists, build one.
		err_imu = imu_comm_calibration_start(imu);
		log_n_jump(err_imu,end_imu,"Failed to start calibration!");
		goto end_imu;
	    }

	    /// Get new unread data
	    if(!imu_comm_unread(imu) || !imu_comm_avg_ready(imu))
	    {
		// we only used averaged data
		goto end_imu;
	    }

	    err_imu = imu_comm_get_avg_unread(imu,&imu_data);
	    log_n_jump(err_imu,end_imu,"IMU did not have new avg!");
#if LOG_IMU_AVG
	    err_imu = imu_comm_print_data(&imu_data, log_imu_avg);
	    log_n_jump(err_imu,end_imu,"Failed to log imu avg!");
#endif // LOG_IMU_AVG

	    gettimeofday(&tv_tmp,NULL);
	    err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_imu);
	    if(err_imu < 0)
	    {
		log_n_jump(ERROR_TIMING,end_imu,"Timing error!");
	    }

	    // store time since last IMU sample
	    imu_data.timestamp = tv_diff;

	    /// new data will be useful!
	    imu_update = true;
	    gettimeofday(&tv_last_imu,NULL);

	    end_imu:;
	    // will jump here if something went wrong during IMU reading
	}//if(read)

#if USE_GPS
	/// -- -- -- -- -- -- -- --
	/// Check GPS updates
	/// -- -- -- -- -- -- -- --
	if(reg_gps)
	{
	    err_gps = io_dev_ready(io,gps_fds,&read,&write);
	    if(read)
	    {
		err_gps = gps_comm_read(gps);
		log_n_jump(err_gps,end_gps,"GPS had no data!");
		if(gps_comm_get_status(gps))
		{
		    if(runs >= STARTUP_RUNS)
			// ignore startup data
			gps_update = true;
		    gettimeofday(&tv_tmp,NULL);
		    err_gps = uquad_timeval_substract(&tv_gps_diff,tv_tmp,tv_gps_last);
		    gettimeofday(&tv_gps_last,NULL);
		    fprintf(stdout,"\tlat:%f\n\tlon:%f\n\talt:%f\n\ttimestamp:%f\t%ld.%06ld\n\n",
			    gps->fix.latitude,
			    gps->fix.longitude,
			    gps->fix.altitude,
			    gps->fix.time,
			    tv_gps_diff.tv_sec,tv_gps_diff.tv_usec);
		}
	    }
	    end_gps:;
	    // will jump here if something went wrong during GPS reading
	}
#endif

	/// -- -- -- -- -- -- -- --
	/// check if new data
	/// -- -- -- -- -- -- -- --
	if(!imu_update && !gps_update)
	    continue;

#if IMU_COMM_FAKE
	// simulate delay (no delay when reading from txt)
	sleep_ms(10);
#endif

	/// -- -- -- -- -- -- -- --
	/// Update state estimation
	/// -- -- -- -- -- -- -- --
	gettimeofday(&tv_tmp,NULL);
	retval = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_kalman);
	if(retval < 0)
	{
	    log_n_continue(ERROR_TIMING,"Absurd timing!");
	}
#if TIMING && TIMING_KALMAN
	gettimeofday(&tv_pgm,NULL);
	printf("KALMAN:\t%ld\t\t%ld.%06ld\n", tv_diff.tv_usec,
	       tv_pgm.tv_sec - tv_start.tv_sec,
	       tv_pgm.tv_usec);
#endif
	/// Check sampling period jitter
	retval = in_range_us(tv_diff, TS_MIN, TS_MAX);
	static unsigned long kalman_loops = 0;
	kalman_loops = (kalman_loops+1)%65536;// avoid overflow
	if(retval != 0)
	{
	    static unsigned long ts_errors = 0;
	    ts_jitter_rate = 100*((double)ts_errors++/(kalman_loops+1));
	    if(ts_jitter_rate > TS_JITTER_RATE_MAX)
	    {
		fprintf(stderr,"Jitter rate: %0.2lf%%\t",ts_jitter_rate);
		err_log_tv("TS supplied to Kalman out of range!:",tv_diff);
	    }
	    tv_diff.tv_usec = (retval > 0) ? TS_MAX:TS_MIN;
	}
	/// Mark time when we run Kalman
	gettimeofday(&tv_last_kalman,NULL);
	if(runs_kalman > STARTUP_KALMAN)
	    // use real w
	    retval = uquad_kalman(kalman,
				  mot->w_curr,
				  &imu_data,
				  tv_diff.tv_usec);
	else
	    // use w from setpoint
	    retval = uquad_kalman(kalman,
				  pp->sp->w,
				  &imu_data,
				  tv_diff.tv_usec);
	log_n_continue(retval,"Kalman update failed");

#if DEBUG
#if DEBUG_KALMAN_INPUT
	fprintf(log_kalman_in, "%lu\t",tv_diff.tv_usec);
	retval = imu_comm_print_data(&imu_data, log_kalman_in);
#endif //DEBUG_KALMAN_INPUT
#if DEBUG_X_HAT
	retval = uquad_mat_transpose(x_hat_T, kalman->x_hat);
	uquad_mat_dump(x_hat_T,log_x_hat);
#endif //DEBUG_X_HAT
#endif //DEBUG
	if(!(runs_kalman > STARTUP_KALMAN))
	{
	    /**
	     * Startup:
	     *   - Kalman estimator.
	     *   - Ramp motors.
	     */
	    ++runs_kalman;
	    if(runs_kalman == 1)
	    {
		/**
		 * Use current IMU calibration to set
		 * yaw == 0, this will keep us looking forward
		 * when hovering.
		 */
		if(pp->pt != HOVER)
		{
		    quit_log_if(retval,"ERR: theta (Yaw) set to IMU calibration"\
				"is only valid when in HOVER mode");
		}
		retval = imu_comm_raw2data(imu, &imu->calib.null_est, &imu_data);
		quit_log_if(retval,"Failed to correct setpoint!");
		pp->sp->x->m_full[5] = imu_data.magn->m_full[2];
	    }
	    if(runs_kalman == STARTUP_KALMAN)
	    {
		gettimeofday(&tv_last_m_cmd,NULL);
		retval = uquad_timeval_substract(&tv_diff,tv_last_m_cmd,tv_start);
		if(retval < 0)
		{
		    err_log("Absurd Kalman startup time!");
		    continue;
		}
		retval = ERROR_OK;
		err_log_tv("Kalman startup completed in ", tv_diff);
		++runs_kalman; // so re-entry doesn't happen
	    }
	    else
	    {
		// Ramp up motors to MOT_W_HOVER, avoid step
		for(i = 0; i < MOT_C; ++i)
		    w->m_full[i] = MOT_W_IDLE +
			runs_kalman*(MOT_W_STARTUP_RANGE/STARTUP_KALMAN);
		retval = mot_set_vel_rads(mot, w);
		log_n_continue(retval,"Failed to set motor speed!");
		continue;
	    }
	}

	/// -- -- -- -- -- -- -- --
	/// Update setpoint
	/// -- -- -- -- -- -- -- --
	retval = pp_update_setpoint(pp, kalman->x_hat);
	log_n_continue(retval,"Kalman update failed");

	/// -- -- -- -- -- -- -- --
	/// Run control
	/// -- -- -- -- -- -- -- --
	retval = control(ctrl, w, kalman->x_hat, pp->sp);
	log_n_continue(retval,"Control failed!");
#if DEBUG && LOG_W_CTRL
	retval = uquad_mat_transpose(wt,w);
	uquad_mat_dump(wt,log_w_ctrl);
#endif

	/// -- -- -- -- -- -- -- --
	/// Set motor speed
	/// -- -- -- -- -- -- -- --
	gettimeofday(&tv_tmp,NULL);
	uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_m_cmd);
	if (tv_diff.tv_usec > MOT_UPDATE_T)
	{
	    gettimeofday(&tv_last_m_cmd,NULL);

	    /// Update motor controller
	    retval = mot_set_vel_rads(mot, w);
	    log_n_continue(retval,"Failed to set motor speed!");
#if DEBUG && LOG_W
	    fprintf(log_w, "%ld\t", tv_diff.tv_usec);
	    retval = uquad_mat_transpose(wt,mot->w_curr);
	    uquad_mat_dump(wt,log_w);
#endif
	}

	/// -- -- -- -- -- -- -- --
	/// Check stdin
	/// -- -- -- -- -- -- -- --
	if(reg_stdin)
	{
	    retval = io_dev_ready(io,STDIN_FILENO,&read,&write);
	    quit_log_if(retval,"io_dev_ready() error");
	    if(read){
		retval = fread(tmp_buff,1,1,stdin);
		if(retval<=0)
		    fprintf(stdout,"\nNo user input!!\n\n");
		else
		{
		    gettimeofday(&tv_tmp,NULL);
		    retval = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
		    log_tv(log_tv, "RET:", tv_diff);
		}
	    }
	    retval = ERROR_OK;
	}
	fflush(stdout);
    }

    return 0;
}
    
