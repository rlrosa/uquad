#include <uquad_error_codes.h> // DEBUG is defined here
#if DEBUG // The following define will affect includes
#define TIMING             0
#define TIMING_KALMAN      0
#define TIMING_IMU         0
#define TIMING_IO          0
#define LOG_W              1
#define LOG_W_CTRL         1
#define LOG_IMU_RAW        1
#define LOG_IMU_DATA       1
#define LOG_IMU_AVG        1
#define DEBUG_X_HAT        1
#define LOG_GPS            0
#define DEBUG_KALMAN_INPUT 1
#define LOG_TV             1
#define LOG_BUKAKE         0
#endif

#define GPS_FAKE           1 // Simulate GPS data (use zeros)

#if LOG_BUKAKE
#define LOG_BUKAKE_STDOUT  1
#endif

#define W_SP_STEP 1.0L
#define W_SP_INC  'i'
#define W_SP_DEC  'k'

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

#define UQUAD_HOW_TO   "./main <imu_device> /path/to/log/"
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
#define TS_JITTER          2000L // Max jitter accepted
#define TS_ERROR_WAIT      10    // Wait 10 errors before logging again
#define TS_MAX             (TS_DEFAULT_US + TS_JITTER)
#define TS_MIN             (TS_DEFAULT_US - TS_JITTER)

#define LOG_DIR_DEFAULT    "/media/sda1/"

#define LOG_W_NAME         "w"
#define LOG_W_CTRL_NAME    "w_ctrl"

#define LOG_IMU_RAW_NAME   "imu_raw"
#define LOG_IMU_DATA_NAME  "imu_data"
#define LOG_IMU_AVG_NAME   "imu_avg"
#define LOG_X_HAT_NAME     "x_hat"
#define LOG_KALMAN_IN_NAME "kalman_in"
#define LOG_GPS_NAME       "gps"
#define LOG_TV_NAME        "tv"
#define LOG_BUKAKE_NAME    "buk"

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
#if USE_GPS && !GPS_FAKE
static gps_t *gps;
#endif
/// Global var
uquad_mat_t *w, *wt;
uquad_mat_t *x;
imu_data_t imu_data;
#if USE_GPS
gps_comm_data_t *gps_dat;
#endif
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
#if LOG_GPS && USE_GPS
FILE *log_gps = NULL;
#endif // LOG_GPS && USE_GPS
#if LOG_BUKAKE && !LOG_BUKAKE_STDOUT
FILE *log_bukake = NULL;
#endif //LOG_BUKAKE
#if LOG_TV
FILE *log_tv = NULL;
#endif // LOG_TV
#endif //DEBUG

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

#if USE_GPS && !GPS_FAKE
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
#if USE_GPS && !GPS_FAKE
    gps_comm_data_free(gps_dat);
#endif

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
#if LOG_GPS && USE_GPS
    uquad_logger_remove(log_gps);
#endif //LOG_GPS && USE_GPS
#if LOG_BUKAKE
#if !LOG_BUKAKE_STDOUT
    uquad_logger_remove(log_bukake);
#else
#define log_bukake stdout
#endif // !LOG_BUKAKE_STDOUT
#endif // LOG_BUKAKE
#if LOG_TV
    uquad_logger_remove(log_tv);
#endif // LOG_TV
#endif //DEBUG

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
    int
	retval = ERROR_OK,
	i;
    char
	*device_imu,
	*log_path;
    double
	dtmp;
    unsigned long
	kalman_loops = 0,
	ts_error_wait = 0;

#if LOG_IMU_RAW || LOG_IMU_DATA
    imu_raw_t imu_frame;
#endif // LOG_IMU_RAW || LOG_IMU_DATA

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
	if(argc < 3)
	    log_path = LOG_DIR_DEFAULT;
	else
	    log_path   = argv[2];
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

#if USE_GPS && !GPS_FAKE
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
    retval = imu_data_alloc(&imu_data);
    quit_if(retval);

    if( x == NULL || w == NULL || wt == NULL)
    {
	err_log("Cannot run without x or w, aborting...");
	quit();
    }

#if USE_GPS
    gps_dat = gps_comm_data_alloc();
    if(gps_dat == NULL)
    {
	err_log("Failed to allocate GPS!...");
	quit();
    }
#endif

    /// Logs
#if DEBUG
#if LOG_IMU_RAW
    log_imu_raw = uquad_logger_add(LOG_IMU_RAW_NAME, log_path);
    if(log_imu_raw == NULL)
    {
	err_log("Failed to open log_imu_raw!");
	quit();
    }
#endif //LOG_IMU_RAW
#if LOG_IMU_DATA
    log_imu_data = uquad_logger_add(LOG_IMU_DATA_NAME, log_path);
    if(log_imu_data == NULL)
    {
	err_log("Failed to open log_imu_data!");
	quit();
    }
#endif //LOG_IMU_DATA
#if LOG_IMU_AVG
    log_imu_avg = uquad_logger_add(LOG_IMU_AVG_NAME, log_path);
    if(log_imu_avg == NULL)
    {
	err_log("Failed to open log_imu_avg!");
	quit();
    }
#endif //LOG_IMU_AVG
#if LOG_W
    log_w = uquad_logger_add(LOG_W_NAME, log_path);
    if(log_w == NULL)
    {
	err_log("Failed to open log_w!");
	quit();
    }
#endif //LOG_W
#if LOG_W_CTRL
    log_w_ctrl = uquad_logger_add(LOG_W_CTRL_NAME, log_path);
    if(log_w_ctrl == NULL)
    {
	err_log("Failed to open log_w_ctrl!");
	quit();
    }
#endif //LOG_W_CTRL
#if DEBUG_X_HAT
    log_x_hat = uquad_logger_add(LOG_X_HAT_NAME, log_path);
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
    log_kalman_in = uquad_logger_add(LOG_KALMAN_IN_NAME, log_path);
    if(log_kalman_in == NULL)
    {
	err_log("Failed open kalman_in log!");
	quit();
    }
#endif
#if LOG_GPS && USE_GPS
    log_gps = uquad_logger_add(LOG_GPS_NAME, log_path);
    if(log_gps == NULL)
    {
	err_log("Failed open kalman_in log!");
	quit();
    }
#endif //LOG_GPS && USE_GPS
#if LOG_BUKAKE && !LOG_BUKAKE_STDOUT
    log_bukake = uquad_logger_add(LOG_BUKAKE_NAME, log_path);
    if(log_bukake == NULL)
    {
	err_log("Failed open kalman_in log!");
	quit();
    }
#endif //LOG_BUKAKE && !LOG_BUKAKE_STDOUT
#if LOG_TV
    log_tv = uquad_logger_add(LOG_TV_NAME, log_path);
    if(log_tv == NULL)
    {
	err_log("Failed to open tv_log!");
	quit();
    }
#endif // LOG_TV
#endif //DEBUG

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Register IO devices
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // imu
    int imu_fds;
    retval = imu_comm_get_fds(imu,& imu_fds);
    quit_log_if(retval,"Failed to get imu fds!!");
    retval = io_add_dev(io,imu_fds);
    quit_log_if(retval,"Failed to add imu to dev list");
#if USE_GPS && !GPS_FAKE
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
#if USE_GPS && !GPS_FAKE
    uquad_bool_t reg_gps = (gps == NULL)?false:true;
#endif
    int runs_imu = 0, runs_kalman = 0;
    int err_imu = ERROR_OK, err_gps = ERROR_OK;
    unsigned char tmp_buff[2];
    struct timeval
	tv_tmp, tv_diff,
	tv_last_m_cmd,
	tv_last_ramp,
	tv_last_kalman,
	tv_last_imu,
	tv_gps_last,
	tv_start;
    gettimeofday(&tv_start,NULL);
    gettimeofday(&tv_last_ramp,NULL);
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
	//gps_update = false; // This is cleared within the loop
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
	    err_imu = gettimeofday(&tv_tmp,NULL);
	    err_log_std(err_imu);
	    err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_io_ok);
	    if(err_imu < 0)
	    {
		err_log("Timing error!");
	    }
	    err_imu = gettimeofday(&tv_last_io_ok,NULL);
	    err_log_std(err_imu);
	    err_imu = gettimeofday(&tv_pgm,NULL);
	    err_log_std(err_imu);
	    printf("IO:\t%ld\t\t%ld.%06ld\n", tv_diff.tv_usec,
		   tv_pgm.tv_sec - tv_start.tv_sec,
		   tv_pgm.tv_usec);
#endif
#if TIMING && TIMING_IMU
	    err_imu = gettimeofday(&tv_imu_start,NULL);
	    err_log_std(err_imu);
#endif

	    err_imu = imu_comm_read(imu, &imu_update);
	    log_n_jump(err_imu,end_imu,"imu_comm_read() failed!");
	    if(!imu_update)
	    {
		goto end_imu;
	    }
	    imu_update = false; // data may not be of direct use, may be calib
#if IMU_COMM_FAKE
	    // simulate delay (no delay when reading from txt)
	    sleep_ms(10);
#endif

#if LOG_IMU_RAW || LOG_IMU_DATA
	    err_imu = gettimeofday(&tv_tmp,NULL);
	    err_log_std(err_imu);
	    err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
	    if(err_imu < 0)
	    {
		err_log("Timing error!");
	    }
	    err_imu = imu_comm_get_raw_latest(imu,&imu_frame);
	    log_n_jump(err_imu,end_imu,"could not get new frame...");
#if LOG_IMU_RAW
	    log_tv_only(log_imu_raw,tv_diff);
	    err_imu= imu_comm_print_raw(&imu_frame, log_imu_raw);
	    log_n_jump(err_imu,end_imu,"could not print new raw frame...");
	    fflush(log_imu_raw);
#endif // LOG_IMU_RAW
#if LOG_IMU_DATA
	    err_imu = imu_comm_raw2data(imu, &imu_frame, &imu_data);
	    log_n_jump(err_imu,end_imu,"could not convert new raw...");
	    log_tv_only(log_imu_data,tv_diff);
	    err_imu = imu_comm_print_data(&imu_data, log_imu_data);
	    log_n_jump(err_imu,end_imu,"could not print new data...");
	    fflush(log_imu_data);
#endif // LOG_IMU_DATA
#endif // LOG_IMU_RAW || LOG_IMU_DATA

#if TIMING && TIMING_IMU
	    err_imu = gettimeofday(&tv_tmp,NULL);
	    err_log_std(err_imu);
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
	    err_imu = gettimeofday(&tv_last_imu_read,NULL);
	    err_log_std(err_imu);
	    err_imu = gettimeofday(&tv_pgm,NULL);
	    err_log_std(err_imu);
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
		    err_imu = gettimeofday(&tv_gps_last,NULL);
		    err_log_std(err_imu);
		    err_imu = gettimeofday(&tv_last_imu,NULL);
		    err_log_std(err_imu);
		    tv_gps_last    = tv_last_imu;
		    tv_last_kalman = tv_last_imu;
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

	    gettimeofday(&tv_tmp,NULL);

	    err_imu = imu_comm_get_avg_unread(imu,&imu_data);
	    log_n_jump(err_imu,end_imu,"IMU did not have new avg!");
#if LOG_IMU_AVG
	    uquad_timeval_substract(&tv_diff, tv_tmp, tv_start);
	    log_tv_only(log_imu_avg, tv_diff);
	    err_imu = imu_comm_print_data(&imu_data, log_imu_avg);
	    log_n_jump(err_imu,end_imu,"Failed to log imu avg!");
#endif // LOG_IMU_AVG

	    err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_imu);
	    if(err_imu < 0)
	    {
		log_n_jump(ERROR_TIMING,end_imu,"Timing error!");
	    }

	    // store time since last IMU sample
	    imu_data.timestamp = tv_diff;

	    /// new data will be useful!
	    imu_update = true;
	    err_imu = gettimeofday(&tv_last_imu,NULL);
	    err_log_std(err_imu);

	    end_imu:;
	    // will jump here if something went wrong during IMU reading
	}//if(read)

#if USE_GPS
#if !GPS_FAKE
	/// -- -- -- -- -- -- -- --
	/// Check GPS updates
	/// -- -- -- -- -- -- -- --
	if(reg_gps && !gps_update)
	{
	    err_gps = io_dev_ready(io,gps_fds,&read,&write);
	    if(read)
	    {
		err_gps = gps_comm_read(gps);
		log_n_jump(err_gps,end_gps,"GPS had no data!");
		if(runs_imu >= STARTUP_RUNS && gps_comm_3dfix(gps))
		    // ignore startup data
		    gps_update = true;
		else
		    goto end_gps;
		// Use latest IMU update to estimate speed from GPS data
		err_gps = gps_comm_get_data_unread(gps, gps_dat, NULL);
		log_n_jump(err_gps,end_gps,"Failed to get GPS data!");
		
		gettimeofday(&tv_tmp,NULL);
		err_gps = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
#if LOG_GPS
		log_tv_only(log_gps, tv_diff);
		gps_comm_dump(gps, gps_dat, log_gps);
#endif // LOG_GPS
		tv_gps_last = tv_tmp;
	    }
	    end_gps:;
	    // will jump here if something went wrong during GPS reading
	}
#else // GPS_FAKE
	if(!gps_update)
	{
	    gettimeofday(&tv_tmp,NULL);
	    retval = uquad_timeval_substract(&tv_diff, tv_tmp, tv_gps_last);
	    if(tv_diff.tv_sec > 1 && (runs_imu > STARTUP_RUNS))
	    {
		// gps_dat is set to 0 when allocated, so just use it.
		gps_update = true;
		tv_gps_last = tv_tmp;
		if(pp->pt != HOVER)
		{
		    quit_log_if(ERROR_GPS, "Fake GPS does not make sense if not hovering!");
		}
	    }
	    else
	    {
		gps_update = false;
	    }
	    retval = ERROR_OK; // clear retval
	}
#endif // GPS_FAKE
#endif // USE_GPS

	/// -- -- -- -- -- -- -- --
	/// check if new data
	/// -- -- -- -- -- -- -- --
	if(!imu_update)
	    /**
	     * We don't check gps_update here.
	     * If gps_update && !imu_update, then
	     * wait until imu_update to avoid unstable T_s.
	     *   T_gps = 1s
	     *   T_imu = 10ms
	     * so use approx. that T_gps+T_imu ~ T_gps.
	     */
	    continue;

	/// -- -- -- -- -- -- -- --
	/// Startup Kalman estimator
	/// -- -- -- -- -- -- -- --
	if(runs_kalman == 0)
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
	    pp->sp->x->m_full[SV_THETA] = imu_data.magn->m_full[2];
	    /// Start kalman from calibration value
	    kalman->x_hat->m_full[SV_PSI]   = imu_data.magn->m_full[0];
	    kalman->x_hat->m_full[SV_PHI]   = imu_data.magn->m_full[1];
	    kalman->x_hat->m_full[SV_THETA] = imu_data.magn->m_full[2];
	    retval = imu_comm_print_data(&imu_data, stderr);
	    if(retval != ERROR_OK)
	    {
		err_log("Failed to print IMU calibration!");
	    }
	    retval = ERROR_OK;// ignore error
	    fflush(stderr);
	}

	/// -- -- -- -- -- -- -- --
	/// Update state estimation
	/// -- -- -- -- -- -- -- --
	gettimeofday(&tv_tmp,NULL); // will be used to set tv_last_kalman
	if(runs_kalman == 0)
	{
	    // First time here, use fake timestamp
	    tv_diff.tv_usec = TS_DEFAULT_US;
	}
	else
	{
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
	    kalman_loops = (kalman_loops+1)%32768;// avoid overflow
	    if(retval != 0)
	    {
		if(ts_error_wait == 0)
		{
		    // Avoid saturating log
		    err_log_tv("TS supplied to Kalman out of range!:",tv_diff);
		    ts_error_wait = TS_ERROR_WAIT;
		}
		ts_error_wait--;
		/// Lie to kalman, avoid large drifts
		tv_diff.tv_usec = (retval > 0) ? TS_MAX:TS_MIN;
	    }
	    else
	    {
		// Print next T_s error immediately
		ts_error_wait = 0;
	    }
	}
	if(runs_kalman > STARTUP_KALMAN)
	{
	    // use real w
	    retval = uquad_kalman(kalman,
				  mot->w_curr,
				  &imu_data,
				  tv_diff.tv_usec);
	    log_n_continue(retval,"Inertial Kalman update failed");
	}
	else
	{
	    // use w from setpoint
	    retval = uquad_kalman(kalman,
				  pp->sp->w,
				  &imu_data,
				  tv_diff.tv_usec);
	    log_n_continue(retval,"Inertial Kalman update failed");
	}
	/// Mark time when we run Kalman
	tv_last_kalman = tv_tmp;
#if USE_GPS
	if(gps_update)
	{
	    retval = uquad_kalman_gps(kalman, gps_dat);
	    log_n_continue(retval,"GPS Kalman update failed");
	    gps_update = false; // Clear gps status
	}
#endif // USE_GPS


#if DEBUG
#if DEBUG_KALMAN_INPUT
	uquad_timeval_substract(&tv_diff,tv_last_kalman,tv_start);
	log_tv_only(log_kalman_in,tv_diff);
	retval = imu_comm_print_data(&imu_data, log_kalman_in);
	fflush(log_kalman_in);
#endif //DEBUG_KALMAN_INPUT
#if DEBUG_X_HAT
	retval = uquad_mat_transpose(x_hat_T, kalman->x_hat);
	uquad_mat_dump(x_hat_T,log_x_hat);
	fflush(log_x_hat);
#endif //DEBUG_X_HAT
#endif //DEBUG
	if(!(runs_kalman > STARTUP_KALMAN))
	{
	    /**
	     * Startup:
	     *   - Ramp motors.
	     */
	    ++runs_kalman;
	    if(runs_kalman == STARTUP_KALMAN)
	    {
		gettimeofday(&tv_tmp,NULL);
		retval = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
		if(retval < 0)
		{
		    err_log("Absurd Kalman startup time!");
		    continue;
		}
		retval = ERROR_OK;
		// save to error log
		err_log_tv("Kalman startup completed in ", tv_diff);
#if LOG_TV
		// save to RET log, add end of line
		log_tv_only(log_tv,tv_diff);
		log_tv(log_tv, "Kalman startup completed in ", tv_diff);
#endif // LOG_TV
		++runs_kalman; // so re-entry doesn't happen
	    }
	    else
	    {
		// Ramp up motors to MOT_W_HOVER, avoid step
		retval = gettimeofday(&tv_tmp,NULL);
		err_log_std(retval);
		for(i = 0; i < MOT_C; ++i)
		    w->m_full[i] = MOT_W_IDLE +
			runs_kalman*(MOT_W_STARTUP_RANGE/STARTUP_KALMAN);
		retval = mot_set_vel_rads(mot, w);
		log_n_continue(retval,"Failed to set motor speed!");
#if LOG_W
		uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
		log_tv_only(log_w,tv_diff);
		retval = uquad_mat_transpose(wt,w);
		log_n_continue(retval,"Failed to transpose!");
		uquad_mat_dump(wt,log_w);
		fflush(log_w);
#endif // LOG_W
		tv_last_ramp  = tv_tmp;
		tv_last_m_cmd = tv_tmp;
		retval = gettimeofday(&tv_last_ramp,NULL);
		log_n_continue(retval,"Failed to update ramp timer!");
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
	gettimeofday(&tv_tmp,NULL);
	uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_m_cmd);
	retval = control(ctrl, w, kalman->x_hat, pp->sp, (double)tv_diff.tv_usec);
	log_n_continue(retval,"Control failed!");
#if DEBUG && LOG_W_CTRL
	retval = uquad_mat_transpose(wt,w);
	uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
	log_tv_only(log_w_ctrl,tv_diff);
	uquad_mat_dump(wt,log_w_ctrl);
	fflush(log_w_ctrl);
#endif

	/// -- -- -- -- -- -- -- --
	/// Set motor speed
	/// -- -- -- -- -- -- -- --
	gettimeofday(&tv_tmp,NULL);
	uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_m_cmd);
	if (tv_diff.tv_usec > MOT_UPDATE_T || tv_diff.tv_sec > 1)
	{
	    /// Update motor controller
	    retval = mot_set_vel_rads(mot, w);
	    log_n_continue(retval,"Failed to set motor speed!");
#if DEBUG && LOG_W
	    uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
	    log_tv_only(log_w,tv_diff);
	    retval = uquad_mat_transpose(wt,mot->w_curr);
	    uquad_mat_dump(wt,log_w);
	    fflush(log_w);
#endif

#if LOG_BUKAKE
	    uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
	    log_tv_only(log_bukake,tv_diff);
	    int ind; uquad_mat_t *xm = kalman->x_hat; int len = xm->r*xm->c;
	    for(ind = 0; ind < len; ++ind)
		log_double_only(log_bukake,xm->m_full[ind]);
	    fdatasync(fileno(log_bukake));
	    retval = uquad_mat_transpose(wt,mot->w_curr);
	    uquad_mat_dump(wt,log_bukake);
	    fdatasync(fileno(log_bukake));
#endif
	    tv_last_m_cmd = tv_tmp;
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
		{
		    err_log("No user input!!");
		}
		else
		{
		    gettimeofday(&tv_tmp,NULL);
		    retval = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
		    if(retval <= 0)
		    {
			err_log("Absurd timing!");
		    }
		    retval = ERROR_OK; // clear error
		    dtmp = 0.0;
		    switch(tmp_buff[0])
		    {
		    case W_SP_INC:
			dtmp = W_SP_STEP;
			break;
		    case W_SP_DEC:
			dtmp = -W_SP_STEP;
			break;
		    default:
			break;
		    }
		    if(dtmp != 0.0)
		    {
			for(i = 0; i < MOT_C; ++i)
			    pp->sp->w->m_full[i] += dtmp;
			// display on screen
			log_tv_only(stdout,tv_diff);
			log_double(stdout,"Current w_sp",pp->sp->w->m_full[0]);
			fflush(stdout);
		    }
#if LOG_TV
		    // save to log file
		    log_tv_only(log_tv, tv_diff);
		    log_double(log_tv,"Current w_sp",pp->sp->w->m_full[0]);
		    fflush(log_tv);
#endif
		}
	    }
	    retval = ERROR_OK;
	}
    }
    // never gets here
    return 0;
}
    
