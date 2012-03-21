#include <uquad_error_codes.h> // DEBUG is defined here
#if DEBUG // The following define will affect includes
#define TIMING 0
#define TIMING_KALMAN 0
#define TIMING_IMU 0
#define TIMING_IO 0
#define LOG_W 1
#define LOG_W_CTRL 1
#define DEBUG_X_HAT 1
#define DEBUG_KALMAN_INPUT 1
#endif

#include <uquad_types.h>
#include <macros_misc.h>
#include <uquad_aux_io.h>
#include <imu_comm.h>
#include <mot_control.h>
#include <uquad_kalman.h>
#include <control.h>
#include <path_planner.h>
#include <uquad_gps_comm.h>

#include <sys/signal.h> // for SIGINT and SIGQUIT
#include <unistd.h>     // for STDIN_FILENO

#define UQUAD_HOW_TO "./main <imu_device>"
#define MAX_ERRORS 20
#define STARTUP_RUNS 200
#define FIXED 3

#define LOG_W_NAME "w.log"
#define LOG_W_CTRL_NAME "w_ctrl.log"

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
static gps_t *gps;
/// Global var
uquad_mat_t *w, *wt;
uquad_mat_t *x;
imu_data_t imu_data;
/// Logs
#if DEBUG
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

    /// GPS
    gps_comm_deinit(gps);

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
#if LOG_W
    if(log_w != NULL)
	fclose(log_w);
#endif //LOG_W
#if LOG_W_CTRL
    if(log_w_ctrl != NULL)
	fclose(log_w_ctrl);
#endif //LOG_W_CTRL
#if DEBUG_X_HAT
    if(log_x_hat != NULL)
	fclose(log_x_hat);
    uquad_mat_free(x_hat_T);
#endif //DEBUG_X_HAT
#if DEBUG_KALMAN_INPUT
    if(log_kalman_in != NULL)
	fclose(log_kalman_in);
#endif
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
    for(dtmp = MOT_W_HOVER;dtmp > MOT_IDLE_W;dtmp -= SLOW_LAND_STEP_W)
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

    /// GPS
    gps = gps_comm_init();
    if(gps == NULL)
    {
	err_log("WARN: GPS not available!");
	//	quit_log_if(ERROR_FAIL,"gps init failed!");
    }

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
    if(mot == NULL)
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
    for(i=0; i < 4; ++i)
	w->m_full[i] = MOT_IDLE_W;
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
#if LOG_W
    log_w = fopen(LOG_W_NAME,"w");
    if(log_w == NULL)
    {
	err_log("Failed to open log_w!");
	quit();
    }
#endif //LOG_W
#if LOG_W_CTRL
    log_w_ctrl = fopen(LOG_W_CTRL_NAME,"w");
    if(log_w_ctrl == NULL)
    {
	err_log("Failed to open log_w_ctrl!");
	quit();
    }
#endif //LOG_W_CTRL
#if DEBUG_X_HAT
    log_x_hat = fopen("x_hat.log","w");
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
    log_kalman_in = fopen("kalman_in.log","w");
    if(log_kalman_in == NULL)
    {
	err_log("Failed open kalman_in log!");
	quit();
    }
#endif
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
    // gps
    int gps_fds;
    if(gps != NULL)
    {
	gps_fds = gps_comm_get_fd(gps);
	retval = io_add_dev(io,gps_fds);
	quit_log_if(retval,"Failed to add gps to dev list");
    }
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
	gps_update = false,
	reg_stdin = true,
	reg_gps = (gps == NULL)?false:true;
    int runs = 0;
    int err_imu = ERROR_OK, err_gps = ERROR_OK;
    unsigned char tmp_buff[2];
    struct timeval
	tv_tmp, tv_diff,
	tv_last_m_cmd,
	tv_last_kalman,
	tv_gps_last,
	tv_gps_diff;
    gettimeofday(&tv_last_kalman,NULL);
    gettimeofday(&tv_gps_last,NULL);
#if TIMING
    struct timeval
	tv_start,
	tv_pgm,
	tv_last_io_ok;
    gettimeofday(&tv_last_m_cmd,NULL);
    gettimeofday(&tv_last_io_ok,NULL);
    gettimeofday(&tv_pgm,NULL);
    gettimeofday(&tv_start,NULL);
#endif
#if TIMING && TIMING_IMU
    struct timeval tv_last_imu_read, tv_imu_start, tv_imu_diff;
    gettimeofday(&tv_last_imu_read,NULL);
#endif
    int count_err = 0, count_ok = FIXED;
    retval = ERROR_OK;
    //    poll_n_read:
    while(1){
	if((runs > STARTUP_RUNS) &&
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
		//		err_log_num("Recovered! Errors:",count_err);//TODO restore!
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
	    if(!imu_update)
	    {
		goto end_imu;
	    }

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

	    if(!imu_comm_avg_ready(imu) || runs < STARTUP_RUNS)
	    {
		// not enough samples yet.
		runs++;
		goto end_imu;
	    }
	    /// Get new unread data
	    err_imu = imu_comm_get_avg_unread(imu,&imu_data);
	    log_n_jump(err_imu,end_imu,"IMU did not have new avg!");

	    gettimeofday(&tv_tmp,NULL);
	    err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_kalman);
	    if(err_imu < 0)
	    {
		log_n_jump(ERROR_TIMING,end_imu,"Timing error!");
	    }
	    /// Get new state estimation
	    if(runs == STARTUP_RUNS)
		// the first time here, timing will not make sense, so use IMU
		tv_diff.tv_usec = imu_data.T_us;
	    // store time since last IMU sample
	    imu_data.timestamp = tv_diff;
	    end_imu:;
	    // will jump here if something went wrong during IMU reading
	}//if(read)

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

	if(!imu_update && !gps_update)
	    continue;

	/// -- -- -- -- -- -- -- --
	/// Update state estimation
	/// -- -- -- -- -- -- -- --
	retval = uquad_kalman(kalman,
			      mot->w_curr,
			      &imu_data,
			      imu_data.timestamp.tv_usec);
	log_n_continue(retval,"Kalman update failed");
	/// Mark time when we leave Kalman
	gettimeofday(&tv_last_kalman,NULL);
#if TIMING && TIMING_KALMAN
	gettimeofday(&tv_pgm,NULL);
	printf("KALMAN:\t%ld\t\t%ld.%06ld\n", tv_diff.tv_usec,
	       tv_pgm.tv_sec - tv_start.tv_sec,
	       tv_pgm.tv_usec);
#endif

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
	/// Get current set point
	retval = pp_update_setpoint(pp, kalman->x_hat);
	log_n_continue(retval,"Kalman update failed");

	/// Get control command
	retval = control(ctrl, w, kalman->x_hat, pp->sp);
	log_n_continue(retval,"Control failed!");
#if DEBUG && LOG_W_CTRL
	retval = uquad_mat_transpose(wt,w);
	uquad_mat_dump(wt,log_w_ctrl);
#endif

	/// -- -- -- -- -- -- -- --
	/// Run control
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
		    err_log("Will land and quit!");
		    slow_land();
		    // program ends here
		}
	    }
	}
	fflush(stdout);
    }

    return 0;
}
    
