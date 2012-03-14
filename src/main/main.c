#include <uquad_error_codes.h> // DEBUG is defined here
#include <uquad_types.h>
#include <macros_misc.h>
#include <uquad_aux_io.h>
#include <imu_comm.h>
#include <mot_control.h>
#include <uquad_kalman.h>
#include <control.h>
#include <path_planner.h>
//#include <uquad_gps_comm.h> //TODO add!

#include <sys/signal.h> // for SIGINT and SIGQUIT
#include <unistd.h>     // for STDIN_FILENO

#define UQUAD_HOW_TO "./main <imu_device>"
#define MAX_ERRORS 20
#define FIXED 3
#define LOG_W 1
#define LOG_W_NAME "w.log"

#define MOT_COMMAND_T_US MOT_UPDATE_MAX_US


/// Global structs
static imu_t *imu;
static kalman_io_t *kalman;
static uquad_mot_t *mot;
static io_t *io;
static ctrl_t *ctrl;
static path_planner_t *pp;
//static gps_t *gps; //TODO add
/// Global var
uquad_mat_t *w, *wt;
uquad_mat_t *x;
imu_data_t imu_data;
/// Logs
#if DEBUG
#if LOG_W
FILE *log_w = NULL;
#endif //LOG_W
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
	retval = mot_set_vel_rads(mot, w->m_full);
	if(retval != ERROR_OK)
	    break;
	else
	    sleep_ms(RETRY_IDLE_WAIT_MS);
    }
    sleep_ms(IDLE_TIME_MS);
    for(dtmp = MOT_W_HOVER;dtmp > MOT_W_IDLE;dtmp -= SLOW_LAND_STEP_W)
    {
	retval = mot_set_vel_rads(mot, w->m_full);
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
    int ret = ERROR_OK;
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
	w->m_full[i] = MOT_W_IDLE;
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
	err_log("Failed to open w_log!");
	quit();
    }
#endif //LOG_W
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
    uquad_bool_t read = false,write = false, imu_update = false;
    uquad_bool_t first_run = true;
    uquad_bool_t reg_stdin = true;
    unsigned char tmp_buff[2];
    struct timeval tv_last_m_cmd, tv_tmp, tv_diff;
    gettimeofday(&tv_last_m_cmd,NULL);
    int count_err = 0, count_ok = FIXED;
    retval = ERROR_OK;
    poll_n_read:
    while(1){
	if(!imu_update && retval != ERROR_OK)
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
	retval = io_poll(io);
	quit_log_if(retval,"io_poll() error");
	retval = io_dev_ready(io,imu_fds,&read,&write);
	quit_log_if(retval,"io_dev_ready() error");
	if(read)
	{
	    retval = imu_comm_read(imu);
	    if(retval != ERROR_OK)
	    {
		imu_update = false;
		continue;
	    }

	    /// Get new unread data
	    imu_update = true;
	    retval = imu_comm_get_data_latest(imu,&imu_data);
	    log_n_continue(retval,"IMU did not have new data!");
	    if(first_run)
	    {
		first_run = false;
		continue;
	    }

	    /// Get new state estimation
	    retval = uquad_kalman(kalman, pp->sp->w, &imu_data);
	    log_n_continue(retval,"Kalman update failed");

	    /// Get current set point
	    retval = pp_update_setpoint(pp, kalman->x_hat);
	    log_n_continue(retval,"Kalman update failed");

	    /// Get control command
	    retval = control(ctrl, w, kalman->x_hat, pp->sp);
	    log_n_continue(retval,"Control failed!");

	    sleep_ms(100);

	    gettimeofday(&tv_tmp,NULL);
	    uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_m_cmd);	    
	    if (tv_diff.tv_usec > 2*MOT_COMMAND_T_US)
	    {
		gettimeofday(&tv_last_m_cmd,NULL);

		/// Update motor controller
		retval = mot_set_vel_rads(mot, w->m_full);
		log_n_continue(retval,"Failed to set motor speed!");
#if DEBUG && LOG_W
		fprintf(log_w, "%ld\t", tv_diff.tv_usec);
		retval = uquad_mat_transpose(wt,w);
		uquad_mat_dump(wt,log_w);
#endif
	    }
	}//if(read)
	// stdin
	if(reg_stdin){
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
    
