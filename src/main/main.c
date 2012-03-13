#include <uquad_error_codes.h>
#include <uquad_types.h>
#include <macros_misc.h>
#include <uquad_aux_io.h>
#include <imu_comm.h>
#include <mot_control.h>
#include <uquad_kalman.h>
//#include <uquad_gps_comm.h> //TODO add!
#include <sys/signal.h> // for SIGINT and SIGQUIT
#include <unistd.h> // for STDIN_FILENO

#define UQUAD_HOW_TO "./main <imu_device>"

/// Global structs
static imu_t *imu;
static kalman_io_t *kalman;
static uquad_mot_t *mot;
static io_t *io;
//static gps_t *gps; //TODO add
/// Global var
uquad_mat_t *w;
uquad_mat_t *x;
imu_data_t imu_data;

/** 
 * Clean up and close
 * 
 */
void quit()
{
    int retval;
    kalman_deinit(kalman);
    retval = io_deinit(io);
    if(retval != ERROR_OK)
    {
	err_log("Could not close IO correctly!");
    }
    retval = imu_comm_deinit(imu);
    if(retval != ERROR_OK)
    {
	err_log("Could not close IMU correctly!");
    }
    retval = mot_deinit(mot);
    if(retval != ERROR_OK)
    {
	err_log("Could not close motor driver correctly!");
    }
    //TODO deinit everything?
    exit(retval);
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

    /// Global vars
    w = uquad_mat_alloc(4,1);        // Current angular speed [rad/s]
    for(i=0; i < 4; ++i)
	w->m_full[i] = MOT_W_IDLE;
    x = uquad_mat_alloc(1,12);   // State vector

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
    uquad_bool_t reg_stdin = true;
    unsigned char tmp_buff[2];
    int counter = 0;
    poll_n_read:
    while(1){
	imu_update = false;
	retval = io_poll(io);
	quit_log_if(retval,"io_poll() error");
	retval = io_dev_ready(io,imu_fds,&read,&write);
	quit_log_if(retval,"io_dev_ready() error");
	if(read)
	{
	    retval = imu_comm_read(imu);
	    if(retval != ERROR_OK) continue;

	    /// Get new unread data
	    imu_update = true;
	    retval = imu_comm_get_data_latest_unread(imu,&imu_data);
	    log_n_continue(retval,"IMU did not have new data!");

	    /// Get new state estimation
	    retval = uquad_kalman(kalman, w, &imu_data);
	    log_n_continue(retval,"Kalman update failed");

	    /// Get current set point
#warning "//TODO! path_planner.h not implemented!"

	    /// Get control command
	    //retval = uquad_control(kalman->x_hat);
	    //TODO control.h!
#warning "//TODO! control.h not implemented!"

	    /// Update motor controller
	    retval = mot_set_vel_rads(mot, w->m_full);
	    log_n_continue(retval,"Failed to set motor speed!");

	}
	// stdin
	if(reg_stdin){
	    retval = io_dev_ready(io,STDIN_FILENO,&read,&write);
	    quit_log_if(retval,"io_dev_ready() error");
	    if(read){
		retval = fread(tmp_buff,1,1,stdin);
		if(retval<=0)
		    fprintf(stdout,"\nNo user input!!\n\n");
		else
		    // Each time user presses RET a device will be removed from
		    // the registered dev list.
		    // TODO do something!
		    continue;
	    }
	}
	fflush(stdout);
    }

    return 0;
}
    
