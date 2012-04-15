#include <imu_comm.h>
#include <uquad_logger.h>
#include <macros_misc.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/signal.h> // for SIGINT and SIGQUIT
#include <unistd.h>     // For STDIN_FILENO
#include <stdio.h>

#define IMU_COMM_TEST_EXIT_CHAR 'q'
#define IMU_COMM_TEST_CALIB_CHAR 'c'
#define IMU_COMM_TEST_CALIB_SHOW_CHAR 's'
#define IMU_COMM_TEST_HOW_TO "\nIncorrect arguments!\n"\
    "Usage: ./imu_comm_test /dev/tty# log_to_file\n\n"\
    "If last argument is supplied (it can be any string) "\
    "then the program will log raw data and\n processed"\
    " data to two files, named according to the current timestamp."
#define WAIT_COUNTER_MAX 10
#define IMU_COMM_TEST_EOL_LIM 128

#define PRINT_RAW  1
#define PRINT_DATA 0
#define PRINT_AVG  0

#define TIMING_DEBUG   1
#define TIMING_ERR_MAX 0

#define PRINT_LOOP          0         // Stop reading from IMU, and loop in printing
#define PRINT_LOOP_DELAY_US (1000*10) // Match IMU sampling freq

static imu_t *imu = NULL;
#if PRINT_RAW
static FILE *log_imu_raw = NULL;
#endif // PRINT_RAW
#if PRINT_DATA
static FILE *log_imu_data = NULL;
#endif // PRINT_DATA
#if PRINT_AVG
static FILE *log_imu_avg = NULL;
#endif // PRINT_AVG

static struct timeval tv_max;

void quit()
{
    // Deinit structure & close connection
    (void) imu_comm_deinit(imu);
    // Close log files, if any
#if PRINT_RAW
    uquad_logger_remove(log_imu_raw);
#endif // PRINT_RAW
#if PRINT_DATA
    uquad_logger_remove(log_imu_data);
#endif // PRINT_DATA
#if PRINT_AVG
    uquad_logger_remove(log_imu_avg);
#endif
    err_log_tv("Max delay:",tv_max);
    printf("Exit successful!\n");
    exit(0);
}

void uquad_sig_handler(int signal_num){
    err_log_num("Caught signal:",signal_num);
    quit();
}

int main(int argc, char *argv[]){
    int retval;
    int err_count = 0;
    unsigned char tmp[2];
    char * device;
    fd_set rfds;
    struct timeval tv, tv_start, tv_diff, tv_tmp, tv_tmp2, tv_old;
    gettimeofday(&tv_start,NULL);
    memset(&tv_max,0,sizeof(struct timeval));

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    if(argc < 2)
    {
	fprintf(stderr,IMU_COMM_TEST_HOW_TO);
	quit();
    }
    else
    {
	device = argv[1];
    }

#if PRINT_RAW
    // Setup raw frame log
    log_imu_raw = uquad_logger_add("imu_raw");
    if(log_imu_raw == NULL)
    {
	err_log("Failed to create frame log file...");
	quit();
    }
#endif

#if PRINT_DATA
    // Setup data log
    log_imu_data = uquad_logger_add("imu_data");
    if(log_imu_data == NULL)
    {
	err_log("Failed to create data log file...");
	quit();
    }
#endif

#if PRINT_AVG
    // Setup avg log
    log_imu_avg = uquad_logger_add("imu_avg");
    if(log_imu_avg == NULL)
    {
	err_log("Failed to create avg log file...");
	quit();
    }
#endif

    // Initialize structure
    imu = imu_comm_init(device);
    if(imu==NULL){
	fprintf(stderr,"Fatal error.");
	exit(1);
    }

    // do stuff...
    imu_data_t data;
#if PRINT_RAW
    imu_raw_t raw;
#endif
    imu_calib_t *imu_calib;
    uquad_bool_t do_sleep = false, calibrating = false;
    uquad_bool_t data_ready = false;
    int read_will_not_lock;
    int wait_counter = WAIT_COUNTER_MAX;
    int imu_fd;
    unsigned long sample_count = 0;
    data.acc = uquad_mat_alloc(3,1);
    data.gyro = uquad_mat_alloc(3,1);
    data.magn = uquad_mat_alloc(3,1);
    retval = imu_comm_get_fds(imu, &imu_fd);
    FD_ZERO(&rfds);
    printf("Options:\n'q' to abort,\n'c' to calibrate\n's' to display current calibration\n\n");
    gettimeofday(&tv_old,NULL);
    while(1){
	if(do_sleep){
	    printf("Waiting...\n");
	    //	    usleep(1000*500); // Wait for a while... (0.5 sec)
	    printf("Running again.\n");
	    if(--wait_counter<0){
		// waited enough, now die
		printf("\nGave up on waiting, terminating program...\n");
		return -1;
	    }
	}else{
	    wait_counter = WAIT_COUNTER_MAX;
	}
	// Run loop until user presses any key or velociraptors chew through the power cable
	FD_SET(STDIN_FILENO,&rfds);
	FD_SET(imu_fd,&rfds);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	read_will_not_lock = select((STDIN_FILENO>imu_fd?STDIN_FILENO:imu_fd)+1,&rfds,NULL,NULL,&tv);
	if (read_will_not_lock < 0){
	    err_check(ERROR_IO,"select() failed!");
	}else{
	    // Read from IMU
	    if(FD_ISSET(imu_fd,&rfds)){
		do_sleep = false;
		retval = imu_comm_read(imu, &data_ready);
		if(retval == ERROR_READ_TIMEOUT){
		    printf("Not enough data available...\n");
		    do_sleep = true;
		    continue;// skip the rest of the loop
		}
		if(retval != ERROR_OK){
		    err_log_num("Error detected!:",retval);
		    continue;
		}

		if(!data_ready)
		    continue;
		sample_count++;

#if TIMING_DEBUG
		gettimeofday(&tv_tmp,NULL);
		uquad_timeval_substract(&tv_diff, tv_tmp, tv_old);
		retval = uquad_timeval_substract(&tv_tmp2, tv_diff, tv_max);
		if(retval > 0)
		{
		    tv_max = tv_diff;
		    err_log_tv("Current max delay:",tv_max);
		}
		if(tv_diff.tv_sec > 0)
		{
		    if(++err_count > TIMING_ERR_MAX)
		    {
			err_log_num("Out of range!:",err_count);
			quit();
		    }
		}
		retval = imu_comm_get_raw_latest_unread(imu,&raw);
		quit_if(retval);
		retval = imu_comm_print_raw(&raw,log_imu_raw);
		quit_if(retval);
		tv_old = tv_tmp;
		continue;
#endif


		if(imu_comm_get_status(imu) == IMU_COMM_STATE_CALIBRATING)
		    // if calibrating, then data should not be used.
		    continue;
		else if(!imu_comm_calib_estim(imu))
		{
		    // if no calibration estim exists, build one.
		    retval = imu_comm_calibration_start(imu);
		    quit_if(retval);
		    calibrating = true;
		    continue;
		}

		if(calibrating)
		{
		    printf("Calibration completed!\nPress enter to continue...");
		    gettimeofday(&tv_old,NULL);
		    calibrating = false;
		}
#if PRINT_RAW
		retval = imu_comm_get_raw_latest_unread(imu,&raw);
		if(retval == ERROR_OK)
		{
		    gettimeofday(&tv_tmp,NULL);
		    uquad_timeval_substract(&tv_diff, tv_tmp, tv_start);
		    log_tv_only(log_imu_raw, tv_diff);
		    log_eol(log_imu_raw);
		    //		    retval = imu_comm_print_raw(&raw,log_imu_raw);
		    //		    quit_if(retval);
		}
#endif
#if PRINT_DATA
#if PRINT_RAW
		// undread data was already taken by get_raw_latest_unread()
		retval = imu_comm_get_data_latest(imu,&data);
#else
		retval = imu_comm_get_data_latest_unread(imu,&data);
#endif // PRINT_RAW
		if(retval == ERROR_OK)
		{
		    gettimeofday(&tv_tmp,NULL);
		    uquad_timeval_substract(&tv_diff, tv_tmp, tv_start);
		    log_tv_only(log_imu_data, tv_diff);
		    retval = imu_comm_print_data(&data,log_imu_data);
		    quit_if(retval);
		}
#endif // PRINT_DATA

#if PRINT_AVG
		if(imu_comm_avg_ready(imu)){
		    retval = imu_comm_get_avg(imu,&data);
		    quit_if(retval);
		    gettimeofday(&tv_tmp,NULL);
		    uquad_timeval_substract(&tv_diff, tv_tmp, tv_start);
		    log_tv_only(log_imu_avg, tv_diff);
		    retval = imu_comm_print_data(&data,log_imu_avg);
		    quit_if(retval);
		}
#endif // PRINT_AVG
	    }

	    // Handle user input
	    if(FD_ISSET(STDIN_FILENO,&rfds)){
		printf("\nGetting user input...\n");
		// Get & clear user input
		retval = fread(tmp,1,2,stdin);
		printf("Read:%c\n",tmp[0]);
		
		// exit
		if(tmp[0] == IMU_COMM_TEST_EXIT_CHAR)
		    break;
		// do calibration
		if(tmp[0] == IMU_COMM_TEST_CALIB_CHAR){
		    sleep_ms(500);
		    printf("Starting calibration...\n");
		    retval = imu_comm_calibration_start(imu);
		    quit_if(retval);
		    calibrating = true;
		}

		// display current calibration
		if(tmp[0] == IMU_COMM_TEST_CALIB_SHOW_CHAR){
		    if(!imu_comm_calib_estim(imu))
		    {
			printf("Calibration not ready.\nPress enter to continue...");
			wait_for_enter;
		    }
		    else
		    {
			retval = imu_comm_calibration_get(imu,&imu_calib);
			quit_if(retval);
			printf("Current calibration:\n");
			retval = imu_comm_print_calib(imu_calib,stdout);
			quit_if(retval);
		    }
		}
		printf("\nPress enter to continue...\n");
		fflush(stdout);
		wait_for_enter;
	    }
	}
    }
	
    quit();
    return 0;
}
    
