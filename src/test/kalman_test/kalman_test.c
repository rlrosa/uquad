#include <imu_comm.h>
#include <uquad_error_codes.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h> // For STDIN_FILENO
#include <stdio.h>
#include <uquad_gps_comm.h>
#include "uquad_kalman.h"

#define EXIT_CHAR 'q'
#define CMD_CHAR_1 'c'
#define SHOW_CHAR 's'

#define KALMAN_TEST_HOW_TO "\nIncorrect arguments!\nUsage: ./imu_comm_test /dev/tty# log_to_file\n\nIf last argument is supplied (it can be any string) then the program will log raw data and\n processed data to two files, named according to the current timestamp."
#define WAIT_COUNTER_MAX 10
#define IMU_COMM_TEST_EOL_LIM 128

#define wait_for_enter printf("ERROR!\n") //while(fread(tmp,1,1,stdin) == 0)

static int generate_log_name(char * log_name, char * start_string){
    time_t rawtime;
    struct tm * tm;
    int retval;
    time(&rawtime);
    tm = localtime (&rawtime);
    retval = sprintf(log_name,"%04d_%02d_%02d_xx_%02d_%02d_%02d", 1900 + tm->tm_year, tm->tm_mon + 1,tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
    if(retval < 0)
	return ERROR_FAIL;
    if(start_string != NULL)
	retval = sprintf(log_name,"%s%s",start_string,log_name);
    return retval;
}

int main(int argc, char *argv[]){
    int retval;
    FILE * output_frames = NULL;
    unsigned char tmp[2];
    char * device;
    char log_name[FILENAME_MAX];
    char log_filename[FILENAME_MAX];
    struct imu * imu = NULL;
    fd_set rfds;
    struct timeval tv;

    if(argc<2){
	err_log(KALMAN_TEST_HOW_TO);
	exit(1);
    }else{
	device = argv[1];
    }
    if(argc<3){
	fprintf(stdout,"Using stdout to output...\n");
    }else{
	retval = generate_log_name(log_name,argv[2]);
	if(retval < 0){
	    err_log("Failed to create log name...");
	    exit(1);
	}
   
	// Setup frame log
	sprintf(log_filename,"./logs/%s.log",log_name);
	output_frames = fopen(log_filename,"w");
	if(output_frames == NULL){
	    err_log("Failed to create frame log file...");
	    exit(1);
	}
	fprintf(stdout,"Sending frame output to log file: %s\n",log_name);
    }

    // Initialize structure
    imu = imu_comm_init(device);
    if(imu==NULL){
	err_log("Fatal error.");
	exit(1);
    }

    // do stuff...
    uquad_mat_t* tmp4print = uquad_mat_alloc(1,12);
    imu_data_t data;
    //    uquad_bool_t do_sleep = false;
    int read_will_not_lock;
    int imu_fd;
    data.acc = uquad_mat_alloc(3,1);
    data.gyro = uquad_mat_alloc(3,1);
    data.magn = uquad_mat_alloc(3,1);
    retval = imu_comm_get_fds(imu, &imu_fd);
    FD_ZERO(&rfds);
    printf("Options:\n'q' to abort,\n'c' to calibrate\n's' to display current calibration\n\n");


    kalman_io_t* kalman_io_data = kalman_init();
    uquad_mat_t* w = uquad_mat_alloc(4,1);
    uquad_bool_t first_run = true,
	imu_ready = false;
    struct timeval tv_last_kalman, tv_diff, tv_tmp;

    uquad_bool_t gps_update;
    int gps_counter = 0;
    gps_comm_data_t* gps_dat;
    gps_dat = gps_comm_data_alloc();
    retval = uquad_mat_zeros(gps_dat -> pos);
    err_propagate(retval);
    retval = uquad_mat_zeros(gps_dat -> vel);
    err_propagate(retval);

    while(1){
	gps_counter++;
	gps_update = (gps_counter % 100 == 0);
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
		//		do_sleep = false;
		retval = imu_comm_read(imu, &imu_ready);
		if(!imu_ready){
		    //		    printf("Not enough data available...\n");
		    //		    do_sleep = true;
		    continue;// skip the rest of the loop
		}
		if(retval != ERROR_OK){
		    wait_for_enter;
		    continue;
		}

		if(imu_comm_get_status(imu) == IMU_COMM_STATE_RUNNING)
		{
		    // Printing to stdout is unreadable

		    retval = imu_comm_get_data_latest(imu,&data);
		    err_propagate(retval);
		    if(retval == ERROR_OK)
		    {
			w -> m_full[0] = 334.28;
			w -> m_full[1] = 334.28;
			w -> m_full[2] = 334.28;
			w -> m_full[3] = 334.28;
			if(first_run)
			{
			    gettimeofday(&tv_last_kalman,NULL);
			    first_run=false;
			    continue;
			}
			gettimeofday(&tv_tmp,NULL);
			retval = uquad_timeval_substract(&tv_diff, tv_tmp, tv_last_kalman);
			if(retval < 0)
			{
			    // negative time, absurd
			    err_check(ERROR_FAIL, "Negative time is not valid!");
			}
			//			retval = uquad_kalman(kalman_io_data, w, &data, tv_diff.tv_usec);
			retval = uquad_kalman(kalman_io_data, 
					      w, &data, 13000.0);
			err_propagate(retval);

			if(gps_update)
			{
			    retval = uquad_kalman_gps(kalman_io_data, 
						      gps_dat);
			    err_propagate(retval);
			}

			retval = uquad_mat_transpose(tmp4print,kalman_io_data->x_hat);
			err_propagate(retval);
			uquad_mat_dump(tmp4print,output_frames);
			//retval = imu_comm_print_data(&data,output_frames);
			//err_propagate(retval);

			if(output_frames == NULL)
			    fflush(stdout);
		    }
		}
	    }

	    // Handle user input
	    if(FD_ISSET(STDIN_FILENO,&rfds)){
		printf("\nGetting user input...\n");
		// Get & clear user input
		retval = fread(tmp,1,2,stdin);
		printf("Read:%c\n",tmp[0]);
		
		// exit
		if(tmp[0] == EXIT_CHAR)
		    break;
		printf("\nPress enter to continue...\n");
		fflush(stdout);
		wait_for_enter;
	    }
	}
    }
		
    // Deinit structure & close connection
    retval = imu_comm_deinit(imu);
    err_propagate(retval);
    // Close log files, if any
    if(output_frames != NULL)
	fclose(output_frames);
    printf("Exit successful!\n");

    return 0;
}
    
