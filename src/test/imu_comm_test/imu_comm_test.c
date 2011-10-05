#include <imu_comm.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h> // For STDIN_FILENO
#include <stdio.h>

#define IMU_COMM_TEST_EXIT_CHAR 'q'
#define IMU_COMM_TEST_HOW_TO "\nIncorrect arguments!\nUsage: ./imu_comm_test /dev/tty#\n"
#define WAIT_COUNTER_MAX 10
#define IMU_COMM_TEST_EOL_LIM 128

static int fix_end_of_time_string(char * string, int lim){
    int i;
    if(lim<0)
	return -1;
    for(i=0;i<lim;++i){
	if(string[i]!='\n')
	    continue;
	break;
    }
    if(i==lim){
	fprintf(stderr,"Failed to fix time string");
	return ERROR_FAIL;
    }else{
	string[i]='\0';
    }	
    return ERROR_OK;
}

int main(int argc, char *argv[]){
    int retval;
    FILE * output_frames = NULL;
    FILE * output_avg = NULL;
    unsigned char tmp;
    char * device;
    char log_name[FILENAME_MAX];
    char * time_string;
    struct imu * imu = NULL;
    fd_set rfds;
    struct timeval tv;

    if(argc<2){
	fprintf(stderr,IMU_COMM_TEST_HOW_TO);
	exit(1);
    }else{
	device = argv[1];
    }
    if(argc<3){
	fprintf(stdout,"Using stdout to output...\n");
    }else{
	time_t rawtime;
	int i;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime (&rawtime);

	time_string = asctime (timeinfo);
	retval = fix_end_of_time_string(time_string,IMU_COMM_TEST_EOL_LIM);
	if(retval != ERROR_OK)
	    exit(1);
	// Setup frame log
	sprintf(log_name,"./logs/%s.log",time_string);
	output_frames = fopen(log_name,"w");
	if(output_frames == NULL){
	    fprintf(stderr,"Failed to create frame log file...");
	    exit(1);
	}
	fprintf(stdout,"Sending frame output to log file: %s\n",log_name);

	// Setup avg log
	sprintf(log_name,"./logs/%savg.log",time_string);
	if(retval != ERROR_OK)
	    exit(1);
	output_avg = fopen(log_name,"w");
	if(output_avg == NULL){
	    fprintf(stderr,"Failed to create avg log file...");
	    exit(1);
	}
	fprintf(stdout,"Sending avg output to log file: %s\n",log_name);

    }

    // Initialize structure
    imu = imu_comm_init(device);
    if(imu==NULL){
	fprintf(stderr,"Fatal error.");
	exit(1);
    }

    // do stuff...
    imu_data_t data;
    uquad_bool_t do_sleep = false;
    int user_input_detected;
    uquad_bool_t data_ready = false;
    int wait_counter = WAIT_COUNTER_MAX;
    FD_ZERO(&rfds);
    printf("Press 'q' to abort..\n");
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
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	user_input_detected = select(STDIN_FILENO+1,&rfds,NULL,NULL,&tv);
	if (user_input_detected < 0){
	    err_check(ERROR_IO,"select() failed!");
	}else{
	    retval = imu_comm_poll(imu,&data_ready);
	    err_propagate(retval);
	    if (user_input_detected == 0){
	    // No user input, so keep running the loop.
		if(data_ready){
		    // IMU will do ADC conv, send, then next sensor, send, etc
		    // We nead to wait for the ADC and for the comm delays.
		    // See imu_comm.h for more info.
		    // We should wait for this, then read out all the info.
		    //		    usleep(120*50);//TODO understand and tune according!!
		    retval = imu_comm_read_frame(imu);
		    if(retval == ERROR_READ_TIMEOUT){
			do_sleep = true;
			printf("Not enough data available...\n");
			continue;// skip the rest of the loop
		    }
		    if(retval != ERROR_OK)
			continue;
		    do_sleep = false;
		    if(output_frames != NULL){
			// Printing to stdout is unreadable
			retval = imu_comm_get_data_raw_latest_unread(imu,&data);
			if(retval == ERROR_OK){
			    retval = imu_comm_print_data(&data,output_frames);
			    err_propagate(retval);
			}
		    }

		    // Get avg
		    if(imu_comm_avg_ready(imu)){
			retval = imu_comm_get_avg(imu,&data);
			err_propagate(retval);
			retval = imu_comm_print_data(&data,output_avg);
			err_propagate(retval);
		    }
		    if(output_frames == NULL || output_avg == NULL)
			fflush(stdout);
		}
	    }else{ // retval > 0
		// Handle user input
	    	printf("\nKey detected, getting avg...\n");
		// Clear user input
		retval = fread(&tmp,1,1,stdin);
		if(tmp == IMU_COMM_TEST_EXIT_CHAR)
		    break;
	    }
	}
    }

    // Deinit structure & close connection
    retval = imu_comm_deinit(imu);
    err_propagate(retval);
    // Close log files, if any
    if(output_avg != NULL)
	fclose(output_avg);
    if(output_frames != NULL)
	fclose(output_frames);
    printf("Exit successful!\n");

    return 0;
}
    
