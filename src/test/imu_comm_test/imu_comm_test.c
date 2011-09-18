#include <imu_comm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h> // For STDIN_FILENO
#include <stdio.h>

#define IMU_COMM_TEST_HOW_TO "\nIncorrect arguments!\nUsage: ./imu_comm_test /dev/tty#\n"
#define WAIT_COUNTER_MAX 10

int main(int argc, char *argv[]){
    int retval;
    char * device;
    struct imu * imu = NULL;
    fd_set rfds;
    struct timeval tv;

    if(argc<2){
	fprintf(stderr,IMU_COMM_TEST_HOW_TO);
	exit(1);
    }else{
	device = argv[1];
    }
    
    // Initialize structure
    imu = imu_comm_init();
    if(imu==NULL){
	fprintf(stderr,"Fatal error.");
	exit(1);
    }

    // Init connection
    retval = imu_comm_connect(imu,device);
    err_propagate(retval);

    // do stuff...
    imu_data_t data;
    uquad_bool_t do_sleep = false;
    int user_input_detected;
    uquad_bool_t data_ready = false;
    int wait_counter = WAIT_COUNTER_MAX;
    FD_ZERO(&rfds);
    printf("Press any key to abort..\n");
    while(1){
	if(do_sleep){
	    printf("Waiting...\n");
	    usleep(1000*500); // Wait for a while... (0.5 sec)
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
		    usleep(120);
		    retval = imu_comm_read_frame(imu);
		    if(retval == ERROR_READ_TIMEOUT){
			do_sleep = true;
			printf("Not enough data available...\n");
			continue;// skip the rest of the loop
		    }else{
			err_propagate(retval);
		    }
		    retval = imu_comm_print_frame(imu->frame_buffer + imu->frames_sampled,NULL);
		    err_propagate(retval);
		}
	    }else{ // retval > 0
		// Handle user input
	    	printf("\nKey detected, getting avg...\n");
		if(imu_comm_avg_ready(imu)){
		    fprintf(stdout,"\nAvg ready:\n");
		    retval = imu_comm_get_avg(imu,&data);
		}else{
		    fprintf(stdout,"\nAvg NOT ready:\n");
		    retval = imu_comm_get_avg(imu,&data);
		}
		err_propagate(retval);
		retval = imu_comm_print_data(&data,NULL);
		break;
	    }
	}
    }

    // Close connection
    retval = imu_comm_disconnect(imu);
    err_propagate(retval);
    printf("Connection to IMU closed\n");

    // Deinit structure
    retval = imu_comm_deinit(imu);
    err_propagate(retval);
    printf("Exit successful!\n");

    return 0;
}
    
