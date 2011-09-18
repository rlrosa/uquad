#include <../imu/imu_comm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h> // For STDIN_FILENO
#include <stdio.h>


#define quit_if_not_ok(retval) if(retval!=ERROR_OK)exit(1);
#define IMU_COMM_TEST_HOW_TO "\nIncorrect arguments!\nUsage: ./imu_comm_test /dev/tty#\n"

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
    quit_if_not_ok(retval);

    // do stuff...
    imu_data_t data;
    uquad_bool_t data_ready = false;
    int i;
    FD_ZERO(&rfds);
    printf("Press any key to abort..\n");
    while(1){
	// Run loop until user presses any key or velociraptors chew through the power cable
	FD_SET(STDIN_FILENO,&rfds);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	retval = select(STDIN_FILENO+1,&rfds,NULL,NULL,&tv);
	if (retval != 0){
	    if(retval<0){
		err_check(ERROR_IO,"select() failed!");
	    }else{//retval>0
		// FD_ISSET(STDIN_FILENO, &rfds) will be true.
		printf("\nKey detected, aborting...\n");
		break;
	    }
	}else{
	    // No user input, so keep running the loop.
	    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	    // begin:Useful part of the test
	    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	    retval = imu_comm_poll(imu,&data_ready);
	    if(data_ready){
		// IMU will do ADC conv, send, then next sensor, send, etc
		// We nead to wait for the ADC and for the comm delays.
		// See imu_comm.h for more info.
		// We should wait for this, then read out all the info.
		usleep(120);
		retval = imu_comm_get_data(imu,&data);
		quit_if_not_ok(retval);
		fprintf(stdout,"sec|usec:%d|%d\n",(int)data.timestamp.tv_sec,(int)data.timestamp.tv_usec);
		for(i=0;i<IMU_SENSOR_COUNT;++i){
		    fprintf(stdout,"%f\t",data.xyzrpy[i]);
		}
		fprintf(stdout,"\n");

	    }
	    // end:Useful part of the test
	    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	}
    }

    // Close connection
    retval = imu_comm_disconnect(imu);
    quit_if_not_ok(retval);
    printf("Connection to IMU closed\n");

    // Deinit structure
    retval = imu_comm_deinit(imu);
    quit_if_not_ok(retval);
    printf("Exit successful!\n");

    return 0;
}
    
