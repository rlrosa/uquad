#include "imu_comm.h"

#define quit_if_not_ok(retval) if(retval!=ERROR_OK)exit(1);
#define IMU_COMM_TEST_HOW_TO "\nIncorrect arguments!\nUsage: ./imu_comm_test /dev/tty#\n"

int main(int argc, char *argv[]){
    int retval;
    char * device;
    struct imu * imu = NULL;
    if(argc<2){
	fprintf(stderr,IMU_COMM_TEST_HOW_TO);
	exit(1);
    }else{
	device = argv[1];
    }
    
    // Initialize structure
    retval = imu_init(imu);
    quit_if_not_ok(retval);

    // Init connection
    retval = imu_comm_connect(imu,device);
    quit_if_not_ok(retval);

    // do stuff...

    // Close connection
    retval = imu_comm_disconnect(imu);
    quit_if_not_ok(retval);

    // Deinit structure
    retval = imu_deinit(imu);
    quit_if_not_ok(retval);

    return 0;
}
    
