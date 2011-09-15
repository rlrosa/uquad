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
    imu = imu_comm_init();
    if(imu==NULL){
	fprintf(stderr,"Fatal error.");
	exit(1);
    }

    // Init connection
    retval = imu_comm_connect(imu,device);
    quit_if_not_ok(retval);

    // do stuff...
    double xyzrpy[IMU_SENSOR_COUNT];
    retval = imu_comm_get_data(imu,xyzrpy);
    quit_if_not_ok(retval);

    // Close connection
    retval = imu_comm_disconnect(imu);
    quit_if_not_ok(retval);

    // Deinit structure
    retval = imu_comm_deinit(imu);
    quit_if_not_ok(retval);

    return 0;
}
    
