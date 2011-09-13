#ifndef IMU_COMM_H
#define IMU_COMM_H

#include <sys/socket.h>
#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define ERROR_OK 0
#define ERROR_FAIL -1
#define ERROR_READ_TIMEOUT -2
#define ERROR_READ_SYNC -3
#define ERROR_OPEN -4
#define ERROR_CLOSE -5
#define ERROR_MALLOC -6

#define IMU_FRAME_INIT_CHAR 'A'
#define IMU_FRAME_END_CHAR 'Z'
#define IMU_INIT_END_SIZE 1
#define IMU_DEFAULT_FRAME_SIZE_BYTES 16 // 4 bytes init/end chars, 2 bytes per sensor reading
#define IMU_FRAME_SAMPLE_AVG_COUNT 16
#define IMU_DEFAULT_FS 10
#define IMU_DEFAULT_T 0.1
#define IMU_DEFAULT_GYRO_SENS 0

#define IMU_SAMPLE_COUNT_SIZE 2
#define IMU_BYTES_PER_SENSOR 2
#define IMU_BYTES_COUNT 2
#define IMU_SENSOR_COUNT 6
#define IMU_ACCS 3
#define IMU_GYROS 3
#define IMU_SENS_OPT_COUNT 4
#define IMU_GRAVITY 9.81

#define READ_RETRIES 16

#define err_propagate(retval) if(retval!=ERROR_OK)return retval;
#define err_check(retval,msg) if(retval!=ERROR_OK){fprintf(stderr,msg);return retval;}

// Example timestamp usage
//struct timeval detail_time;
//gettimeofday(&detail_time,NULL);
//printf("%d %d",
//detail_time.tv_usec /1000,  /* milliseconds */
//detail_time.tv_usec); /* microseconds */

struct imu_frame{
    unsigned short raw[IMU_DEFAULT_FRAME_SIZE_BYTES];
    unsigned short int count;
    struct timeval timestamp;
};

/// Based on ADC counts, no units
struct imu_null_estimates{
    double xyzrpy[IMU_SENSOR_COUNT];
    struct timeval timestamp;
};

struct imu_settings{
    // sampling frequency
    unsigned int fs;
    // sampling period
    double T;
    // sens index
    unsigned int gyro_sens;
    // 
    unsigned int frame_width_bytes;
};
    
struct imu{
    struct imu_settings settings;
    struct imu_null_estimates null_estimates;
    struct imu_frame frame_buffer[IMU_FRAME_SAMPLE_AVG_COUNT];
    struct timeval frame_avg_init,frame_avg_end;
    int frames_sampled;
    int unread_data;
    FILE * device;
}imu;

struct imu * imu_comm_init(void);

int imu_comm_connect(struct imu * imu, char * device);

int imu_comm_disconnect(struct imu * imu);

int imu_comm_deinit(struct imu * imu);

int imu_comm_get_data(struct imu * imu, double * xyzrpy);

#endif // IMU_COMM_H
