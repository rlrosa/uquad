#ifndef IMU_COMM_H
#define IMU_COMM_H

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/select.h>
#include <../common/error_codes.h>
#include <../common/uquad_types.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define IMU_FRAME_INIT_CHAR 'A'
#define IMU_FRAME_END_CHAR 'Z'
#define IMU_INIT_END_SIZE 1
#define IMU_DEFAULT_FRAME_SIZE_BYTES 16 // 4 bytes init/end chars, 2 bytes per sensor reading
#define IMU_FRAME_SAMPLE_AVG_COUNT 16 // Reduce variance my taking avg
#define IMU_DEFAULT_FS 10
#define IMU_DEFAULT_T 0.1
#define IMU_DEFAULT_ACC_SENS 0
#define IMU_ADC_BITS 10
#define IMU_ADC_VREF 3.3
#define IMU_ADC_COUNTS_2_VOLTS 0.00322265625 // IMU_ADC_VREF/(1<<IMU_ADC_BITS)

#define IMU_SAMPLE_COUNT_SIZE 2
#define IMU_BYTES_PER_SENSOR 2
#define IMU_BYTES_COUNT 2
#define IMU_SENSOR_COUNT 6
#define IMU_ACCS 3
#define IMU_GYROS 3
#define IMU_SENS_OPT_COUNT 4
#define IMU_GRAVITY 9.81

#define READ_RETRIES 16

#if (IMU_BUTES_PER_SENSOR > 2)
#define short int
#endif

struct imu_frame{
    unsigned short raw[IMU_SENSOR_COUNT];
    unsigned short int count;
    struct timeval timestamp;
};

/// Based on ADC counts, no units
struct imu_data{
    double xyzrpy[IMU_SENSOR_COUNT];
    struct timeval timestamp;
};

typedef struct imu_data imu_data_t;
typedef struct imu_data imu_null_estimates_t;

struct imu_settings{
    // sampling frequency
    unsigned int fs;
    // sampling period
    double T;
    // sens index
    unsigned int acc_sens;
    // 
    unsigned int frame_width_bytes;
};

struct imu{
    struct imu_settings settings;
    imu_null_estimates_t null_estimates;
    struct imu_frame frame_buffer[IMU_FRAME_SAMPLE_AVG_COUNT];
    struct timeval frame_avg_init,frame_avg_end;
    int frames_sampled;
    int unread_data;
    uquad_bool_t avg_ready;
    imu_data_t avg;
    FILE * device;
}imu;

struct imu * imu_comm_init(void);

int imu_comm_connect(struct imu * imu, char * device);

int imu_comm_disconnect(struct imu * imu);

int imu_comm_deinit(struct imu * imu);

uquad_bool_t imu_comm_avg_ready(struct imu * imu);

int imu_comm_get_avg(struct imu * imu, imu_data_t * data);

int imu_comm_get_data(struct imu * imu, imu_data_t * data);

int imu_comm_poll(struct imu * imu, uquad_bool_t * ready);

int imu_comm_read_frame(struct imu * imu);

int imu_comm_calibrate(struct imu * imu);

int imu_comm_print_data(imu_data_t * data, FILE * stream);

#endif // IMU_COMM_H

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Notes and unused code
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#if 0

//Example timestamp usage
struct timeval detail_time;
gettimeofday(&detail_time,NULL);
printf("%d %d",
detail_time.tv_usec /1000,  /* milliseconds */
detail_time.tv_usec); /* microseconds */

/*
About IMU code
- IMU runs a loop that does:
  - set timer for 1/fs
  - for each channel
    - Read ADC chan
    - send data
  - check if commands were issued via uart, if so, handle them
  - wait for timer to finish

That is the "sampling frequency" parameter.
Each ADC conversion takes 13 clock cycles, except the first which takes 25, to warmup analog circuitry.
IMU fw defines clock as 10MHz, though default is 1MHz, and nothing seems to change the default. Also, the 1us sleep in main.c makes more sense if the clock were 1Mhz. Not sure about this yet.
Waiting (1/50kHz)*6 = 120us should be enough
*/

#endif
