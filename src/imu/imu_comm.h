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
#define IMU_DEFAULT_FS 1 // this is an index
#define IMU_DEFAULT_ACC_SENS 0 // this is an index
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
#define IMU_FS_OPT_COUNT 5
#define IMU_GRAVITY 9.81

/// ASCII 35, starts the unit running in binary mode with all channels active
#define IMU_COMMAND_RUN '#'
/// ASCII 32, stops the unit and returns it to the idle state.
#define IMU_COMMAND_IDLE ' '
/**
 * Issuing another ' ' displays cfg menu, which is bad, since then we would have to
 * read out all the crap.
 * AVOID two succesive ' '
 * 
 */

/// ASCII 37, sets the accelerometer sensitivity to 1.5g
#define IMU_COMMAND_ACC_1G '%'
/// ASCII 38, sets the accelerometer sensitivity to 2g
#define IMU_COMMAND_ACC_2G '&'
/// ASCII 39, sets the accelerometer sensitivity to 4g
#define IMU_COMMAND_ACC_4G '\''
/// ASCII 40, sets the accelerometer sensitivity to 6g
#define IMU_COMMAND_ACC_6G '('

/// ASCII 41, sets the sample frequency to 50Hz
#define IMU_COMMAND_FS_50 ')' 
/// ASCII 42, sets the sample frequency to 100Hz
#define IMU_COMMAND_FS_100 '*'
/// ASCII 43, sets the sample frequency to 150Hz
#define IMU_COMMAND_FS_150 '+'
/// ASCII 44, sets the sample frequency to 200Hz
#define IMU_COMMAND_FS_200 ','
/// ASCII 45, sets the sample frequency to 250Hz
#define IMU_COMMAND_FS_250 '-'

#define READ_RETRIES 16

#if (IMU_BUTES_PER_SENSOR > 2)
#define short int
#endif

struct imu_frame{
    unsigned short raw[IMU_SENSOR_COUNT];
    unsigned short count;
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
    int fs;
    // sampling period
    double T;
    // sens index
    int acc_sens;
    int frame_width_bytes;
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
    uquad_bool_t in_cfg_mode;
    FILE * device;
}imu;

struct imu * imu_comm_init(const char * device);

int imu_comm_deinit(struct imu * imu);

// -- -- -- -- -- -- -- -- -- -- -- --
// Reading from IMU
// -- -- -- -- -- -- -- -- -- -- -- --

uquad_bool_t imu_comm_avg_ready(struct imu * imu);

int imu_comm_get_avg(struct imu * imu, imu_data_t * data);

int imu_comm_get_data(struct imu * imu, imu_data_t * data);

int imu_comm_poll(struct imu * imu, uquad_bool_t * ready);

int imu_comm_read_frame(struct imu * imu);

int imu_comm_calibrate(struct imu * imu);

int imu_comm_print_data(imu_data_t * data, FILE * stream);

// -- -- -- -- -- -- -- -- -- -- -- --
// Configuring IMU
// -- -- -- -- -- -- -- -- -- -- -- --

int imu_comm_set_acc_sens(struct imu * imu, int new_value);

int imu_comm_set_fs(struct imu * imu, int new_value);

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
