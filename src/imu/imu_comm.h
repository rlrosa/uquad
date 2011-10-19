#ifndef IMU_COMM_H
#define IMU_COMM_H

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/select.h>
#include <uquad_error_codes.h>
#include <uquad_types.h>
#include <uquad_aux_time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define IMU_FRAME_INIT_CHAR 'A'
#define IMU_FRAME_END_CHAR 'Z'
#define IMU_INIT_END_SIZE 1
#define IMU_DEFAULT_FRAME_SIZE_BYTES 16 // 4 bytes init/end chars, 2 bytes per sensor reading
#define IMU_FRAME_SAMPLE_AVG_COUNT 8 // Reduce variance my taking avg
#define IMU_COMM_CALIBRATION_NULL_SIZE 256 // Tune!
#define IMU_DEFAULT_FS 5 // this is an index
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

#define IMU_COMM_AVG_MAX_INTERVAL 2*IMU_FRAME_SAMPLE_AVG_COUNT //Too much...?

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

/// ASCII 49, select option #1 in some IMU menus
#define IMU_COMMAND_ONE '1'
/// ASCII 57, exit main IMU menu
#define IMU_COMMAND_EXIT '9'
/// ASCII 120, select option in some IMU menus
#define IMU_COMMAND_X 'x'

#define READ_RETRIES 16

#if (IMU_BUTES_PER_SENSOR > 2)
#define short int
#endif

struct imu_frame{
    unsigned short raw[IMU_SENSOR_COUNT];
    unsigned short count;
    struct timeval timestamp;
};
typedef struct imu_frame imu_frame_t;

/// Based on ADC counts, no units
struct imu_data{
    double xyzrpy[IMU_SENSOR_COUNT];
    struct timeval timestamp;
};

typedef struct imu_data imu_data_t;
typedef imu_data_t imu_null_estimates_t;
typedef imu_data_t imu_measurements_t;

struct imu_settings{
    // sampling frequency
    int fs;
    // sampling period
    double T;
    // sens index
    int acc_sens;
    int frame_width_bytes;
};

enum imu_status{IMU_COMM_STATE_RUNNING,IMU_COMM_STATE_IDLE,IMU_COMM_STATE_CALIBRATING,IMU_COMM_STATE_UNKNOWN};
typedef enum imu_status imu_status_t;

struct imu{
    // config & status
    struct imu_settings settings;
    imu_status_t status;
    uquad_bool_t in_cfg_mode;
    FILE * device;
    // calibration
    imu_null_estimates_t null_estimates;
    int calibration_counter;
    uquad_bool_t is_calibrated;
    // data
    imu_frame_t frame_buffer[IMU_FRAME_SAMPLE_AVG_COUNT];
    struct timeval frame_avg_init,frame_avg_end;
    int frames_sampled;
    int unread_data;
    int frame_next;
    uquad_bool_t avg_ready;
    imu_data_t avg;
};
typedef struct imu imu_t;

imu_t * imu_comm_init(const char * device);
int imu_comm_deinit(imu_t * imu);

imu_status_t imu_comm_get_status(imu_t * imu);

// -- -- -- -- -- -- -- -- -- -- -- --
// Reading from IMU
// -- -- -- -- -- -- -- -- -- -- -- --

int imu_comm_get_fds(imu_t * imu, int * fds);
int imu_comm_read(imu_t * imu, uquad_bool_t * success);

uquad_bool_t imu_comm_avg_ready(imu_t * imu);
int imu_comm_get_measurements_avg(imu_t * imu, imu_measurements_t * measurements);

int imu_comm_get_measurements_latest(imu_t * imu, imu_measurements_t * measurements);
int imu_comm_get_measurements_latest_unread(imu_t * imu, imu_measurements_t * measurements);
int imu_comm_get_data_raw_latest_unread(imu_t * imu, imu_data_t * data);

int imu_comm_print_data(imu_data_t * data, FILE * stream);

// -- -- -- -- -- -- -- -- -- -- -- --
// Configuring IMU
// -- -- -- -- -- -- -- -- -- -- -- --

int imu_comm_set_acc_sens(imu_t * imu, int new_value);
int imu_comm_get_acc_sens(imu_t * imu, int * acc_index);

int imu_comm_set_fs(imu_t * imu, int new_value);
int imu_comm_get_fs(imu_t * imu, int * fs_index);

// -- -- -- -- -- -- -- -- -- -- -- --
// Calibration
// -- -- -- -- -- -- -- -- -- -- -- --
uquad_bool_t imu_comm_calibration_is_calibrated(imu_t * imu);
int imu_comm_calibration_get(imu_t * imu, imu_null_estimates_t * calibration);
int imu_comm_calibration_start(imu_t * imu);
int imu_comm_calibration_abort(imu_t * imu);
int imu_comm_calibration_print(imu_null_estimates_t * calibration, FILE * stream);

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
