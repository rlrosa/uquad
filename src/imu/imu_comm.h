#ifndef IMU_COMM_H
#define IMU_COMM_H

#include <sys/time.h>
#include <sys/select.h>
#include <uquad_error_codes.h>
#include <uquad_types.h>
#include <uquad_aux_time.h>
#include <uquad_aux_math.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#define IMU_DEFAULT_CALIB_PATH "imu_calib.txt"

#define IMU_FRAME_INIT_CHAR 'A'
#define IMU_FRAME_INIT_CHAR_ALT 'C'
#define IMU_FRAME_INIT_DIFF 0x2
#define IMU_FRAME_END_CHAR 'Z'
#define IMU_INIT_END_SIZE 1
#define IMU_DEFAULT_FRAME_SIZE_BYTES 30
#define IMU_DEFAULT_FRAME_SIZE_DATA_BYTES IMU_DEFAULT_FRAME_SIZE_BYTES - 6 // init,end,time
#define IMU_FRAME_BUFF_SIZE 16
#define IMU_FRAME_SAMPLE_AVG_COUNT 8 // Reduce variance my taking avg
#define IMU_COMM_CALIBRATION_NULL_SIZE 256 // Tune!
#define IMU_DEFAULT_FS 5 // this is an index
#define IMU_DEFAULT_ACC_SENS 0 // this is an index

#define IMU_SAMPLE_COUNT_SIZE 2
#define IMU_BYTES_PER_SENSOR 2
#define IMU_BYTES_T_US 4
#define IMU_SENSOR_COUNT 6
#define IMU_ACCS 3
#define IMU_GYROS 3
#define IMU_SENS_OPT_COUNT 4
#define IMU_FS_OPT_COUNT 1
#define IMU_GRAVITY 9.81

#define IMU_COMM_AVG_MAX_INTERVAL 2*IMU_FRAME_SAMPLE_AVG_COUNT //Too much...?

/// ASCII 35, exits from menu and runs unit
#define IMU_COMMAND_RUN '#'
/// ASCII 36, stops sampling and shows menu
#define IMU_COMMAND_STOP '$'

/// ASCII 33, sets the unit in default mode
#define IMU_COMMAND_DEF '!'

#define READ_RETRIES 16

/**
 * Raw data from IMU
 * Frame has 29 bytes, 27 without init/end.
 *
 *   [1b:Init char]
 *   [4b:T        ]
 *   [2b:Acc X    ]
 *   [2b:Acc Y    ]
 *   [2b:Acc Z    ]
 *   [2b:Gyro X   ]
 *   [2b:Gyro Y   ]
 *   [2b:Gyro Z   ]
 *   [2b:Magn X   ]
 *   [2b:Magn Y   ]
 *   [2b:Magn Z   ]
 *   [1b:Temp     ]
 *   [4b:Press    ]//TODO use relative press to save bw
 *   [1b:End char ]
 * 
 * NOTE: No separation chars.
 */
struct imu_frame{
    uint16_t T_us; // Time since previous sample in us
    int16_t acc[3];
    int16_t gyro[3];
    int16_t magn[3];
    uint16_t temp;
    uint32_t pres;
    struct timeval timestamp;
};
typedef struct imu_frame imu_raw_t;

/// Calibrated data
struct imu_data{
    double T_us;
    uquad_mat_t *acc;  // m/s^2
    uquad_mat_t *gyro; // °/s
    uquad_mat_t *magn; // °
    double temp;       // °C
    double alt;        // m

    struct timeval timestamp;
};
typedef struct imu_data imu_data_t;

struct imu_calibration_lin_model{
    // data = T*K_inv*(raw - b)
    uquad_mat_t *K_inv;
    uquad_mat_t *T;
    uquad_mat_t *b;
};
typedef struct imu_calibration_lin_model imu_calib_lin_t;

struct imu_calibration{
    imu_calib_lin_t m_lin[3]; //{acc,gyro,magn}
    struct timeval timestamp;
};
typedef struct imu_calibration imu_calib_t;


struct imu_settings{
    // sampling frequency
    int fs;
    // sampling period
    double T;
    // sens index
    int acc_sens;
    int frame_width_bytes;
};
typedef struct imu_settings imu_settings_t;

enum imu_status{
    IMU_COMM_STATE_RUNNING,
    IMU_COMM_STATE_STOPPED,
    IMU_COMM_STATE_CALIBRATING,
    IMU_COMM_STATE_UNKNOWN};
typedef enum imu_status imu_status_t;

struct imu{
    // config & status
    imu_settings_t settings;
    imu_status_t status;
    uquad_bool_t in_cfg_mode;
    FILE * device;
    // calibration
    imu_calib_t calib;
    int calibration_counter;
    uquad_bool_t is_calibrated;
    // data
    imu_raw_t frame_buff[IMU_FRAME_BUFF_SIZE];
    int frame_buff_latest; // last sample is here
    int frame_buff_next; // new data will go here
    struct timeval frame_avg_init,frame_avg_end;
    int frames_sampled;
    int unread_data;
    uquad_bool_t avg_ready;
    imu_data_t avg;
};
typedef struct imu imu_t;

imu_t *imu_comm_init(const char *device);
int imu_comm_deinit(imu_t *imu);

imu_status_t imu_comm_get_status(imu_t *imu);

// -- -- -- -- -- -- -- -- -- -- -- --
// Reading from IMU
// -- -- -- -- -- -- -- -- -- -- -- --

int imu_comm_get_fds(imu_t *imu, int *fds);
int imu_comm_read(imu_t *imu, uquad_bool_t *success);

uquad_bool_t imu_comm_avg_ready(imu_t *imu);
int imu_comm_get_avg(imu_t *imu, imu_data_t *data_avg);

int imu_comm_get_data_latest(imu_t *imu, imu_data_t *data);
int imu_comm_get_data_latest_unread(imu_t *imu, imu_data_t *data);
int imu_comm_get_raw_latest_unread(imu_t *imu, imu_raw_t *raw);

int imu_comm_print_data(imu_data_t *data, FILE *stream);
int imu_comm_print_raw(imu_raw_t *frame, FILE *stream);
int imu_comm_print_calib(imu_calib_t *calib, FILE *stream);

// -- -- -- -- -- -- -- -- -- -- -- --
// Configuring IMU
// -- -- -- -- -- -- -- -- -- -- -- --

int imu_comm_set_acc_sens(imu_t *imu, int new_value);
int imu_comm_get_acc_sens(imu_t *imu, int *acc_index);

int imu_comm_set_fs(imu_t *imu, int new_value);
int imu_comm_get_fs(imu_t *imu, int *fs_index);

// -- -- -- -- -- -- -- -- -- -- -- --
// Calibration
// -- -- -- -- -- -- -- -- -- -- -- --
uquad_bool_t imu_comm_calibration_is_calibrated(imu_t *imu);
int imu_comm_calibration_get(imu_t *imu, imu_calib_t **calib);
int imu_comm_calibration_start(imu_t *imu);
int imu_comm_calibration_abort(imu_t *imu);
int imu_comm_calibration_print(imu_calib_t *calib, FILE *stream);

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
