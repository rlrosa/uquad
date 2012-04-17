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

/// Default path for calibration file.
#define IMU_DEFAULT_CALIB_PATH     "imu_calib.txt"

#define IMU_DEBUG                  0

#define IMU_COMM_PRINT_DATA_FORMAT "%0.8f\t%0.8f\t%0.8f\t%0.8f\t%0.8f\t%0.8f\t%0.8f\t%0.8f\t%0.8f\t%0.8f\t%0.8f\t%0.8f\n"
#define IMU_COMM_PRINT_RAW_FORMAT  "%u\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%u\t%u\n"

/**
 * Setting this define to 1 will make imu_comm
 * interact with a w/r serial port transmitting binary
 * data.
 * Setting it to 0 will allow reading from a read only 
 * ascii log file.
 * 
 */
#define IMU_COMM_FAKE                     0

/**
 * Definition of default sampling period, in microseconds.
 * Should match code running on IMU.
 */
#define TS_DEFAULT_US                     10000L

#define IMU_FRAME_ELEMENTS                12
#define IMU_FRAME_ALTERNATES_INIT         0
#define IMU_FRAME_INIT_CHAR               'A'
#define IMU_FRAME_INIT_CHAR_ALT           'C'
#define IMU_FRAME_INIT_DIFF               0x2
#define IMU_FRAME_END_CHAR                'Z'
#define IMU_INIT_END_SIZE                 1
#define IMU_DEFAULT_FRAME_SIZE_BYTES      30
#define IMU_DEFAULT_FRAME_SIZE_DATA_BYTES IMU_DEFAULT_FRAME_SIZE_BYTES - 6 // init,end,time
#define IMU_SYNC_FAIL_MAX                 IMU_DEFAULT_FRAME_SIZE_BYTES

#define IMU_FRAME_BUFF_SIZE               32

#define IMU_AVG_COUNT                     1 // Reduce variance my taking avg
#define IMU_CALIB_SIZE                    512 //TODO Tune!

#define IMU_GYRO_DEFAULT_GAIN             14.375L // Not used
#define IMU_P0_DEFAULT                    101325.0L // Value used if no calibration is available.

#define IMU_TH_DEADLOCK_ACC               9.72L // m/s^2
#define IMU_TH_DEADLOCK_ANG               0.17069L // rad

#define IMU_BYTES_T_US                    4

#define IMU_COMM_AVG_MAX_INTERVAL         2*IMU_FRAME_SAMPLE_AVG_COUNT //Too much...?

/// ASCII 35, exits from menu and runs unit
#define IMU_COMMAND_RUN                   '#'
/// ASCII 36, stops sampling and shows menu
#define IMU_COMMAND_STOP                  '$'

/// ASCII 33, sets the unit in default mode
#define IMU_COMMAND_DEF                   '!'

/// If READ_RETRIES > 1, then reading may block.
#define READ_RETRIES                      1

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
typedef struct imu_frame{
    uint16_t T_us;   // Time since previous sample in us
    int16_t acc [3]; // ADC counts
    int16_t gyro[3]; // ADC counts
    int16_t magn[3]; // ADC counts
    uint16_t temp;   // Tens of °C
    uint32_t pres;   // Pa
    struct timeval timestamp;
}imu_raw_t;

/**
 * Extended raw struct, for calibration accumulation.
 * Calibration will average a bunch of samples to get an
 * accurate null estimate.
 * 
 */
typedef struct imu_frame_null{
    int32_t acc [3]; // ADC counts
    int32_t gyro[3]; // ADC counts
    int32_t magn[3]; // ADC counts
    uint32_t temp;   // Tens of °C
    uint64_t pres;   // Pa
}imu_raw_null_t;

/**
 * imu_data_t: Stores calibrated data. This is the final output of imu_comm,
 * and should be passed on the the control loop.
 */
typedef struct imu_data{
    double T_us;       // us
    uquad_mat_t *acc;  // m/s^2
    uquad_mat_t *gyro; // rad/s
    uquad_mat_t *magn; // rad - Euler angles - {psi/roll,phi/pitch,theta/yaw}
    double temp;       // °C
    double alt;        // m

    struct timeval timestamp;
}imu_data_t;

/**
 * Acc, gyro and magn use a linear model:
 *   data = T*K_inv*(raw - b)
 *   - raw  : Raw data from sensor, in bits
 *   - b    : Offset.
 *   - K_inv: Inverse of a diagonal gain matrix.
 *   - T    : Cross axis sensitivity
 *
 * Struct stores TK_inv == T*inv(K)
 *
 * K=[kx 0 0;
 *    0 ky 0;
 *    0 0 kz];
 *
 * b=[bx;
 *    by;
 *    bz];
 *
 * T= [1  -yz zy;
 *     xz  1 -zx;
 *    -xy  yx 1];
 */
typedef struct imu_calibration_lin_model{
    uquad_mat_t *TK_inv;
    uquad_mat_t *b;
}imu_calib_lin_t;

/**
 * Stores calibration parameteres for each of the 3 sensors
 * that use a linear model imu_calib_lin_t(): acc,gyro and  magn
 */
typedef struct imu_calibration{
    imu_calib_lin_t m_lin[3];      // {acc,gyro,magn}.
    uquad_mat_t *acc_t_off;        // acc temp offset {x,y,z}  [m/(s^2°C)]
    double acc_to;                 // acc calibration temp
    uquad_mat_t *gyro_t_off;       // gyro temp offset {x,y,z} [rad/(s)]
    double gyro_to;                // gyro calibration temp
    struct timeval timestamp_file; // time at which calib was read.
    uquad_bool_t calib_file_ready; // calibration was read from file.

    imu_raw_t null_est;            // null estimates gathered this run.
    imu_data_t null_est_data;      // null estimates, converted
    struct timeval timestamp_estim;// time at which null estimate finished.
    uquad_bool_t calib_estim_ready;// null estimates are ready.
    int calibration_counter;       // current number of frames available for calibration.
}imu_calib_t;

/**
 * If IMU setting were to be modified from imu_comm, the 
 * current setting should be stored here.
 * //TODO use or remove
 * 
 */
typedef struct imu_settings{
    // sampling frequency
    //    int fs;
    // sampling period
    //    double T;
    // sens index
    //    int acc_sens;
    //    int frame_width_bytes;
}imu_settings_t;

enum imu_status{
    IMU_COMM_STATE_RUNNING,
    IMU_COMM_STATE_STOPPED,
    IMU_COMM_STATE_CALIBRATING,
    IMU_COMM_STATE_UNKNOWN};
typedef enum imu_status imu_status_t;

typedef struct imu{
#if IMU_COMM_FAKE
    FILE *device;
#else
    int device;
#endif
    imu_status_t status;
    /// calibration
    imu_calib_t calib;
    /// data
    imu_raw_t frame_buff[IMU_FRAME_BUFF_SIZE];
    int frame_buff_latest; // last sample is here
    int frame_buff_next;   // new data will go here
    int unread_data;

    /// avg
    int frame_count;       // # of frames available for avg
    imu_data_t tmp_avg;    // Aux mem used for avg.
}imu_t;

imu_t *imu_comm_init(const char *device);
int imu_comm_deinit(imu_t *imu);

imu_status_t imu_comm_get_status(imu_t *imu);

#if !IMU_COMM_FAKE
int imu_comm_stop(imu_t *imu);
int imu_comm_resume(imu_t *imu);
#endif

int imu_data_alloc(imu_data_t *imu_data);
void imu_data_free(imu_data_t *imu_data);

// -- -- -- -- -- -- -- -- -- -- -- --
// Reading from IMU
// -- -- -- -- -- -- -- -- -- -- -- --

int imu_comm_get_fds(imu_t *imu, int *fds);
int imu_comm_read(imu_t *imu, uquad_bool_t *ready);

uquad_bool_t imu_comm_unread(imu_t *imu);

int imu_comm_get_data_latest(imu_t *imu, imu_data_t *data);
int imu_comm_get_data_latest_unread(imu_t *imu, imu_data_t *data);
int imu_comm_get_raw_latest(imu_t *imu, imu_raw_t *raw);
int imu_comm_get_raw_latest_unread(imu_t *imu, imu_raw_t *raw);

uquad_bool_t imu_comm_avg_ready(imu_t *imu);
int imu_comm_get_avg(imu_t *imu, imu_data_t *data);
int imu_comm_get_avg_unread(imu_t *imu, imu_data_t *data);

int imu_comm_raw2data(imu_t *imu, imu_raw_t *raw, imu_data_t *data);

int imu_comm_print_data(imu_data_t *data, FILE *stream);
int imu_comm_print_raw(imu_raw_t *frame, FILE *stream);
int imu_comm_print_calib(imu_calib_t *calib, FILE *stream);

// -- -- -- -- -- -- -- -- -- -- -- --
// Calibration
// -- -- -- -- -- -- -- -- -- -- -- --
uquad_bool_t imu_comm_calib_file(imu_t *imu);
uquad_bool_t imu_comm_calib_estim(imu_t *imu);
int imu_comm_calib_save(imu_t *imu, const char *filename);

int imu_comm_calibration_start(imu_t *imu);
int imu_comm_calibration_abort(imu_t *imu);

int imu_comm_calibration_get(imu_t *imu, imu_calib_t **calib);

#endif // IMU_COMM_H












// -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Notes and unused code
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#if 0

uquad_bool_t imu_comm_avg_ready(imu_t *imu);
int imu_comm_get_avg(imu_t *imu, imu_data_t *data_avg);


// -- -- -- -- -- -- -- -- -- -- -- --
// Configuring IMU
// -- -- -- -- -- -- -- -- -- -- -- --

int imu_comm_set_acc_sens(imu_t *imu, int new_value);
int imu_comm_get_acc_sens(imu_t *imu, int *acc_index);

int imu_comm_set_fs(imu_t *imu, int new_value);
int imu_comm_get_fs(imu_t *imu, int *fs_index);

int imu_comm_calibration_get(imu_t *imu, imu_calib_t **calib);
int imu_comm_calibration_start(imu_t *imu);
int imu_comm_calibration_abort(imu_t *imu);
int imu_comm_calibration_print(imu_calib_t *calib, FILE *stream);


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
