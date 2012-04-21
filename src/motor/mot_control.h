#ifndef MOT_CONTROL_H
#define MOT_CONTROL_H

#include <uquad_types.h>
#include <uquad_aux_math.h>
#include <uquad_aux_time.h>
#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <stdio.h>
#include <stdint.h>

#define MOT_I2C_MAX         220     // i2c
#define MOT_I2C_IDLE        50      // i2c
#define MOT_W_MAX           387.0L  // rad/s (match MOT_MAX_I2C)
#define MOT_W_MIN           (MOT_W_HOVER - (MOT_W_MAX - MOT_W_HOVER))
#define MOT_W_IDLE          109.0L  // rad/s (match MOT_IDLE_I2C)
#define MOT_W_STARTUP_RANGE (MOT_W_HOVER - MOT_W_IDLE)
#define MOT_C               4

#define MOT_UPDATE_MAX_US   2000  // us
#define MOT_WAIT_STARTUP_MS 420   // ms
#define MOT_WAIT_STARTUP_S  3     // s

#define START_MOTOR_CMD "./cmd > cmd_output.log &"
#define KILL_MOTOR_CMD  "kill -INT `pidof -s cmd` > /dev/null &"

/// Communication with motor driver is done via kernel msgs
#define MOT_SERVER_KEY 169 // some number
#define MOT_DRIVER_KEY 170 // some other number

/// Log data
#define MOT_LOG_TX "cmd_tx.log"

/**
 * Structure to hold motor status.
 * 
 * Motors are defined as:
 *  0: 0x69
 *  1: 0x6b
 *  2: 0x6a
 *  3: 0x68
 *
 * The driver order is:
 *  0: 0x69
 *  1: 0x6a
 *  2: 0x6b
 *  3: 0x68
 * Order is adapted before sending
 * 
 */
typedef struct uquad_mot{
    FILE *tx_log;
    uint8_t i2c_curr[MOT_C];
    uint8_t i2c_target[MOT_C];
    uquad_mat_t *w_curr;
    uquad_kmsgq_t *kmsgq;
    struct timeval last_set;
}uquad_mot_t;

/** 
 * Allocates memory for uquad_mot_t, opens log
 * files and opens file for interaction will motor controlling
 * program.
 * 
 * @return Instance of uquad_mot_t, or NULL if error.
 */
uquad_mot_t *mot_init(void);

/** 
 * Takes an array of rads/s and sets the corresponding
 * i2c value as target speed.
 * 
 * @param mot 
 * @param w target speed, in rad/s
 * 
 * @return error code
 */
int mot_set_vel_rads(uquad_mot_t *mot, uquad_mat_t *w);

/** 
 * Sets idle speed as target speed for all motors.
 * The motors will be running, but not fast enough
 * to move the cuadcopter.
 * 
 * @param mot 
 * 
 * @return error code
 */
int mot_set_idle(uquad_mot_t *mot);

/** 
 * Sets speed to zero for all motors.
 * 
 * @param mot 
 * 
 * @return error code
 */
int mot_stop(uquad_mot_t *mot);

/** 
 * Cleans up uquad_mot_t, closing all opened files and
 * freeing memory.
 * Will only close opened files (will check), and will
 * check if mem was previously allocated before attemping
 * to free.
 * 
 * @param mot 
 * 
 * @return error code to indicate if system call to kill
 * driver was successfull.
 */
int mot_deinit(uquad_mot_t *mot);
#endif //MOT_CONTROL_H
