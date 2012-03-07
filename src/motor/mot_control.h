#ifndef MOT_CONTROL_H
#define MOT_CONTROL_H

#include <uquad_types.h>
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>
#include <stdio.h>
#include <stdint.h>

#define MOT_MAX_I2C 220
#define MOT_I2C_IDLE 50
#define MOT_C 4

#define START_MOTOR_CMD "sh ~/.cmd"
#define KILL_MOTOR_CMD "kill -INT `pidof -s cmd"

/// Communication with i2c handling program si done through file
#define MOT_COMM_FILE "mot_control.file"

/// Log data
#define MOT_LOG_TX "tx.log"

#define MAX_W 422

/**
 * Structure to hold motor status.
 * 
 * Motors are defined as:
 *  0: 0x69
 *  1: 0x6a
 *  2: 0x6b
 *  3: 0x68
 * 
 */
typedef struct uquad_mot{
    FILE *ctrl_file;
    FILE *tx_log;
    int i2c_curr[MOT_C];
    int i2c_target[MOT_C];
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
int mot_set_vel_rads(uquad_mot_t *mot, int *w);

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
 * @return error code
 */
int mot_deinit(uquad_mot_t *mot);
#endif //MOT_CONTROL_H
