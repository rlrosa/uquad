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

uquad_mot_t *mot_init(void);

int mot_set_vel_rads(uquad_mot_t *mot, int *v);
int mot_set_idle(uquad_mot_t *mot);
int mot_stop(uquad_mot_t *mot);

int mot_deinit(uquad_mot_t *mot);
#endif //MOT_CONTROL_H
