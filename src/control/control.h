#ifndef CONTROL_H
#define CONTROL_H

#include <macros_misc.h>
#include <uquad_error_codes.h>
#include <uquad_aux_math.h>
#include <path_planner.h>
#include <uquad_types.h>

#define CTRL_INTEGRAL       0
#if FULL_CONTROL
#define CTRL_MAT_K_NAME     "K_full.txt"
#define CTRL_MAT_K_INT_NAME "K_int_full.txt"
#else
#define CTRL_MAT_K_NAME     "K.txt"
#define CTRL_MAT_K_INT_NAME "K_int.txt"
#endif // FULL_CONTROL

typedef struct ctrl{
    uquad_mat_t *K;
#if CTRL_INTEGRAL
    uquad_mat_t *K_int; // Integral gain
    uquad_mat_t *x_int; // Integral term
#endif
}ctrl_t;

/**
 * Allocate mem for ctrl, and loads gain from file.
 * File must have (in order):
 *   - Proportional gain matrix.
 *   - Integral gain matrix.
 *
 * @return
 */
ctrl_t *control_init(void);

/** 
 * Calculates action (motor speed) that should be performed on system.
 * 
 * @param ctrl 
 * @param w Answer                           [rad/s]
 * @param x Current state estimation
 * @param sp Current setpoint
 * @param T_us Time since las control action [us]
 * 
 * @return 
 */
int control(ctrl_t *ctrl, uquad_mat_t *w, uquad_mat_t *x, set_point_t *sp, double T_us);

void control_deinit(ctrl_t *ctrl);

#endif
