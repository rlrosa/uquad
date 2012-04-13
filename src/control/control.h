#ifndef CONTROL_H
#define CONTROL_H

#include <macros_misc.h>
#include <uquad_error_codes.h>
#include <uquad_aux_math.h>
#include <path_planner.h>

#define STATES_CONTROLLED 8
#define CTRL_INTEGRAL     0

typedef struct ctrl{
    uquad_mat_t *K;
#if CTRL_INTEGRAL
    uquad_mat_t *K_int; // Integral gain
    uquad_mat_t *x_int; // Integral term
#endif
}ctrl_t;

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
