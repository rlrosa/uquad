#ifndef CONTROL_H
#define CONTROL_H

#include <macros_misc.h>
#include <uquad_error_codes.h>
#include <uquad_aux_math.h>
#include <path_planner.h>
#include <uquad_types.h>
#include <uquad_config.h>

#if FULL_CONTROL
/**
 * These matrices are loaded when full control
 * is used.
 * Matrices were obtained from the simulator,
 * using LQR.
 */
  #if CTRL_INTEGRAL
    #define CTRL_MAT_K_NAME   "K_prop_full.txt"
  #else
    #define CTRL_MAT_K_NAME   "K_full.txt"
  #endif // CTRL_INTEGRAL
  #define CTRL_MAT_K_INT_NAME "K_int_full.txt"

#else // FULL_CONTROL
/**
 * These matrices are loaded when partial control
 * is used.
 */
  #if CTRL_INTEGRAL
    #define CTRL_MAT_K_NAME   "K_prop.txt"
  #else
    #define CTRL_MAT_K_NAME   "K.txt"
  #endif // CTRL_INTEGRAL
  #define CTRL_MAT_K_INT_NAME "K_int.txt"
#endif // FULL_CONTROL

#if CTRL_INTEGRAL
/**
 * Threshold to limit integral control
 *   - Max state variations accepted (*DELTA*)
 *   - Max state diff accumulation (*ACCUM*)
 */
#define CTRL_INT_DELTA_MAX_PSI   1.74e-3// [rad*s] - @Ts=10ms, max 10° expected
#define CTRL_INT_DELTA_MAX_THETA PI/4.0 // [rad*s]
#define CTRL_INT_DELTA_MAX_Z     0.5    // [m]
#define CTRL_INT_DELTA_MAX_Y     1.0    // [m]
#define CTRL_INT_DELTA_MAX_X     1.0    // [m]
#define CTRL_INT_ACCUM_MAX_PSI   0.35   // [rad*s]
#define CTRL_INT_ACCUM_MAX_THETA 0.35   // [rad*s]
#define CTRL_INT_ACCUM_MAX_Z     2.0    // [m]
#define CTRL_INT_ACCUM_MAX_Y     5.0    // [m]
#define CTRL_INT_ACCUM_MAX_X     5.0    // [m]
#endif // CTRL_INTEGRAL

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

/**
 * Saves control matrices to file.
 *
 * @param ctrl
 * @param output log file, or NULL for stdout.
 *
 * @return error code
 */
int control_dump(ctrl_t *ctrl, FILE *output);

#endif
