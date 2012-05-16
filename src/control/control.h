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
#define CTRL_INT_DELTA_MAX_PHI   1.74e-3// [rad*s] - @Ts=10ms, max 10° expected
#define CTRL_INT_DELTA_MAX_THETA 1.74e-3// [rad*s]
#define CTRL_INT_DELTA_MAX_Z     1e-5   // [m*s]
#define CTRL_INT_DELTA_MAX_Y     1.0    // [m*s]
#define CTRL_INT_DELTA_MAX_X     1.0    // [m*s]
#define CTRL_INT_ACCUM_MAX_PSI   0.35   // [rad*s]
#define CTRL_INT_ACCUM_MAX_PHI   0.35   // [rad*s]
#define CTRL_INT_ACCUM_MAX_THETA 11.54   // [rad*s]
#define CTRL_INT_ACCUM_MAX_Z     9.42    // [m*s]
#define CTRL_INT_ACCUM_MAX_Y     5.0    // [m*s]
#define CTRL_INT_ACCUM_MAX_X     5.0    // [m*s]
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
 * @return ctrl structure, or NULL if error.
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
 * @return error code
 */
int control(ctrl_t *ctrl, uquad_mat_t *w, uquad_mat_t *x, set_point_t *sp, double T_us);


/**
 * This functions returns the discrete LQR realimentation matrix.
 * If a continuous time system verifies:
 *     \dot{X}=AX+BU
 * where X is the state vector and u the input of the system, then
 * the discretization of the system is:
 *     X(k+1)=phi*X(k)+gamma*U
 * where:
 *     phi=exp(A*Ts)
 *     gama=int(exp(A*s),0,Ts)
 *
 * @param pp current pp, will be updated.
 * @param weight weight of the quadcopter.
 *
 * @return error code
 */
int control_update_K(path_planner_t *pp, double weight);

/**
 * Solves Riccati equation, using an algorithm by Anders Friis Sorensen
 *
 * @param K
 * @param A
 * @param B 
 * @param Q
 * @param R
 * 
 * @return error code
 */
int control_lqr(uquad_mat_t *K, uquad_mat_t *phi, uquad_mat_t *gamma, uquad_mat_t *Q, uquad_mat_t *R);

/**
 * Returns the discrete form X(k+1)=phi*X(k)+gamma*U of a
 * system \dot{x} = AX+BU
 *
 * @param phi
 * @param gamma
 * @param A
 * @param B
 * @param Ts sampling period [s]
 *
 * @return error code
 */
int control_disc(uquad_mat_t *phi,uquad_mat_t *gamma,uquad_mat_t *A,uquad_mat_t *B, double Ts);

/**
 * Performs the linearization of the physic model of the quadrotor:
 *      \dot{X} = A*X+B*U
 * 
 * @param A
 * @param B
 * @param pt type of trajectory
 * @param sp current set point
 * @param weight of the quadcopter
 * 
 * @return error code
 */
int control_lin_model(uquad_mat_t *A, uquad_mat_t *B, path_type_t pt, set_point_t *sp, double weight);

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
