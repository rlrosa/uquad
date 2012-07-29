/**
 * control: lib for determing control actions.
 * Copyright (C) 2012  Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file   control.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for determing control actions.
 *
 * Expects to load gain matrices from text files. The number and name of the matrices depends
 * on the mode being used.
 *
 * //TODO:
 * An implementation of the LQR algorithm is complete, this should allow to calculate the gain
 * matrices on-the-go, allowing to change trajectories.
 * The missing element is the linealization function, it is not working yet. LQR requires a
 * linear version of the system as input.
 *
 * Examples:
 *   - src/test/control/control_test.c
 *   - src/main/main.c
 */
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
    #if CTRL_INTEGRAL_ANG
      #define CTRL_MAT_K_NAME   "K_prop_full_ppzt.txt"
    #else // CTRL_INTEGRAL_ANG
      #define CTRL_MAT_K_NAME   "K_prop_full.txt"
    #endif // CTRL_INTEGRAL_ANG
  #else // CTRL_INTEGRAL
    #define CTRL_MAT_K_NAME     "K_full.txt"
  #endif // CTRL_INTEGRAL

  #if CTRL_INTEGRAL_ANG
    #define CTRL_MAT_K_INT_NAME "K_int_full_ppzt.txt"
  #else // CTRL_INTEGRAL_ANG
    #define CTRL_MAT_K_INT_NAME "K_int_full.txt"
  #endif // CTRL_INTEGRAL_ANG
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

/**
 * LQR gain values are loaded from text files.
 * These will be used to generate new K matrices.
 */
#define CTRL_MAT_Q_NAME       "lqr-Q.txt"
#define CTRL_MAT_R_NAME       "lqr-R.txt"

#if CTRL_INTEGRAL
/**
 * Threshold to limit integral control
 *   - Max state variations accepted (*DELTA*)
 *   - Max state diff accumulation (*ACCUM*)
 */
#define CTRL_INT_DELTA_MAX_PSI   1.0e-3 // [rad*s] - @Ts=10ms, max 10° expected
#define CTRL_INT_DELTA_MAX_PHI   1.0e-3 // [rad*s] - @Ts=10ms, max 10° expected
#define CTRL_INT_DELTA_MAX_THETA 7.0e-3 // [rad*s]
#define CTRL_INT_DELTA_MAX_Z     0.25e-2 // [m*s]
#define CTRL_INT_DELTA_MAX_Y     1.0    // [m*s]
#define CTRL_INT_DELTA_MAX_X     1.0    // [m*s]
#define CTRL_INT_ACCUM_MAX_PSI   1.0    // [rad*s]
#define CTRL_INT_ACCUM_MAX_PHI   1.0    // [rad*s]
#define CTRL_INT_ACCUM_MAX_THETA 2.0    // [rad*s]
#define CTRL_INT_ACCUM_MAX_Z     2.0    // [m*s]
#define CTRL_INT_ACCUM_MAX_Y     5.0    // [m*s]
#define CTRL_INT_ACCUM_MAX_X     5.0    // [m*s]
#define CTRL_INT_TH_PSI          0.26   // [rad]
#define CTRL_INT_TH_PHI          0.26   // [rad]
#define CTRL_INT_TH_THETA        0.78   // [rad]
#define CTRL_INT_TH_Z            2.0    // [m]
#define CTRL_INT_TH_Y            1.0    // [m]
#define CTRL_INT_TH_X            1.0    // [m]
#endif // CTRL_INTEGRAL

#define CTRL_LQR_TH              1e-5   // Iteration threshold

#define CTRL_LIMIT_X             18.0   // [m]
#define CTRL_LIMIT_Y             18.0   // [m]
#define CTRL_LIMIT_Z             4.0    // [m]
#define CTRL_LIMIT_THETA         1.05   // [rad] - (corresponds to 60deg)


typedef struct ctrl{
    uquad_mat_t *K;
    uquad_mat_t *K_lqr;
#if CTRL_INTEGRAL
    uquad_mat_t *K_int; // Integral gain
    uquad_mat_t *x_int; // Integral term
#endif
    uquad_mat_t *A;
    uquad_mat_t *Acirc;
    uquad_mat_t *B;
    uquad_mat_t *Qhov;
    uquad_mat_t *Rhov;
    uquad_mat_t *Qrec;
    uquad_mat_t *Rrec;
    uquad_mat_t *Qcirc;
    uquad_mat_t *Rcirc;
    uquad_mat_t *Q;
    uquad_mat_t *R;
}ctrl_t;

/**
 * Allocate mem for ctrl, and loads gain from file.
 * File must have (in order):
 *   - Proportional gain matrix.
 *   - Integral gain matrix.
 *
 * @return structure or NULL if error.
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
 * The process is completed when update_complete is true. Several calls to this
 * funcion may be required, if calculation is timed out before completion. This
 * is required to avoid delaying the flight controller actions.
 *
 * @param ctrl
 * @param sp setpoint to update matrix to.
 * @param weight weight of the quadcopter.
 * @param update_complete indicator, true when matrix calculations have been completed.
 * @param start If !NULL, then a new matrix calculation is triggered. If NULL, then a calculation
 *              started previously is continued.
 *
 * @return error code
 */
int control_update_K(ctrl_t *ctrl, set_point_t *sp, double weight, uquad_bool_t *update_complete, uquad_bool_t *start);

/**
 * Solves Riccati equation, using an algorithm by Anders Friis Sorensen
 *
 * @param K
 * @param A
 * @param B 
 * @param Q
 * @param R
 * @parma update_complete Indicates if new control matrix calculation was completed.
 * @param start If !NULL, then a new matrix calculation is triggered. If NULL, then a calculation
 *              started previously is continued.
 * 
 * @return error code
 */
int control_lqr(uquad_mat_t *K, uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *Q, uquad_mat_t *R,
		uquad_bool_t *update_complete, uquad_bool_t *start);

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
