/**
 * uquad_kalman: lib for EKF.
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
 * @file   uquad_kalman.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for EKF.
 *
 * Examples:
 *   - src/main/main.c
 *   - src/test/kalman_test/kalman_tests.c
 */
#ifndef UQUAD_KALMAN_H
#define UQUAD_KALMAN_H
#include <uquad_aux_math.h>
#include <imu_comm.h>
#include <uquad_gps_comm.h>

#define KALMAN_ROWS_H_GPS 12
#define KALMAN_ROWS_H     10

/**
 * Dinamically adjust acc/magn covariance based on the norm
 * of the reading. If it differs by too much (threshold is
 * given by imu_comm) from the expected value then acc/magn
 * should not be trusted, and covariance increased.
 *
 * Pseudocode:
 *   if(imu->acc_ok)
 *   {
 *     cov_psi = COV_PSI_OK
 *     cov_phi = COV_PHI_OK
 *   }
 *   else
 *   {
 *     cov_psi = COV_PSI_BAD
 *     cov_phi = COV_PHI_BAD
 *   }
 *
 * The situation is analogous with theta and the magnetometer.
 */
#define COV_PSI_OK        1e2
#define COV_PSI_BAD       1e9
#define COV_PHI_OK        COV_PSI_OK
#define COV_PHI_BAD       COV_PSI_BAD
#define COV_THE_OK        1e5
#define COV_THE_BAD       1e9
#define COV_ACC_OK        1e4
#define COV_ACC_BAD       1e4

/**
 * Kalman struct description.
 * State vectors x_hat and x_ are described in uquad_types.h
 *
 * T: Time since last sample, in us
 * weight: Weight of the cuadcopter - kg
 *
 */
typedef struct kalman_io {
    uquad_mat_t *x_hat;
    uquad_mat_t *u;
    uquad_mat_t *z;
    uquad_mat_t *z_gps;
    double T;
    double weight;
    uquad_mat_t *x_;
    uquad_mat_t *R;
    uquad_mat_t *Q;
    uquad_mat_t *P;
    uquad_mat_t *P_;
    uquad_mat_t *R_imu_gps;
} kalman_io_t;

/**
 * Run EKF algorithm.
 * New estimated state will be saved in kd->x_hat, and covariance
 * matrices will be updated.
 *
 * NOTE: Will modify input argument data to assure continuity on theta.
 *
 * @param kd Current state of kalman state machine.
 * @param w Speed that will be applied to the motors.
 * @param data IMU data. theta will be modified to mantain continuity.
 * @param T_us Time since last sample, in microseconds.
 * @param weight Weight of the quadcopter.
 * @param gps_i_data Data from GPS.
 *
 * @return error code.
 */
int uquad_kalman(kalman_io_t * kd, uquad_mat_t* w,
		 imu_data_t* data, double T_us,
		 double weight, gps_comm_data_t *gps_i_data);

int uquad_kalman_gps(kalman_io_t* kd, gps_comm_data_t* gps_i_data);

/**
 * Will startup kalman structure, allocate memory and initialize all
 * data.
 *
 * @return initialized structure or NULL if error.
 */
kalman_io_t* kalman_init();

/**
 * Frees all memory allocated for kalman.
 *
 * @param kd
 */
void kalman_deinit(kalman_io_t *kd);

/**
 * Saves Kalman noise settings to file
 *
 * @param kd
 * @param output log file, or NULL for stdout.
 *
 * @return error code
 */
int kalman_dump(kalman_io_t *kd, FILE *output);

#endif
