#ifndef UQUAD_KALMAN_H
#define UQUAD_KALMAN_H
#include <uquad_aux_math.h>
#include <imu_comm.h>

/**
 * Kalman struct description:
 *
 * x_hat: State vector
 *    [x         ] - m
 *    [y         ] - m
 *    [z         ] - m
 *    [psi/roll  ] - rad
 *    [phi/pitch ] - rad
 *    [theta/yaw ] - rad
 *    [vqx       ] - m/s
 *    [vqy       ] - m/s
 *    [vqz       ] - m/s
 *    [wqx       ] - rad/s
 *    [wqy       ] - rad/s
 *    [wqz       ] - rad/s
 *
 * T: Time since last sample, in us
 *
 */
typedef struct kalman_io {
    uquad_mat_t *x_hat;
    uquad_mat_t *u;
    double T;
    uquad_mat_t *z;
    uquad_mat_t *R;
    uquad_mat_t *Q;
    uquad_mat_t *P;
    uquad_mat_t *x_;
} kalman_io_t;

int uquad_kalman(kalman_io_t * kalman_io_data, uquad_mat_t* w, imu_data_t* data, double T);
kalman_io_t* kalman_init();

void kalman_deinit(kalman_io_t *kalman_io_data);

#endif
