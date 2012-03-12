#ifndef UQUAD_KALMAN_H
#define UQUAD_KALMAN_H
#include <uquad_aux_math.h>
#include <imu_comm.h>

// Encabezados de las funciones del uquad_kalman.c que voy a llamar desde otro lado

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

int uquad_kalman(kalman_io_t * kalman_io_data, uquad_mat_t* w, imu_data_t* data);
kalman_io_t* kalman_init();

#endif
