#ifndef UQUAD_KALMAN_H
#define UQUAD_KALMAN_H
#include <uquad_aux_math.h>

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
  uquad_mat_t *u_; // la entrada en el instate anterior
} kalman_io_t;

#endif
