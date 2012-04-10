#ifndef CONTROL_H
#define CONTROL_H

#include <macros_misc.h>
#include <uquad_error_codes.h>
#include <uquad_aux_math.h>
#include <path_planner.h>

#define STATES_CONTROLLED 8
#define CTRL_INTEGRAL     1

typedef struct ctrl{
    uquad_mat_t *K;
#if CTRL_INTEGRAL
    uquad_mat_t *K_int; // Integral gain
    uquad_mat_t *x_int; // Integral term
#endif
}ctrl_t;

ctrl_t *control_init(void);

int control(ctrl_t *ctrl, uquad_mat_t *w, uquad_mat_t *x, set_point_t *sp);

void control_deinit(ctrl_t *ctrl);

#endif
