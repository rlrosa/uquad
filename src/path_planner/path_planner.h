/**
 * path_planner: lib for handling waypoints and progress along desired route.
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
 * @file   path_planner.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for handling waypoints and progress along desired route.
 *
 * //TODO:
 *   - Currently the only type of trajectory supported is hovering.
 *   - Waypoint system is not implemented.
 *
 * Examples:
 *   - src/main/main.c
 *   - src/test/gps/gps_test.c
 */
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <uquad_aux_math.h>

typedef enum path_type{
    HOVER = 0,
    STRAIGHT,
    CIRCULAR,
    PATH_TYPE_COUNT
}path_type_t;

typedef struct set_point{
    uquad_mat_t *x;
    uquad_mat_t *w;
}set_point_t;

typedef struct path_planner{
    path_type_t pt;
    set_point_t *sp;
    uquad_mat_t *A;
    uquad_mat_t *Acirc;
    uquad_mat_t *B;
    double Ts;
    uquad_mat_t *Qhov;
    uquad_mat_t *Rhov;
    uquad_mat_t *Qrec;
    uquad_mat_t *Rrec;
    uquad_mat_t *Qcirc;
    uquad_mat_t *Rcirc;
    uquad_mat_t *Q;
    uquad_mat_t *R;
    uquad_mat_t *K;
}path_planner_t;

path_planner_t *pp_init(void);

int pp_update_setpoint(path_planner_t *pp, uquad_mat_t *x, double w_hover);

void pp_deinit(path_planner_t *pp);

int pp_update_K(path_planner_t *pp, double weight);

int pp_lqr(uquad_mat_t *K, uquad_mat_t *phi, uquad_mat_t *gamma, uquad_mat_t *Q, uquad_mat_t *R);
int pp_disc(uquad_mat_t *phi,uquad_mat_t *gamma,uquad_mat_t *A,uquad_mat_t *B, double Ts);
#endif //PATH_PLANNER_H

int pp_lin_model(uquad_mat_t *A, uquad_mat_t *B, path_type_t pt, set_point_t *sp, double weight);
