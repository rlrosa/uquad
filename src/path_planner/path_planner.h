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
    uquad_mat_t *x; /// State vector
    uquad_mat_t *w; /// Motor speeds [rad/s]
}set_point_t;

typedef struct path_planner{
    path_type_t pt;
    set_point_t *sp;
}path_planner_t;

/**
 * Allocates memory for set point structure
 *
 * NOTE: Will NOT select set point, this must be done by
 *       either pp_update_setpoint() or pp_new_setpoint().
 *
 * @return new structure, or NULL if error.
 */
path_planner_t *pp_init(void);

/**
 * Calculates distance between current state vector and set point, if it is
 * sufficiently small, then the set point should be set to the next waypoint,
 * and the control matrix should be updated to match the new trajectory.
 *
 * @param pp
 * @param x current state
 * @param w_hover
 * @param ctrl_outdated if true, then control matrix has to be updated to new sp.
 *
 * @return error code
 */
int pp_update_setpoint(path_planner_t *pp, uquad_mat_t *x, double w_hover, uquad_bool_t *ctrl_outdated);

/**
 * Will set x (state vector) as current setpoint, and w as desired motor speed.
 * This function can come in handy to set a hovering set point before taking off,
 * avoiding calculations during takeoff.
 *
 * @param pp
 * @param x  New setpoint (state vector) or NULL if it shouldn't change.
 * @param w New setpoint (motor speed) or NULL if it shouldn't change.
 *
 * @return error code
 */
int pp_new_setpoint(path_planner_t *pp, uquad_mat_t *x, uquad_mat_t *w);

/**
 * Will free memory allocated for path planner.
 *
 * @param pp
 */
void pp_deinit(path_planner_t *pp);

#endif // PATH_PLANNER_H
