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

/**
 * Constants for deciding when the current setpoint has been
 * reached. If within range of the setpoint, a counter will
 * start, and arrival will be triggered only if the estimated
 * state is PP_REACHED_COUNT times within range.
 */
#define PP_REACHED_X     4.0  // [m]
#define PP_REACHED_Y     4.0  // [m]
#define PP_REACHED_Z     2.0  // [m]
#define PP_REACHED_COUNT 20

/// Load setpoints from file
#define PP_SP_X_FILENAME "pp_sp_x.txt"

typedef enum path_type{
    HOVER = 0,
    STRAIGHT,
    CIRCULAR,
    PATH_TYPE_COUNT
}path_type_t;

typedef struct set_point{
    path_type_t pt;
    uquad_mat_t *x; /// State vector
    uquad_mat_t *w; /// Motor speeds [rad/s]
}set_point_t;

typedef struct path_planner{
    set_point_t *sp;
    set_point_t **sp_list;
    int sp_list_len;
    int sp_list_curr;
}path_planner_t;

/**
 * Allocates memory for set points and path planner.
 * If filename is NULL the path will be only hovering. If a valid filename is
 * supplied then setpoints will be loaded. The file must have the following format:
 *   x.x x.x ... x.x N w.w w.w w.w w.w
 * where:
 *   - each x.x is one of the state variables given in uquad_types.h, without
 * including the accelerometer bias estimation.
 *   X Y Z PSI PHI THETA VQX VQY VQZ WQX WQY WQZ
 *   - the elemen N is an integer representing the path type, which can take 3 values:
 *       0:HOVER
 *       1:STRAIGHT
 *       2:CIRCULAR
 *   - each w.w is the setpoint for a motor relative to hovering (setting 0.0 will set
 *   the motor setpoint to hovering).
 *
 *
 * @param filename File to load path from, or NULL for only hovering.
 * @param x current state
 *
 * @return new structure, or NULL if error.
 */
path_planner_t *pp_init(const char *filename, double w_hover);

/**
 * Calculates distance between current state vector and set point, if it is
 * sufficiently small, then the set point should be set to the next waypoint,
 * and the control matrix should be updated to match the new trajectory.
 * This function will return an indicator so the control matrix can be
 * updated. After the control matrix is updated, pp_update_setpoint() should
 * be called.
 *
 * @param pp
 * @param x current state
 * @param ctrl_outdated if true, then control matrix has to be updated to new sp.
 *
 */
void pp_check_progress(path_planner_t *pp, uquad_mat_t *x, uquad_bool_t *arrived);

/**
 * Will update setpoint to the next in the list.
 * If no setpoints remain, then the next setpoint will be defined as hovering (initial condition).
 *
 * @param pp
 */
void pp_update_setpoint(path_planner_t *pp);

/**
 * Will return next setpoint in list.
 *
 * @param pp
 *
 * @return ans
 */
set_point_t *pp_get_next_sp(path_planner_t *pp);

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
