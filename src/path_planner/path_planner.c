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
 * @file   path_planner.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for handling waypoints and progress along desired route.
 *
 */
#include "path_planner.h"
#include <uquad_types.h>
#include <mot_control.h>
#include <control.h>
#include <math.h>
#include <stdlib.h>


void setpoint_deinit(set_point_t *sp)
{
    if(sp == NULL)
	return;
    uquad_mat_free(sp->x);
    uquad_mat_free(sp->w);
    free(sp);
}

set_point_t *setpoint_init(void)
{
    int retval = ERROR_OK;
    set_point_t *sp = (set_point_t *)malloc(sizeof(set_point_t));
    mem_alloc_check(sp);
    memset(sp,0,sizeof(set_point_t));
    sp->x = uquad_mat_alloc(STATE_COUNT,1);
    mem_alloc_check(sp->x);
    sp->w = uquad_mat_alloc(LENGTH_INPUT,1);
    mem_alloc_check(sp->w);
    retval = uquad_mat_zeros(sp->x);
    if(retval != ERROR_OK)
    {
	err_log("setpoint_init() failed!");
	return NULL;
    }
    return sp;
}

path_planner_t *pp_init(void)
{
    path_planner_t *pp = (path_planner_t *)malloc(sizeof(path_planner_t));
    mem_alloc_check(pp);
    pp->pt = HOVER;
    pp->sp = setpoint_init();
    if(pp->sp == NULL)
    {
	pp_deinit(pp);
	return NULL;
    }
    return pp;
}

int pp_update_setpoint(path_planner_t *pp, uquad_mat_t *x, double w_hover, uquad_bool_t *ctrl_outdated)
{
    if(pp == NULL || x == NULL || ctrl_outdated == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument.");
    }
    if (pp->pt == HOVER)
    {
	pp->sp->w->m_full[0] = w_hover;
	pp->sp->w->m_full[1] = w_hover;
	pp->sp->w->m_full[2] = w_hover;
	pp->sp->w->m_full[3] = w_hover;
	*ctrl_outdated = false;
    }
    else
    {
	err_check(ERROR_FAIL, "Not implemented!");
    }
    return ERROR_OK;
}

int pp_new_setpoint(path_planner_t *pp, uquad_mat_t *x, uquad_mat_t *w)
{
    int retval = ERROR_OK;
    if(x != NULL)
    {
	retval = uquad_mat_copy(pp->sp->x, x);
	err_propagate(retval);
    }
    if(w != NULL)
    {
	retval = uquad_mat_copy(pp->sp->w, w);
	err_propagate(retval);
    }
    return retval;
}

void pp_deinit(path_planner_t *pp)
{
    if (pp == NULL)
	return;
    setpoint_deinit(pp->sp);
    free(pp);
}
