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

static int line_count (const char *filename, int *lines)
{
    char
	c;
    FILE
	*file;
    if(filename == NULL || lines == NULL)
    {
	err_check(ERROR_INVALID_ARG,"NULL pointer invalid arg!");
    }
    file = fopen(filename,"r");
    if(file == NULL)
    {
	err_log_stderr("fopen()");
	return ERROR_IO;
    }
    while((c = fgetc(file)) != EOF)
	if(c == '\n')
	    (*lines)++;
    fclose(file);

    if(c != '\n')
	(*lines)++;
    return ERROR_OK;
}

path_planner_t *pp_init(const char *filename, double w_hover)
{
    int
	i,
	j,
	p_type,
	retval = ERROR_OK;
    FILE *file;
    path_planner_t *pp = (path_planner_t *)malloc(sizeof(path_planner_t));
    mem_alloc_check(pp);
    if(filename==NULL)
    {
	err_log("No path supplied, will hover in place.");
    }
    else
    {
	retval = line_count(filename, &pp->sp_list_len);
	cleanup_if(retval);
    }
    pp->sp_list_len++; // For default setpoint (hovering)

    pp->sp_list = (set_point_t **)malloc(pp->sp_list_len*sizeof(set_point_t*));
    mem_alloc_check(pp->sp_list);
    for(i=0; i<pp->sp_list_len; ++i)
	pp->sp_list[i] = NULL;

    /**
     * First setpoint is for takeoff:
     *   - hover at HOVER_HEIGHT
     *   - face north
     * Default matrix is designed for this scenario
     */
    pp->sp_list[0] = setpoint_init();
    if(pp->sp_list[0] == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate setpoint!");
    }
    pp->sp_list[0]->x->m_full[SV_Z] = HOVER_HEIGHT;
    /// Set w hover
    for(j=0; j<LENGTH_INPUT; ++j)
	pp->sp_list[0]->w->m_full[j] = w_hover;
    pp->sp = pp->sp_list[0];
    pp->sp_list_curr = 0;

    if(filename != NULL)
    {
	file = fopen(filename,"r");
	if(file == NULL)
	{
	    err_log_stderr("fopen()");
	    goto cleanup;
	}

	for(i=1; i < pp->sp_list_len; ++i)
	{
	    /// Allocate memory for setpoint
	    pp->sp_list[i] = setpoint_init();
	    if(pp->sp_list[i] == NULL)
	    {
		cleanup_log_if(ERROR_MALLOC,"Failed to allocate setpoint!");
	    }

	    /// Load setpoint from file
	    retval = uquad_mat_load(pp->sp_list[i]->x, file);
	    cleanup_if(retval);

	    /// Load path type
	    retval = fscanf(file,"%d",&p_type);
	    if(retval <= 0)
	    {
		err_log_stderr("fscanf()");
		cleanup_if(ERROR_READ);
	    }
	    if(p_type < 0 || p_type >= PATH_TYPE_COUNT)
	    {
		err_log_num("Invalid path type!", p_type);
		cleanup_if(ERROR_FAIL);
	    }
	    pp->sp_list[i]->pt = (path_type_t) p_type;

	    /// Load w setpoint relative to w_hover
	    retval = uquad_mat_load(pp->sp_list[i]->w, file);
	    cleanup_if(retval);
	    /// Add w hover
	    for(j=0; j<LENGTH_INPUT; ++j)
		pp->sp_list[i]->w->m_full[j] += w_hover;

	}
	fclose(file);
	file = NULL;
    }

    return pp;
    cleanup:
    pp_deinit(pp);
    if(file != NULL)
	fclose(file);
    return NULL;
}

uquad_bool_t pp_setpoint_reached(uquad_mat_t *x, set_point_t *sp_curr)
{
    static int in_range = 0;
    if(
       (uquad_abs(x->m_full[SV_X] - sp_curr->x->m_full[SV_X]) < PP_REACHED_X) &&
       (uquad_abs(x->m_full[SV_Y] - sp_curr->x->m_full[SV_Y]) < PP_REACHED_Y) &&
       (uquad_abs(x->m_full[SV_Z] - sp_curr->x->m_full[SV_Z]) < PP_REACHED_Z)
       )
    {
	/// In range, check if we've spent enough samples near the current setpoint
	if(++in_range >= PP_REACHED_COUNT)
	{
	    in_range = 0;
	    return true;
	}
    }
    else
    {
	/// Out of range, decrease counter.
	in_range = uquad_max(in_range -1, 0);
    }
    return false;
}

set_point_t *pp_get_next_sp(path_planner_t *pp)
{
    return
	(pp->sp_list_curr < pp->sp_list_len - 1)?
	pp->sp_list[pp->sp_list_curr+1]:
	pp->sp_list[0];
}

void pp_update_setpoint(path_planner_t *pp)
{
    set_point_t *sp_next = pp_get_next_sp(pp);
    if(sp_next != pp->sp)
    {
	pp->sp_list_curr++;
	pp->sp = sp_next;
    }
    return;
}

void pp_check_progress(path_planner_t *pp, uquad_mat_t *x, uquad_bool_t *arrived)
{
    if(pp == NULL || x == NULL || arrived == NULL)
    {
	err_log("Invalid argument.");
	return;
    }
    *arrived = false;
    if(pp_setpoint_reached(x, pp->sp) &&
       (pp->sp != pp_get_next_sp(pp)))
	*arrived = true;
    if((pp->sp_list_curr == pp->sp_list_len - 1) && *arrived)
    {
	err_log("Route completed, next setpoint will be hovering.");
    }
    return;
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
    int i;
    if (pp == NULL)
	return;
    if(pp->sp_list != NULL)
    {
	for(i=0; i<pp->sp_list_len; ++i)
	    setpoint_deinit(pp->sp_list[i]);
	free(pp->sp_list);
    }
    free(pp);
}
