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
