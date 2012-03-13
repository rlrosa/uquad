#include "path_planner.h"
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
    sp->x = uquad_mat_alloc(12,1);
    mem_alloc_check(sp->x);
    sp->w = uquad_mat_alloc(4,1);
    mem_alloc_check(sp->w);
    retval = uquad_mat_zeros(sp->x);
    if(retval != ERROR_OK)
    {
	err_log("setpoint_init() failed!");
	return NULL;
    }
    /// HOVER at 1m
    sp->x->m_full[0] = 1; 
    sp->w->m_full[0] = W_HOVER;
    sp->w->m_full[1] = W_HOVER;
    sp->w->m_full[2] = W_HOVER;
    sp->w->m_full[3] = W_HOVER;
    return sp;
}

path_planner_t *pp_init(void)
{
    int retval = ERROR_OK;
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


int pp_update_setpoint(path_planner_t *pp, uquad_mat_t *x)
{
    if (pp->pt != HOVER)
    {
	err_check(ERROR_FAIL, "Not implemented!");
    }
    return ERROR_OK;
}

void pp_deinit(path_planner_t *pp)
{
    if (pp == NULL)
	return;
    setpoint_deinit(pp->sp);
    free(pp);
}
