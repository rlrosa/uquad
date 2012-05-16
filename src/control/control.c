#include "control.h"
#include <stdlib.h>
#include <uquad_error_codes.h>

uquad_mat_t *tmp_sub_sp_x;
uquad_mat_t *tmp_x_hat_partial;
uquad_mat_t *w_tmp;
uquad_mat_t *x_int_tmp;

#if CTRL_INTEGRAL
int control_clear_int(ctrl_t * ctrl)
{
    int retval;
    retval = uquad_mat_zeros(ctrl->x_int);
    err_propagate(retval);
    return retval;
}
#endif

ctrl_t *control_init(void)
{
    int retval;
    FILE *file_mat = NULL;
    ctrl_t *ctrl = (ctrl_t *)malloc(sizeof(ctrl_t));
    mem_alloc_check(ctrl);

    ctrl->K = uquad_mat_alloc(LENGTH_INPUT,STATES_CONTROLLED);
    tmp_sub_sp_x = uquad_mat_alloc(STATE_COUNT,1);
    w_tmp = uquad_mat_alloc(LENGTH_INPUT,1);
    if(ctrl->K == NULL || tmp_sub_sp_x == NULL || w_tmp == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate aux mem!");
    }
#if !FULL_CONTROL
    tmp_x_hat_partial = uquad_mat_alloc(STATES_CONTROLLED,1);
    if(tmp_x_hat_partial == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate aux mem!");
    }
#endif

    file_mat = fopen(CTRL_MAT_K_NAME,"r");
    if(file_mat == NULL)
    {
	err_log_str("Failed to open:",CTRL_MAT_K_NAME);
	cleanup_if(ERROR_FAIL);
    }
    else
    {
	err_log_str("Loaded proportional gain matrix from:",
		    CTRL_MAT_K_NAME);
    }

    retval = uquad_mat_load(ctrl->K, file_mat);
    if(retval != ERROR_OK)
    {
	err_log_str("Failed to load gain matrix. Check size...",CTRL_MAT_K_NAME);
	cleanup_if(retval);
    }
    fclose(file_mat);
    file_mat = NULL;

#if CTRL_INTEGRAL
#if CTRL_INTEGRAL_ANG
    int i;
    for(i = 0; i < LENGTH_INPUT; ++i)
    {
	ctrl->K->m[i][SV_X]   = 0;
	ctrl->K->m[i][SV_Y]   = 0;
	ctrl->K->m[i][SV_VQX] = 0;
	ctrl->K->m[i][SV_VQY] = 0;
    }
#endif // CTRL_INTEGRAL_ANG
    ctrl->K_int = uquad_mat_alloc(LENGTH_INPUT,STATES_INT_CONTROLLED);
    ctrl->x_int = uquad_mat_alloc(STATES_INT_CONTROLLED,1);
    x_int_tmp   = uquad_mat_alloc(STATES_INT_CONTROLLED,1);
    if(ctrl->x_int == NULL || ctrl->K_int == NULL || x_int_tmp == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate aux mem!");
    }
    retval = control_clear_int(ctrl);
    cleanup_log_if(retval, "Failed to clear integral term!");

    file_mat = fopen(CTRL_MAT_K_INT_NAME,"r");
    if(file_mat == NULL)
    {
	err_log_str("Failed to open:",CTRL_MAT_K_INT_NAME);
	cleanup_if(ERROR_FAIL);
    }
    else
    {
	err_log_str("Loaded integral gain matrix from:",
		    CTRL_MAT_K_INT_NAME);
    }

    retval = uquad_mat_load(ctrl->K_int, file_mat);
    if(retval != ERROR_OK)
    {
	err_log_str("Failed to load gain matrix. Check size...",CTRL_MAT_K_INT_NAME);
	cleanup_if(retval);
    }
#endif // CTRL_INTEGRAL
    if(file_mat != NULL)
	fclose(file_mat);

    return ctrl;

    cleanup:
    if(file_mat != NULL)
	fclose(file_mat);
    control_deinit(ctrl);
    return NULL;
}

/**
 * Update integral estimation of x_int, controlling step size and max value of
 * integration.
 *
 * @param x_int Current integral value.
 * @param data New data.
 * @param T_s Sampling period for data.
 * @param delta_max Limit on step size.
 * @param accum_max Limit on integration.
 */
static inline void ctrl_int(double *x_int, double data, double T_s, double delta_max, double accum_max)
{
    double
	ddelta;
    ddelta = uquad_min(data * T_s, delta_max);
    if(ddelta < 0)
	ddelta = uquad_max(ddelta, -delta_max);
    *x_int = uquad_min(ddelta + (*x_int), accum_max);
    if(*x_int < 0)
	*x_int = uquad_max(*x_int,-accum_max);
}

int control(ctrl_t *ctrl, uquad_mat_t *w, uquad_mat_t *x, set_point_t *sp, double T_us)
{
    int retval = ERROR_OK;
    if(ctrl == NULL || w == NULL || x == NULL || sp == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Inputs must be non NULL!");
    }

    retval = uquad_mat_get_subm(tmp_sub_sp_x,0,0,x);
    err_propagate(retval);
    retval = uquad_mat_sub(tmp_sub_sp_x, sp->x, tmp_sub_sp_x);
    err_propagate(retval);

#if !FULL_CONTROL
    /// only using part of the state vector
    tmp_x_hat_partial->m_full[0] = tmp_sub_sp_x->m_full[2];  // z
    tmp_x_hat_partial->m_full[1] = tmp_sub_sp_x->m_full[3];  // psi
    tmp_x_hat_partial->m_full[2] = tmp_sub_sp_x->m_full[4];  // phi
    tmp_x_hat_partial->m_full[3] = tmp_sub_sp_x->m_full[5];  // theta
    tmp_x_hat_partial->m_full[4] = tmp_sub_sp_x->m_full[8];  // vqz
    tmp_x_hat_partial->m_full[5] = tmp_sub_sp_x->m_full[9];  // wqx
    tmp_x_hat_partial->m_full[6] = tmp_sub_sp_x->m_full[10]; // wqy
    tmp_x_hat_partial->m_full[7] = tmp_sub_sp_x->m_full[11]; // wqz

    retval = uquad_mat_prod(w, ctrl->K, tmp_x_hat_partial);
    err_propagate(retval);
#else
    // Control all states
    retval = uquad_mat_prod(w, ctrl->K, tmp_sub_sp_x);
    err_propagate(retval);
#endif // FULL_CONTROL

    retval = uquad_mat_add(w,sp->w,w);
    err_propagate(retval);
#if CTRL_INTEGRAL
    double
	T_s = T_us/1000000.0;

    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 1),
    	     tmp_sub_sp_x->m[SV_THETA][0],
    	     T_s,
    	     CTRL_INT_DELTA_MAX_THETA,
    	     CTRL_INT_ACCUM_MAX_THETA);

    /// SV_Z
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 2),
	     tmp_sub_sp_x->m[SV_Z][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_Z,
	     CTRL_INT_ACCUM_MAX_Z);

#if FULL_CONTROL
#if CTRL_INTEGRAL_ANG
    /// SV_PHI
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 3),
	     tmp_sub_sp_x->m[SV_PHI][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_PHI,
	     CTRL_INT_ACCUM_MAX_PHI);

    /// SV_PSI
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 4),
	     tmp_sub_sp_x->m[SV_PSI][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_PSI,
	     CTRL_INT_ACCUM_MAX_PSI);
#else // CTRL_INTEGRAL_ANG
    /// SV_Y
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 3),
	     tmp_sub_sp_x->m[SV_Y][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_Y,
	     CTRL_INT_ACCUM_MAX_Y);

    /// SV_X
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 4),
	     tmp_sub_sp_x->m[SV_X][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_X,
	     CTRL_INT_ACCUM_MAX_X);
#endif // CTRL_INTEGRAL_ANG
#endif // FULL_CONTROL
    retval = uquad_mat_prod(w_tmp, ctrl->K_int, ctrl->x_int);
    err_propagate(retval);
    retval = uquad_mat_add(w, w, w_tmp);
    err_propagate(retval);
#else // CTRL_INTEGRAL
    T_us = T_us; // just to avoid warning
#endif

    return ERROR_OK;
}

void control_deinit(ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
	err_log("No memory had been allocated for ctrl");
	return;
    }
    uquad_mat_free(ctrl->K);
    uquad_mat_free(tmp_sub_sp_x);
    uquad_mat_free(w_tmp);
#if !FULL_CONTROL
    uquad_mat_free(tmp_x_hat_partial);
#endif // !FULL_CONTROL
#if CTRL_INTEGRAL
    uquad_mat_free(ctrl->K_int);
    uquad_mat_free(ctrl->x_int);
    uquad_mat_free(x_int_tmp);
#endif
    free(ctrl);
}

int control_dump(ctrl_t *ctrl, FILE *output)
{
    if(ctrl == NULL)
    {
	err_check(ERROR_NULL_POINTER, "NULL pointer is invalid arg!");
    }
    log_msg(output,"Control - K");
    uquad_mat_dump(ctrl->K, output);
#if CTRL_INTEGRAL
    log_msg(output,"Control - K_int");
    uquad_mat_dump(ctrl->K_int, output);
#endif
    return ERROR_OK;
}
