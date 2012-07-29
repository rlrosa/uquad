/**
 * control: lib for determing control actions.
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
 * @file   control.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for determing control actions.
 *
 */
#include "control.h"
#include <uquad_error_codes.h>
#include <math.h>
#include <stdlib.h>
#include <uquad_aux_time.h>

uquad_mat_t *tmp_sub_sp_x;
uquad_mat_t *tmp_x_hat_partial;
uquad_mat_t *w_tmp;
uquad_mat_t *x_int_tmp;

// Aux matrices used for updating K - control_update_k()
uquad_mat_t *update_k_A = NULL;
uquad_mat_t *update_k_B = NULL;
uquad_mat_t *update_k_Aext = NULL;
uquad_mat_t *update_k_Bext = NULL;
uquad_mat_t *update_k_phi = NULL;
uquad_mat_t *update_k_gamma = NULL;

// Aux matrices used for discretization - control_disc()
uquad_mat_t *disc_aux0 = NULL;
uquad_mat_t *disc_aux1 = NULL;

// Aux matrices used for LQR iterations - control_lqr()
uquad_mat_t *lqr_aux0 = NULL;
uquad_mat_t *lqr_aux1 = NULL;
uquad_mat_t *lqr_aux2 = NULL;
uquad_mat_t *lqr_aux3 = NULL;
uquad_mat_t *lqr_aux4 = NULL;
uquad_mat_t *lqr_aux5 = NULL;
uquad_mat_t *lqr_aux6 = NULL;
uquad_mat_t *lqr_aux7 = NULL;
uquad_mat_t *lqr_aux8 = NULL;
uquad_mat_t *lqr_aux9 = NULL;
uquad_mat_t *lqr_aux_10 = NULL;
uquad_mat_t *lqr_aux_11 = NULL;
uquad_mat_t *lqr_aux_12 = NULL;
uquad_mat_t *lqr_aux_13 = NULL;
uquad_mat_t *lqr_P = NULL;
uquad_mat_t *lqr_k = NULL;
uquad_mat_t *lqr_Bt = NULL;

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

    /// Mem for lqr()
    ctrl->K      = uquad_mat_alloc(LENGTH_INPUT,STATES_CONTROLLED);
    ctrl->K_lqr  = uquad_mat_alloc(LENGTH_INPUT,
				   STATES_CONTROLLED + STATES_INT_CONTROLLED);
    ctrl->A      = uquad_mat_alloc(STATES_CONTROLLED,STATES_CONTROLLED);
    ctrl->B      = uquad_mat_alloc(STATES_CONTROLLED,LENGTH_INPUT);
    ctrl->Q      = uquad_mat_alloc(STATES_CONTROLLED + STATES_INT_CONTROLLED,
			      STATES_CONTROLLED + STATES_INT_CONTROLLED);
    ctrl->R      = uquad_mat_alloc(LENGTH_INPUT, LENGTH_INPUT);
    tmp_sub_sp_x = uquad_mat_alloc(STATE_COUNT,1);
    w_tmp        = uquad_mat_alloc(LENGTH_INPUT,1);

    if(ctrl->K      == NULL ||
       ctrl->K_lqr  == NULL ||
       ctrl->A      == NULL ||
       ctrl->B      == NULL ||
       ctrl->Q      == NULL ||
       ctrl->R      == NULL ||
       tmp_sub_sp_x == NULL ||
       w_tmp        == NULL ||
       ctrl->K_lqr  == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate aux mem!");
    }

    // Aux matrices for control_update_k()
    update_k_A = uquad_mat_alloc(STATES_CONTROLLED,STATES_CONTROLLED);
    update_k_B = uquad_mat_alloc(STATES_CONTROLLED,LENGTH_INPUT);
    update_k_Aext = uquad_mat_alloc(STATES_CONTROLLED + STATES_INT_CONTROLLED,
				    STATES_CONTROLLED + STATES_INT_CONTROLLED);
    update_k_Bext = uquad_mat_alloc(STATES_CONTROLLED + STATES_INT_CONTROLLED,
				    LENGTH_INPUT);
    update_k_phi = uquad_mat_alloc(update_k_Aext->r,update_k_Aext->c);
    update_k_gamma = uquad_mat_alloc(update_k_Bext->r,update_k_Bext->c);

    // Aux matrices for discretization - control_disc()
    disc_aux0 = uquad_mat_alloc(update_k_Aext->r+update_k_Bext->c,update_k_Aext->c+update_k_Bext->c);
    disc_aux1 = uquad_mat_alloc(update_k_Aext->r+update_k_Bext->c,update_k_Aext->c+update_k_Bext->c);

    // Aux matrices used for LQR iterations - control_lqr()
    lqr_P = uquad_mat_alloc(ctrl->Q->r,ctrl->Q->c);
    lqr_k = uquad_mat_alloc(update_k_gamma->c,update_k_gamma->r);
    lqr_Bt = uquad_mat_alloc(update_k_gamma->c,update_k_gamma->r);
    //For K=-(R+B'*P*B)^(-1)*B'*P*A
    lqr_aux0 = uquad_mat_alloc(lqr_P->r,update_k_gamma->c);
    lqr_aux1 = uquad_mat_alloc(ctrl->R->r,ctrl->R->c);
    lqr_aux2 = uquad_mat_alloc(ctrl->R->r,ctrl->R->c);
    lqr_aux3 = uquad_mat_alloc(ctrl->R->r,ctrl->R->c);
    lqr_aux4 = uquad_mat_alloc(ctrl->R->r,2*ctrl->R->c);
    lqr_aux5 = uquad_mat_alloc(ctrl->R->r,update_k_gamma->r);
    lqr_aux6 = uquad_mat_alloc(ctrl->R->r,lqr_P->c);
    //For P=Q+K'*R*K+(A+B*K)'*P*(A+B*K)
    lqr_aux7 = uquad_mat_alloc(ctrl->K_lqr->c,ctrl->K_lqr->r);
    lqr_aux8 = uquad_mat_alloc(ctrl->K_lqr->c,ctrl->R->c);
    lqr_aux9 = uquad_mat_alloc(ctrl->K_lqr->c,ctrl->K_lqr->c);
    //For A+B*K
    lqr_aux_10 = uquad_mat_alloc(update_k_gamma->r,ctrl->K_lqr->c);
    lqr_aux_11 = uquad_mat_alloc(ctrl->K_lqr->c,update_k_gamma->r);
    lqr_aux_12 = uquad_mat_alloc(ctrl->K_lqr->c,lqr_P->c);
    lqr_aux_13 = uquad_mat_alloc(ctrl->K_lqr->r,ctrl->K_lqr->c);

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

    retval = uquad_mat_load(ctrl->K, file_mat);
    if(retval != ERROR_OK)
    {
	err_log_str("Failed to load gain matrix. Check size...",CTRL_MAT_K_NAME);
	cleanup_if(retval);
    }
    else
    {
	err_log_str("Loaded proportional gain matrix from:",
		    CTRL_MAT_K_NAME);
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

#if CTRL_INTEGRAL_ANG
    /// Avoid unstable startup caused by motor differences
    ctrl->x_int->m_full[ctrl->x_int->r - 4] = 0.0; // PSI
    ctrl->x_int->m_full[ctrl->x_int->r - 3] = 0.0; // PHI
    ctrl->x_int->m_full[ctrl->x_int->r - 2] = 0.0; // Z
    ctrl->x_int->m_full[ctrl->x_int->r - 1] = 0.0; // THETA
#endif // CTRL_INTEGRAL_ANG

    file_mat = fopen(CTRL_MAT_K_INT_NAME,"r");
    if(file_mat == NULL)
    {
	err_log_str("Failed to open:",CTRL_MAT_K_INT_NAME);
	cleanup_if(ERROR_FAIL);
    }

    retval = uquad_mat_load(ctrl->K_int, file_mat);
    if(retval != ERROR_OK)
    {
	err_log_str("Failed to load gain matrix. Check size...",CTRL_MAT_K_INT_NAME);
	cleanup_if(retval);
    }
    else
    {
	err_log_str("Loaded integral gain matrix from:",
		    CTRL_MAT_K_INT_NAME);
    }
#endif // CTRL_INTEGRAL
    if(file_mat != NULL)
	fclose(file_mat);

    file_mat = fopen(CTRL_MAT_Q_NAME,"r");
    if(file_mat == NULL)
    {
	err_log_str("Failed to open:",CTRL_MAT_Q_NAME);
	cleanup_if(ERROR_FAIL);
    }
    retval = uquad_mat_load(ctrl->Q, file_mat);
    if(retval != ERROR_OK)
    {
	err_log_str("Failed to load LQR Q matrix. Check size...",CTRL_MAT_Q_NAME);
	cleanup_if(retval);
    }
    else
    {
	err_log_str("Loaded LQR Q matrix from:",
		    CTRL_MAT_Q_NAME);
    }
    fclose(file_mat);

    file_mat = fopen(CTRL_MAT_R_NAME,"r");
    if(file_mat == NULL)
    {
	err_log_str("Failed to open:",CTRL_MAT_R_NAME);
	cleanup_if(ERROR_FAIL);
    }
    retval = uquad_mat_load(ctrl->R, file_mat);
    if(retval != ERROR_OK)
    {
	err_log_str("Failed to load LQR R matrix. Check size...",CTRL_MAT_R_NAME);
	cleanup_if(retval);
    }
    else
    {
	err_log_str("Loaded LQR R matrix from:",
		    CTRL_MAT_R_NAME);
    }
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
 * @param threshold If further that this from threshold, let P control take care
 */
static inline void ctrl_int(double *x_int, double data, double T_s,
			    double delta_max, double accum_max, double threshold)
{
    double
	ddelta;
    if(uquad_abs(data) > threshold)
    {
	/// Too far from setopint
	return;
    }
    ddelta = uquad_min(data * T_s, delta_max);
    if(ddelta < 0)
	ddelta = uquad_max(ddelta, -delta_max);
    *x_int = uquad_min(ddelta + (*x_int), accum_max);
    if(*x_int < 0)
	*x_int = uquad_max(*x_int,-accum_max);
}

/**
 * Limit error acceptable to avoid violent behavior.
 * Examples:
 *   - Initial orientation (yaw) too far from desired.
 *   - Bad GPS signal reading tens of meters from current position.
 *
 * @param x error vector (sp_x - x_est)
 */
static inline void control_limit(uquad_mat_t *x)
{
    x->m_full[SV_X]     = uquad_limit(x->m_full[SV_X], CTRL_LIMIT_X);
    x->m_full[SV_Y]     = uquad_limit(x->m_full[SV_Y], CTRL_LIMIT_Y);
    x->m_full[SV_Z]     = uquad_limit(x->m_full[SV_Z], CTRL_LIMIT_Z);
    x->m_full[SV_THETA] = uquad_limit(x->m_full[SV_THETA], CTRL_LIMIT_THETA);
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

    control_limit(tmp_sub_sp_x);

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
	     CTRL_INT_ACCUM_MAX_THETA,
	     CTRL_INT_TH_THETA);

    /// SV_Z
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 2),
	     tmp_sub_sp_x->m[SV_Z][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_Z,
	     CTRL_INT_ACCUM_MAX_Z,
	     CTRL_INT_TH_Z);

#if FULL_CONTROL
#if CTRL_INTEGRAL_ANG
    /// SV_PHI
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 3),
	     tmp_sub_sp_x->m[SV_PHI][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_PHI,
	     CTRL_INT_ACCUM_MAX_PHI,
	     CTRL_INT_TH_PHI);

    /// SV_PSI
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 4),
	     tmp_sub_sp_x->m[SV_PSI][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_PSI,
	     CTRL_INT_ACCUM_MAX_PSI,
	     CTRL_INT_TH_PSI);

#else // CTRL_INTEGRAL_ANG
    /// SV_Y
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 3),
	     tmp_sub_sp_x->m[SV_Y][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_Y,
	     CTRL_INT_ACCUM_MAX_Y,
	     CTRL_INT_TH_Y);

    /// SV_X
    ctrl_int(ctrl->x_int->m_full + (ctrl->x_int->r - 4),
	     tmp_sub_sp_x->m[SV_X][0],
	     T_s,
	     CTRL_INT_DELTA_MAX_X,
	     CTRL_INT_ACCUM_MAX_X,
	     CTRL_INT_TH_X);

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

    uquad_mat_free(ctrl->A);
    uquad_mat_free(ctrl->B);
    uquad_mat_free(ctrl->Q);
    uquad_mat_free(ctrl->R);

    // Aux matrices used for updating K - control_update_k()
    uquad_mat_free(update_k_A);
    uquad_mat_free(update_k_B);
    uquad_mat_free(update_k_Aext);
    uquad_mat_free(update_k_Bext);
    uquad_mat_free(update_k_phi);
    uquad_mat_free(update_k_gamma);

    // Aux matrices used for dicretization - control_disc()
    uquad_mat_free(disc_aux0);
    uquad_mat_free(disc_aux1);

    // Aux matrices used for LQR iterations - control_lqr()
    uquad_mat_free(lqr_aux0);
    uquad_mat_free(lqr_aux1);
    uquad_mat_free(lqr_aux2);
    uquad_mat_free(lqr_aux3);
    uquad_mat_free(lqr_aux4);
    uquad_mat_free(lqr_aux5);
    uquad_mat_free(lqr_aux6);
    uquad_mat_free(lqr_aux7);
    uquad_mat_free(lqr_aux8);
    uquad_mat_free(lqr_aux9);
    uquad_mat_free(lqr_aux_10);
    uquad_mat_free(lqr_aux_11);
    uquad_mat_free(lqr_aux_12);
    uquad_mat_free(lqr_aux_13);
    uquad_mat_free(lqr_k);
    uquad_mat_free(lqr_P);
    uquad_mat_free(lqr_Bt);

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
    if(output == NULL)
	output = stdout;
    log_msg(output,"Control - K");
    uquad_mat_dump(ctrl->K, output);
#if CTRL_INTEGRAL
    log_msg(output,"Control - K_int");
    uquad_mat_dump(ctrl->K_int, output);
#endif
    return ERROR_OK;
}

int control_update_K(ctrl_t *ctrl, set_point_t *sp, double weight, uquad_bool_t *update_complete, uquad_bool_t *start)
{
    int
	i,
	retval;
    /* uquad_mat_t *update_k_A = NULL; */
    /* uquad_mat_t *update_k_B = NULL; */
    /* uquad_mat_t *update_k_Aext = NULL; */
    /* uquad_mat_t *update_k_Bext = NULL; */
    /* uquad_mat_t *update_k_phi = NULL; */
    /* uquad_mat_t *update_k_gamma = NULL; */
    if(start == NULL)
    {
	/// Calculation of K in progress, continue.
	retval = control_lqr(ctrl->K_lqr, update_k_phi, update_k_gamma, ctrl->Q, ctrl->R,update_complete, NULL);
	cleanup_if(retval);
	if(*update_complete)
	{
	    // use new matrix!
	    // proportional term
	    retval = uquad_mat_get_subm(ctrl->K,0,0,ctrl->K_lqr);
	    cleanup_if(retval);
#if CTRL_INTEGRAL
	    // intergral term
	    retval = uquad_mat_get_subm(ctrl->K_int,0,STATE_COUNT,ctrl->K_lqr);
	    cleanup_if(retval);
#endif // CTRL_INTEGRAL

	    // ignore VQX, VQY
	    for(i=0; i < LENGTH_INPUT; ++i)
	    {
		ctrl->K->m[i][SV_VQX] = 0.0;
		ctrl->K->m[i][SV_VQY] = 0.0;
	    }
	}
	return retval;
    }

    //Linearization of the system
    retval = control_lin_model(update_k_A,update_k_B,sp->pt,sp, weight);
    cleanup_if(retval);

    //Extends the system to include the integrated states

    retval = uquad_mat_zeros(update_k_Aext);
    cleanup_if(retval);
    retval = uquad_mat_zeros(update_k_Bext);
    cleanup_if(retval);
    retval = uquad_mat_set_subm(update_k_Aext,0,0,update_k_A);
    cleanup_if(retval);

#if CTRL_INTEGRAL_ANG
    update_k_Aext->m
	[STATES_CONTROLLED]
	[SV_PSI]
	= 1.0;
    update_k_Aext->m
	[STATES_CONTROLLED + 1]
	[SV_PHI]
	= 1.0;
#else // CTRL_INTEGRAL_ANG
    update_k_Aext->m
	[STATES_CONTROLLED]
	[SV_X]
	= 1.0;
    update_k_Aext->m
	[STATES_CONTROLLED + 1]
	[SV_Y]
	= 1.0;
#endif // CTRL_INTEGRAL_ANG
    update_k_Aext->m
	[STATES_CONTROLLED + 2]
	[SV_Z]
	= 1.0;
    update_k_Aext->m
	[STATES_CONTROLLED + 3]
	[SV_THETA]
	= 1.0;

    retval = uquad_mat_set_subm(update_k_Bext,0,0,update_k_B);
    cleanup_if(retval);
    //Discretization of the system
    /* phi = uquad_mat_alloc(update_k_Aext->r,update_k_Aext->c); */
    /* gamma = uquad_mat_alloc(update_k_Bext->r,update_k_Bext->c); */
    retval = control_disc(update_k_phi,
			  update_k_gamma,
			  update_k_Aext,
			  update_k_Bext,
			  TS_DEFAULT_US_DBL/1e6);
    cleanup_if(retval);
    //Obtains feedback matrix
    retval = control_lqr(ctrl->K_lqr,
			 update_k_phi,
			 update_k_gamma,
			 ctrl->Q,
			 ctrl->R,
			 update_complete,start);
    cleanup_if(retval);

    cleanup:
    /* uquad_mat_free(update_k_A); */
    /* uquad_mat_free(update_k_B); */
    /* uquad_mat_free(update_k_Aext); */
    /* uquad_mat_free(update_k_Bext); */
    /* uquad_mat_free(update_k_phi); */
    /* uquad_mat_free(update_k_gamma); */

    return retval;
}

int control_disc(uquad_mat_t *phi,uquad_mat_t *gamma,uquad_mat_t *A,uquad_mat_t *B, double Ts)
{
    int retval;
    /* uquad_mat_t *disc_aux0 = NULL; */
    /* uquad_mat_t *disc_aux1 = NULL; */
    
    /* disc_aux0 = uquad_mat_alloc(A->r+B->c,A->c+B->c); */
    /* disc_aux1 = uquad_mat_alloc(A->r+B->c,A->c+B->c); */

    retval = uquad_mat_set_subm(disc_aux0, 0,0,A);
    cleanup_if(retval);
    retval =  uquad_mat_set_subm(disc_aux0, 0,A->c,B);
    cleanup_if(retval);
    retval = uquad_mat_scalar_mul(disc_aux0,disc_aux0,Ts);
    cleanup_if(retval);
    retval = uquad_mat_exp(disc_aux1,disc_aux0);
    cleanup_if(retval);
    retval= uquad_mat_get_subm(phi,0,0,disc_aux1);
    cleanup_if(retval);
    retval = uquad_mat_get_subm(gamma,0,A->c,disc_aux1);
    cleanup_if(retval);
    //Cleaning
    cleanup:
    /* uquad_mat_free(disc_aux0); */
    /* uquad_mat_free(disc_aux1); */

    return retval;
}

int control_lqr(uquad_mat_t *K, uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *Q, uquad_mat_t *R,
		uquad_bool_t *finished, uquad_bool_t *start)
{
    int
	err_time,
	retval;
    double norm;
    /* uquad_mat_t *lqr_aux0 = NULL; */
    /* uquad_mat_t *lqr_aux1 = NULL; */
    /* uquad_mat_t *lqr_aux2 = NULL; */
    /* uquad_mat_t *lqr_aux3 = NULL; */
    /* uquad_mat_t *lqr_aux4 = NULL; */
    /* uquad_mat_t *lqr_aux5 = NULL; */
    /* uquad_mat_t *lqr_aux6 = NULL; */
    /* uquad_mat_t *lqr_aux7 = NULL; */
    /* uquad_mat_t *lqr_aux8 = NULL; */
    /* uquad_mat_t *lqr_aux9 = NULL; */
    /* uquad_mat_t *lqr_aux_10 = NULL; */
    /* uquad_mat_t *lqr_aux_11 = NULL; */
    /* uquad_mat_t *lqr_aux_12 = NULL; */
    /* uquad_mat_t *lqr_aux_13 = NULL; */
    /* uquad_mat_t *lqr_P = NULL; */
    /* uquad_mat_t *lqr_k = NULL; */
    /* uquad_mat_t *lqr_Bt = NULL; */

    struct timeval
	tv_in,
	tv_tmp,
	tv_diff,
	tv_out;
    tv_out.tv_sec = 0;
    tv_out.tv_usec = 1500;

    norm=1;

    retval = gettimeofday(&tv_in,NULL);
    if(retval < 0)
    {
	err_log_stderr("gettimeofday()");
	retval = ERROR_TIMING;
	cleanup_if(retval);
    }

    *finished = false;
    if(start != NULL)
    {
	/**
	 * This is a fresh start, inicialize everything and
	 * set a lower timeout to compensate for the previous
	 * calculations (linearization+discretization)
	 */
	tv_out.tv_usec = 500;

	/* lqr_P = uquad_mat_alloc(Q->r,Q->c); */
	/* lqr_k = uquad_mat_alloc(B->c,B->r); */

	retval = uquad_mat_fill(K,1.0);
	cleanup_if(retval);

	retval = uquad_mat_copy(lqr_P,Q);
	cleanup_if(retval);
	retval = uquad_mat_copy(lqr_k,K);
	cleanup_if(retval);
	/* lqr_Bt = uquad_mat_alloc(B->c,B->r); */

	retval = uquad_mat_transpose(lqr_Bt,B);
	cleanup_if(retval);

	//For K=-(R+B'*P*B)^(-1)*B'*P*A
	/* lqr_aux0 = uquad_mat_alloc(P->r,B->c); */
	/* lqr_aux1 = uquad_mat_alloc(R->r,R->c); */
	/* lqr_aux2 = uquad_mat_alloc(R->r,R->c); */
	/* lqr_aux3 = uquad_mat_alloc(R->r,R->c); */
	/* lqr_aux4 = uquad_mat_alloc(R->r,2*R->c); */
	/* lqr_aux5 = uquad_mat_alloc(R->r,B->r); */
	/* lqr_aux6 = uquad_mat_alloc(R->r,P->c); */

	//Performs P=Q+K'*R*K+(A+B*K)'*P*(A+B*K)
	//K'*R*K
	/* lqr_aux7 = uquad_mat_alloc(K->c,K->r); */
	/* lqr_aux8 = uquad_mat_alloc(K->c,R->c); */
	/* lqr_aux9 = uquad_mat_alloc(K->c,K->c); */
	//A+B*K
	/* lqr_aux_10 = uquad_mat_alloc(B->r,K->c); */
	/* lqr_aux_11 = uquad_mat_alloc(K->c,B->r); */
	/* lqr_aux_12 = uquad_mat_alloc(K->c,P->c); */
	/* lqr_aux_13 = uquad_mat_alloc(K->r,K->c); */
    }

    while (norm>CTRL_LQR_TH)
    {
	retval = uquad_mat_copy(lqr_k,K);
	cleanup_if(retval);
	//Performs K=-(R+B'*P*B)^(-1)*B'*P*A; 
	retval = uquad_mat_prod(lqr_aux0,lqr_P,B);
	cleanup_if(retval);
	retval = uquad_mat_prod(lqr_aux1,lqr_Bt,lqr_aux0);
	cleanup_if(retval);
	retval = uquad_mat_add(lqr_aux1,lqr_aux1,R);
	cleanup_if(retval);
	retval = uquad_mat_inv(lqr_aux2,lqr_aux1,lqr_aux3,lqr_aux4);
	cleanup_if(retval);
	
	retval = uquad_mat_prod(lqr_aux5,lqr_aux2,lqr_Bt);
	cleanup_if(retval);
	retval = uquad_mat_prod(lqr_aux6,lqr_aux5,lqr_P);
	cleanup_if(retval);
	retval = uquad_mat_prod(lqr_aux5,lqr_aux6,A);
	cleanup_if(retval);
	retval = uquad_mat_scalar_mul(K,lqr_aux5,-1);
	cleanup_if(retval);

	//Performs P=Q+K'*R*K+(A+B*K)'*P*(A+B*K);

	//K'*R*K
	retval = uquad_mat_transpose(lqr_aux7,K);
	cleanup_if(retval);
	retval = uquad_mat_prod(lqr_aux8,lqr_aux7,R);
	cleanup_if(retval);
	retval = uquad_mat_prod(lqr_aux9,lqr_aux8,K);
	cleanup_if(retval);

	//A+B*K
	retval = uquad_mat_prod(lqr_aux_10,B,K);
	cleanup_if(retval);
	retval = uquad_mat_add(lqr_aux_10,lqr_aux_10,A);
	cleanup_if(retval);

	retval = uquad_mat_transpose(lqr_aux_11,lqr_aux_10);
	cleanup_if(retval);
	retval = uquad_mat_prod(lqr_aux_12,lqr_aux_11,lqr_P);
	cleanup_if(retval);
	retval = uquad_mat_prod(lqr_aux_11,lqr_aux_12,lqr_aux_10);
	cleanup_if(retval);

	retval = uquad_mat_add(lqr_P,Q,lqr_aux9);
	cleanup_if(retval);
	retval = uquad_mat_add(lqr_P,lqr_P,lqr_aux_11);
	cleanup_if(retval);

	retval = uquad_mat_sub(lqr_aux_13,K,lqr_k);
	cleanup_if(retval);

	norm = uquad_mat_norm(lqr_aux_13);

	/// Check timeout
	err_time = gettimeofday(&tv_tmp, NULL);
	if(err_time < 0)
	{
	    retval = ERROR_TIMING;
	    err_log_stderr("gettimeofday()");
	    cleanup_if(retval);
	}
	err_time = uquad_timeval_substract(&tv_diff, tv_tmp, tv_in);
	if(err_time < 0)
	{
	    retval = ERROR_TIMING;
	    cleanup_log_if(retval,"Absurd timing!");
	}
	err_time = uquad_timeval_substract(&tv_diff, tv_diff, tv_out);
	if(err_time >= 0)
	{
	    /// Timed out, get out.
	    break;
	}
    }

    /**
     * Now check if we managed to finish before getting timed out.
     * If we were timed out, then iterations will continue in the
     * next call.
     */
    if(!(norm>CTRL_LQR_TH))
    {
	retval = uquad_mat_scalar_mul(K,K,-1);
	cleanup_if(retval);
	*finished = true;
    }

    cleanup:
    /* uquad_mat_free(lqr_aux0); */
    /* uquad_mat_free(lqr_aux1); */
    /* uquad_mat_free(lqr_aux2); */
    /* uquad_mat_free(lqr_aux3); */
    /* uquad_mat_free(lqr_aux4); */
    /* uquad_mat_free(lqr_aux5); */
    /* uquad_mat_free(lqr_aux6); */
    /* uquad_mat_free(lqr_aux7); */
    /* uquad_mat_free(lqr_aux8); */
    /* uquad_mat_free(lqr_aux9); */
    /* uquad_mat_free(lqr_aux_10); */
    /* uquad_mat_free(lqr_aux_11); */
    /* uquad_mat_free(lqr_aux_12); */
    /* uquad_mat_free(lqr_k); */
    /* uquad_mat_free(lqr_P); */
    /* uquad_mat_free(lqr_Bt); */

    return retval;
}

int control_lin_model(uquad_mat_t *A, uquad_mat_t *B, path_type_t pt, set_point_t *sp, double weight)
{
 if(A == NULL || B == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
 int retval;
 double *xs = sp->x->m_full;
 double *ws = sp->w->m_full;

 retval = uquad_mat_zeros(A);
 err_propagate(retval);
 retval = uquad_mat_zeros(B);
 err_propagate(retval);

 if(pt==CIRCULAR)
 {
     A->m[0][1] = -xs[SV_WQZ];
     A->m[0][2] = xs[SV_WQY];
     A->m[0][6] = 1;
     A->m[0][10] = xs[SV_Z];
     A->m[0][11] = -xs[SV_Y];

     A->m[1][0] = xs[SV_WQZ];
     A->m[1][2] = -xs[SV_WQX];
     A->m[1][7] = 1;
     A->m[1][9] = -xs[SV_Z];
     A->m[1][11] = xs[SV_Y];

     A->m[2][0] = -xs[SV_WQY];
     A->m[2][1] = xs[SV_WQX];
     A->m[2][8] = 1;
     A->m[2][9] = xs[SV_Y];
     A->m[2][10] = -xs[SV_X];

 }
 else{
     A->m[0][3] = xs[SV_VQZ]*(cos(xs[SV_PSI])*sin(xs[SV_THETA])
			      -cos(xs[SV_THETA])*sin(xs[SV_PHI])*sin(xs[SV_PSI]))
	 +xs[SV_VQY]*cos(xs[SV_PSI])*cos(xs[SV_THETA])*sin(xs[SV_PHI]);
     A->m[0][4] = xs[SV_VQY]*(sin(xs[SV_PHI])*sin(xs[SV_THETA])+cos(xs[SV_THETA])*cos(xs[SV_PHI])*sin(xs[SV_PSI]))
	 -xs[SV_VQX]*cos(xs[SV_THETA])*sin(xs[SV_PHI])+xs[SV_VQZ]*cos(xs[SV_PSI])*cos(xs[SV_PHI])*cos(xs[SV_THETA]);
     A->m[0][5] = xs[SV_VQZ]*(sin(xs[SV_PSI])*cos(xs[SV_THETA])-cos(xs[SV_PSI])*sin(xs[SV_PHI])*sin(xs[SV_THETA]))
	 -xs[SV_VQY]*(cos(xs[SV_PHI])*cos(xs[SV_THETA])+sin(xs[SV_PSI])*sin(xs[SV_PHI])*sin(xs[SV_THETA]))
	 -xs[SV_VQX]*cos(xs[SV_PHI])*sin(xs[SV_THETA]);
     A->m[0][6] = cos(xs[SV_PHI])*cos(xs[SV_THETA]);
     A->m[0][7] = cos(xs[SV_THETA])*sin(xs[SV_PSI])*sin(xs[SV_PHI])-cos(xs[SV_PHI])*sin(xs[SV_THETA]);
     A->m[0][8] = sin(xs[SV_PSI])*sin(xs[SV_THETA])+cos(xs[SV_PSI])*cos(xs[SV_THETA])*sin(xs[SV_PHI]);

     A->m[1][3] = -xs[SV_VQY]*(cos(xs[SV_THETA])*sin(xs[SV_PSI])-cos(xs[SV_PSI])*sin(xs[SV_PHI])*sin(xs[SV_THETA]))
	 - xs[SV_VQZ]*(cos(xs[SV_PSI])*cos(xs[SV_THETA])+sin(xs[SV_PSI])*sin(xs[SV_PHI])*sin(xs[SV_THETA]));
     A->m[1][4] = xs[SV_VQY]*sin(xs[SV_THETA])*cos(xs[SV_PHI])*sin(xs[SV_PSI])
	 -xs[SV_VQX]*sin(xs[SV_THETA])*sin(xs[SV_PHI])+xs[SV_VQZ]*cos(xs[SV_PSI])*cos(xs[SV_PHI])*sin(xs[SV_THETA]);
     A->m[1][5] = xs[SV_VQZ]*(sin(xs[SV_PSI])*sin(xs[SV_THETA])+cos(xs[SV_PSI])*sin(xs[SV_PHI])*cos(xs[SV_THETA]))
	 -xs[SV_VQY]*(cos(xs[SV_PSI])*sin(xs[SV_THETA])-sin(xs[SV_PSI])*sin(xs[SV_PHI])*cos(xs[SV_THETA]))
	 +xs[SV_VQX]*cos(xs[SV_PHI])*cos(xs[SV_THETA]);
     A->m[1][6] = cos(xs[SV_PHI])*sin(xs[SV_THETA]);
     A->m[1][7] = cos(xs[SV_PSI])*cos(xs[SV_THETA])+ sin(xs[SV_PSI])*sin(xs[SV_PHI])*sin(xs[SV_THETA]);
     A->m[1][8] = -sin(xs[SV_PSI])*cos(xs[SV_THETA])+cos(xs[SV_PSI])*sin(xs[SV_THETA])*sin(xs[SV_PHI]);

     A->m[2][3] = -xs[SV_VQY]*(cos(xs[SV_THETA])*sin(xs[SV_PSI])-cos(xs[SV_PSI])*sin(xs[SV_PHI])*sin(xs[SV_THETA]))
	 -xs[SV_VQZ]*(cos(xs[SV_PSI])*cos(xs[SV_THETA])+sin(xs[SV_PSI])*sin(xs[SV_PHI])*sin(xs[SV_THETA]));
     A->m[2][4] = xs[SV_VQZ]*cos(xs[SV_PSI])*cos(xs[SV_PHI])*sin(xs[SV_THETA])-xs[SV_VQX]*sin(xs[SV_PHI])*sin(xs[SV_THETA])
	 +xs[SV_VQY]*sin(xs[SV_PSI])*cos(xs[SV_PHI])*sin(xs[SV_THETA]);
     A->m[2][6] = -sin(xs[SV_PHI]);
     A->m[2][7] = cos(xs[SV_PHI])*sin(xs[SV_PSI]);
     A->m[2][8] = cos(xs[SV_PHI])*cos(xs[SV_PSI]);
 }


 A->m[3][3] = xs[SV_WQY]*cos(xs[SV_PSI])*tan(xs[SV_PHI])-xs[SV_WQZ]*sin(xs[SV_PSI])*tan(xs[SV_PHI]);
 A->m[3][4] = (xs[SV_WQZ]*cos(xs[SV_PSI])+xs[SV_WQY]*sin(xs[SV_PSI]))*(uquad_square(tan(xs[SV_PHI]))+1);
 A->m[3][9] = 1;
 A->m[3][10] = sin(xs[SV_PSI])*tan(xs[SV_PHI]);
 A->m[3][11] = cos(xs[SV_PSI])*tan(xs[SV_PHI]);

 A->m[4][3] = -xs[SV_WQY]*sin(xs[SV_PSI])-xs[SV_WQZ]*cos(xs[SV_PSI]);
 A->m[4][10] = cos(xs[SV_PSI]);
 A->m[4][11] = -sin(xs[SV_PSI]);

 A->m[5][3] = (xs[SV_WQY]*cos(xs[SV_PSI])-xs[SV_WQZ]*sin(xs[SV_PSI]))/cos(xs[SV_PHI]);
 A->m[5][4] = (xs[SV_WQY]*sin(xs[SV_PSI])*sin(xs[SV_PHI])+xs[SV_WQZ]*cos(xs[SV_PSI])*sin(xs[SV_PHI]))/uquad_square(cos(xs[SV_PHI]));
 A->m[5][10] = sin(xs[SV_PSI])/cos(xs[SV_PHI]);
 A->m[5][11] = cos(xs[SV_PSI])/cos(xs[SV_PHI]);

 A->m[6][4] = GRAVITY*cos(xs[SV_PHI]);
 A->m[6][7] = xs[SV_WQZ];
 A->m[6][8] = -xs[SV_WQY];
 A->m[6][10] = -xs[SV_VQZ];
 A->m[6][11] = xs[SV_VQY];

 A->m[7][3] = -GRAVITY*cos(xs[SV_PSI])*cos(xs[SV_PHI]);
 A->m[7][4] = GRAVITY*sin(xs[SV_PSI])*sin(xs[SV_PHI]);
 A->m[7][6] = -xs[SV_WQZ];
 A->m[7][8] = xs[SV_WQX];
 A->m[7][9] = xs[SV_VQZ];
 A->m[7][11] = -xs[SV_VQX];

 A->m[8][3] = GRAVITY*sin(xs[SV_PSI])*cos(xs[SV_PHI]);
 A->m[8][4] = GRAVITY*cos(xs[SV_PSI])*sin(xs[SV_PHI]);
 A->m[8][6] = xs[SV_WQY];
 A->m[8][7] = -xs[SV_WQX];
 A->m[8][9] = -xs[SV_VQY];
 A->m[8][10] = xs[SV_VQX];

 A->m[9][3]  = -D_CENTER_MASS_M*weight*GRAVITY*cos(xs[SV_PHI])*cos(xs[SV_PSI])/IXX;
 A->m[9][4]  = D_CENTER_MASS_M*weight*GRAVITY*sin(xs[SV_PHI])*sin(xs[SV_PSI])/IXX;
 A->m[9][10] = IZZM/IXX*(ws[0]-ws[1]+ws[2]-ws[3])+(IYY-IZZ)/IXX*xs[SV_WQZ];
 A->m[9][11] = (IYY-IZZ)/IXX*xs[SV_WQY];

 A->m[10][4] = -D_CENTER_MASS_M*weight*GRAVITY*cos(xs[SV_PHI])/IYY;
 A->m[10][9] = IZZM/IYY*(ws[0]-ws[1]+ws[2]-ws[3])-(IXX-IXX)/IXX*xs[SV_WQZ];
 A->m[10][10] = 0;
 A->m[10][11] = -(IXX-IZZ)/IYY*xs[SV_WQZ];

 B->m[8][0]=(2*F_B1*ws[0]+F_B2)/weight;
 B->m[8][1]=(2*F_B1*ws[1]+F_B2)/weight;
 B->m[8][2]=(2*F_B1*ws[2]+F_B2)/weight;
 B->m[8][3]=(2*F_B1*ws[3]+F_B2)/weight;

 B->m[9][0] = xs[SV_WQY]*IZZM/IYY;
 B->m[9][1] = -xs[SV_WQY]*IZZM/IYY+LENGTH*(2*F_B1*ws[1]+F_B2)/IXX;
 B->m[9][2] = xs[SV_WQY]*IZZM/IYY;
 B->m[9][3] = -xs[SV_WQY]*IZZM/IYY-LENGTH*(2*F_B1*ws[3]+F_B2)/IXX;

 B->m[10][0] = xs[SV_WQX]*IZZM/IYY-LENGTH*(2*F_B1*ws[0]+F_B2)/IYY;
 B->m[10][1] = -xs[SV_WQX]*IZZM/IYY;
 B->m[10][2] = xs[SV_WQX]*IZZM/IYY+LENGTH*(2*F_B1*ws[2]+F_B2)/IYY;
 B->m[10][3] = -xs[SV_WQX]*IZZM/IYY;

 B->m[11][0]=-(2*M_D1*ws[0]+M_D2)/IZZ;
 B->m[11][1]=(2*M_D1*ws[1]+M_D2)/IZZ;
 B->m[11][2]=-(2*M_D1*ws[2]+M_D2)/IZZ;
 B->m[11][3]=(2*M_D1*ws[3]+M_D2)/IZZ;

 return retval;
}
