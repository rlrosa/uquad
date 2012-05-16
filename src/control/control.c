#include "control.h"
#include <uquad_error_codes.h>
#include <math.h>
#include <stdlib.h>
#include <uquad_aux_time.h>

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

int control_update_K(path_planner_t *pp, double weight)
{
    int retval;
    uquad_mat_t *A = NULL;
    uquad_mat_t *B = NULL;
    uquad_mat_t *Aext = NULL;
    uquad_mat_t *Bext = NULL;
    uquad_mat_t *phi = NULL;
    uquad_mat_t *gamma = NULL;
    
    //Linearization of the system
    A = uquad_mat_alloc(STATES_CONTROLLED,STATES_CONTROLLED);
    B = uquad_mat_alloc(STATES_CONTROLLED,LENGTH_INPUT);
    retval = control_lin_model(A,B,pp->pt,pp->sp, weight);
    cleanup_if(retval);

    //Extends the system to include the integrated states
    Aext = uquad_mat_alloc(2*STATES_CONTROLLED, 2*STATES_CONTROLLED);
    Bext = uquad_mat_alloc(2*STATES_CONTROLLED, LENGTH_INPUT);
    retval = uquad_mat_zeros(Aext);
    cleanup_if(retval);
    retval = uquad_mat_zeros(Bext);
    cleanup_if(retval);
    retval = uquad_mat_set_subm(Aext,0,0,A);
    cleanup_if(retval);
    retval = uquad_mat_eye(A);
    cleanup_if(retval);
    retval = uquad_mat_set_subm(Aext,STATES_CONTROLLED,0,A);
    cleanup_if(retval);
    retval = uquad_mat_set_subm(Bext,0,0,B);
    cleanup_if(retval);
    //Discretization of the system;
    phi = uquad_mat_alloc(Aext->r,Aext->c);
    gamma = uquad_mat_alloc(Bext->r,Bext->c);
    retval = control_disc(phi, gamma, Aext, Bext, pp->Ts);
    cleanup_if(retval);
    //Obtains feedback matrix  
    retval = control_lqr(pp->K, phi, gamma, pp->Q, pp->R);
    cleanup_if(retval);

    cleanup:
    uquad_mat_free(A);
    uquad_mat_free(B);
    uquad_mat_free(Aext);
    uquad_mat_free(Bext);
    uquad_mat_free(phi);
    uquad_mat_free(gamma);

    return retval;
}

int control_disc(uquad_mat_t *phi,uquad_mat_t *gamma,uquad_mat_t *A,uquad_mat_t *B, double Ts)
{
    int retval;
    uquad_mat_t *aux0 = NULL;
    uquad_mat_t *aux1 = NULL;
    
    aux0 = uquad_mat_alloc(A->r+B->c,A->c+B->c);
    aux1 = uquad_mat_alloc(A->r+B->c,A->c+B->c);

    retval = uquad_mat_set_subm(aux0, 0,0,A);
    cleanup_if(retval);
    retval =  uquad_mat_set_subm(aux0, 0,A->c,B);
    cleanup_if(retval);
    retval = uquad_mat_scalar_mul(aux0,aux0,Ts); 
    cleanup_if(retval);
    retval = uquad_mat_exp(aux1,aux0);
    cleanup_if(retval);
    retval= uquad_mat_get_subm(phi,0,0,aux1);
    cleanup_if(retval);
    retval = uquad_mat_get_subm(gamma,0,A->c,aux1);
    cleanup_if(retval);
    //Cleaning
    cleanup:
    uquad_mat_free(aux0);
    uquad_mat_free(aux1);

    return retval;
}

int control_lqr(uquad_mat_t *K, uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *Q, uquad_mat_t *R)
{
    int retval;
    double norm;
    uquad_mat_t *aux0 = NULL;
    uquad_mat_t *aux1 = NULL;
    uquad_mat_t *aux2 = NULL;
    uquad_mat_t *aux3 = NULL;
    uquad_mat_t *aux4 = NULL;
    uquad_mat_t *aux5 = NULL;
    uquad_mat_t *aux6 = NULL;
    uquad_mat_t *aux7 = NULL;
    uquad_mat_t *aux8 = NULL;
    uquad_mat_t *aux9 = NULL;
    uquad_mat_t *aux10 = NULL;
    uquad_mat_t *aux11 = NULL;
    uquad_mat_t *aux12 = NULL;
    uquad_mat_t *aux13 = NULL;
    uquad_mat_t *P = NULL;
    uquad_mat_t *k = NULL;
    uquad_mat_t *Bt = NULL;

    struct timeval
	tv_in,
	tv_tmp,
	tv_diff,
	tv_out;
    tv_out.tv_sec = 1;
    tv_out.tv_usec = 0;
    retval = gettimeofday(&tv_in,NULL);
    if(retval < 0)
    {
	err_log_stderr("gettimeofday()");
	retval = ERROR_TIMING;
	cleanup_if(retval);
    }

    P = uquad_mat_alloc(Q->r,Q->r);
    k = uquad_mat_alloc(K->r,K->c);

    retval = uquad_mat_copy(P,Q);
    cleanup_if(retval);
    retval = uquad_mat_copy(k,K);
    cleanup_if(retval);
    Bt = uquad_mat_alloc(B->c,B->r);
   
    retval = uquad_mat_transpose(Bt,B);
    cleanup_if(retval);
    norm=1;


    aux0 = uquad_mat_alloc(B->r,B->c);
    aux1 = uquad_mat_alloc(R->r,R->c);
    aux2 = uquad_mat_alloc(R->r,R->c);
    aux3 = uquad_mat_alloc(R->r,R->c);
    aux4 = uquad_mat_alloc(R->r,2*R->c);
    aux5 = uquad_mat_alloc(R->r,Q->c);
    aux6 = uquad_mat_alloc(R->r,Q->c);
    aux7 = uquad_mat_alloc(K->c,K->r);
    aux8 = uquad_mat_alloc(K->c,K->r);
    aux9 = uquad_mat_alloc(Q->r,Q->c);
    aux10 = uquad_mat_alloc(Q->r,Q->c);	
    aux11 = uquad_mat_alloc(Q->c,Q->r);
    aux12 = uquad_mat_alloc(Q->c,Q->r);
    aux13 = uquad_mat_alloc(K->r,K->c);

    while (norm>1e-5)
    {
	retval = uquad_mat_copy(k,K);
	cleanup_if(retval);
	//Performs K=-(R+B'*P*B)^(-1)*B'*P*A; 
	retval = uquad_mat_prod(aux0,P,B);
	cleanup_if(retval);
	retval = uquad_mat_prod(aux1,Bt,aux0);
	cleanup_if(retval);
	retval = uquad_mat_add(aux1,aux1,R);
	cleanup_if(retval);
	retval = uquad_mat_inv(aux2,aux1,aux3,aux4);  
	cleanup_if(retval);
	
	retval = uquad_mat_prod(aux5,aux2,Bt);
	cleanup_if(retval);
	retval = uquad_mat_prod(aux6,aux5,P);
	cleanup_if(retval);
	retval = uquad_mat_prod(aux5,aux6,A);
	cleanup_if(retval);
	retval = uquad_mat_scalar_mul(K,aux5,-1);
	cleanup_if(retval);

	//Performs P=Q+K'*R*K+(A+B*K)'*P*(A+B*K);

	//K'*R*K
	retval = uquad_mat_transpose(aux7,K);
	cleanup_if(retval);
	retval = uquad_mat_prod(aux8,aux7,R);
	cleanup_if(retval);
	retval = uquad_mat_prod(aux9,aux8,K);
	cleanup_if(retval);

	//A+B*K
	retval = uquad_mat_prod(aux10,B,K);
	cleanup_if(retval);
	retval = uquad_mat_add(aux10,aux10,A);
	cleanup_if(retval);

	retval = uquad_mat_transpose(aux11,aux10);
	cleanup_if(retval);
	retval = uquad_mat_prod(aux12,aux11,P);
	cleanup_if(retval);
	retval = uquad_mat_prod(aux11,aux12,aux10);
	cleanup_if(retval);

	retval = uquad_mat_add(P,Q,aux9);
	cleanup_if(retval);
	retval = uquad_mat_add(P,P,aux11);
	cleanup_if(retval);

	retval = uquad_mat_sub(aux13,K,k);
	cleanup_if(retval);

	norm = uquad_mat_norm(aux13);

	/// Check timeout
	retval = gettimeofday(&tv_tmp, NULL);
	if(retval < 0)
	{
	    retval = ERROR_TIMING;
	    err_log_stderr("gettimeofday()");
	    cleanup_if(retval);
	}
	retval = uquad_timeval_substract(&tv_diff, tv_tmp, tv_in);
	if(retval < 0)
	{
	    retval = ERROR_TIMING;
	    cleanup_log_if(retval,"Absurd timing!");
	}
	retval = uquad_timeval_substract(&tv_diff, tv_diff, tv_out);
	if(retval >= 0)
	{
	    retval = ERROR_FAIL;
	    cleanup_log_if(retval,"ERR: Timed out!");
	}
    }

    retval = uquad_mat_scalar_mul(K,K,-1);
    cleanup_if(retval);

    cleanup:
    uquad_mat_free(aux0);
    uquad_mat_free(aux1);
    uquad_mat_free(aux2);
    uquad_mat_free(aux3);
    uquad_mat_free(aux4);
    uquad_mat_free(aux5);
    uquad_mat_free(aux6);
    uquad_mat_free(aux7);
    uquad_mat_free(aux8);
    uquad_mat_free(aux9);
    uquad_mat_free(aux10);
    uquad_mat_free(aux11);
    uquad_mat_free(aux12);
    uquad_mat_free(k);
    uquad_mat_free(P);

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
     A->m[0][1] = -xs[11];
     A->m[0][2] = xs[10];
     A->m[0][6] = 1;
     A->m[0][10] = xs[2];
     A->m[0][11] = -xs[1];

     A->m[1][0] = xs[11];
     A->m[1][2] = -xs[9];
     A->m[1][7] = 1;
     A->m[1][9] = -xs[2];
     A->m[1][11] = xs[1];

     A->m[2][0] = -xs[10];
     A->m[2][1] = xs[9];
     A->m[2][8] = 1;
     A->m[2][9] = xs[1];
     A->m[2][10] = -xs[0];

 }
 else{
     A->m[0][3] = xs[8]*(cos(xs[3])*sin(xs[5])
			      -cos(xs[5])*sin(xs[4])*sin(xs[3]))
	 +xs[7]*cos(xs[3])*cos(xs[5])*sin(xs[4]);
     A->m[0][4] = xs[7]*(sin(xs[4])*sin(xs[5])+cos(xs[5])*cos(xs[4])*sin(xs[3]))
	 -xs[6]*cos(xs[5])*sin(xs[4])+xs[8]*cos(xs[3])*cos(xs[4])*cos(xs[5]);
     A->m[0][5] = xs[8]*(sin(xs[3])*cos(xs[5])-cos(xs[3])*sin(xs[4])*sin(xs[5]))
	 -xs[7]*(cos(xs[4])*cos(xs[5])+sin(xs[3])*sin(xs[4])*sin(xs[5]))
	 -xs[6]*cos(xs[4])*sin(xs[5]);
     A->m[0][6] = cos(xs[4])*cos(xs[5]);
     A->m[0][7] = cos(xs[5])*sin(xs[3])*sin(xs[4])-cos(xs[4])*sin(xs[5]);
     A->m[0][8] = sin(xs[3])*sin(xs[5])+cos(xs[3])*cos(xs[5])*sin(xs[4]);

     A->m[1][3] = -xs[7]*(cos(xs[5])*sin(xs[3])-cos(xs[3])*sin(xs[4])*sin(xs[5]))
	 - xs[8]*(cos(xs[3])*cos(xs[5])+sin(xs[3])*sin(xs[4])*sin(xs[5]));
     A->m[1][4] = xs[7]*sin(xs[5])*cos(xs[4])*sin(xs[3])
	 -xs[6]*sin(xs[5])*sin(xs[4])+xs[8]*cos(xs[3])*cos(xs[4])*sin(xs[5]);
     A->m[1][5] = xs[8]*(sin(xs[3])*sin(xs[5])+cos(xs[3])*sin(xs[4])*cos(xs[5]))
	 -xs[7]*(cos(xs[3])*sin(xs[5])-sin(xs[3])*sin(xs[4])*cos(xs[5]))
	 +xs[6]*cos(xs[4])*cos(xs[5]);
     A->m[1][6] = cos(xs[4])*sin(xs[5]);
     A->m[1][7] = cos(xs[3])*cos(xs[5])+ sin(xs[3])*sin(xs[4])*sin(xs[5]);
     A->m[1][8] = -sin(xs[3])*cos(xs[5])+cos(xs[3])*sin(xs[5])*sin(xs[4]);

     A->m[2][3] = -xs[7]*(cos(xs[5])*sin(xs[3])-cos(xs[3])*sin(xs[4])*sin(xs[5]))
	 -xs[8]*(cos(xs[3])*cos(xs[5])+sin(xs[3])*sin(xs[4])*sin(xs[5]));
     A->m[2][4] = xs[8]*cos(xs[3])*cos(xs[4])*sin(xs[5])-xs[6]*sin(xs[4])*sin(xs[5])
	 +xs[7]*sin(xs[3])*cos(xs[4])*sin(xs[5]);
     A->m[2][6] = -sin(xs[4]);
     A->m[2][7] = cos(xs[4])*sin(xs[3]);
     A->m[2][8] = cos(xs[4])*cos(xs[3]);
 }


 A->m[3][3] = xs[10]*cos(xs[3])*tan(xs[4])-xs[11]*sin(xs[3])*tan(xs[4]);
 A->m[3][4] = (xs[11]*cos(xs[3])+xs[10]*sin(xs[3]))*(uquad_square(tan(xs[4]))+1);
 A->m[3][9] = 1;
 A->m[3][10] = sin(xs[3])*tan(xs[4]);
 A->m[3][11] = cos(xs[3])*tan(xs[4]);

 A->m[4][3] = -xs[10]*sin(xs[3])-xs[11]*cos(xs[3]);
 A->m[4][10] = cos(xs[3]);
 A->m[4][11] = -sin(xs[3]);

 A->m[5][3] = (xs[10]*cos(xs[3])-xs[11]*sin(xs[3]))/cos(xs[4]);
 A->m[5][4] = (xs[10]*sin(xs[3])*sin(xs[4])+xs[11]*cos(xs[3])*sin(xs[4]))/uquad_square(cos(xs[4]));
 A->m[5][10] = sin(xs[3])/cos(xs[4]);
 A->m[5][11] = cos(xs[3])/cos(xs[4]);

 A->m[6][4] = 9.81*cos(xs[4]);
 A->m[6][7] = xs[11];
 A->m[6][8] = -xs[10];
 A->m[6][10] = -xs[8];
 A->m[6][11] = xs[7];

 A->m[7][3] = -9.81*cos(xs[3])*cos(xs[4]);
 A->m[7][4] = 9.81*sin(xs[3])*sin(xs[4]);
 A->m[7][6] = -xs[11];
 A->m[7][8] = xs[9];
 A->m[7][9] = xs[8];
 A->m[7][11] = -xs[6];

 A->m[8][3] = 9.81*sin(xs[3])*cos(xs[4]);
 A->m[8][4] = 9.81*cos(xs[3])*sin(xs[4]);
 A->m[8][6] = xs[10];
 A->m[8][7] = -xs[9];
 A->m[8][9] = -xs[7];
 A->m[8][10] = xs[6];

 A->m[9][10] = IZZM/IXX*(ws[0]-ws[1]+ws[2]-ws[3])+(IYY-IZZ)/IXX*xs[11];
 A->m[9][11] = (IYY-IZZ)/IXX*xs[10];

 A->m[10][9] = IZZM/IYY*(ws[0]-ws[1]+ws[2]-ws[3])-(IXX-IXX)/IXX*xs[11];
 A->m[10][10] = 0;
 A->m[10][11] = -(IXX-IZZ)/IYY*xs[11];

 B->m[8][0]=(2*F_B1*ws[0]+F_B2)/weight;
 B->m[8][1]=(2*F_B1*ws[1]+F_B2)/weight;
 B->m[8][2]=(2*F_B1*ws[2]+F_B2)/weight;
 B->m[8][3]=(2*F_B1*ws[3]+F_B2)/weight;

 B->m[9][0] = xs[10]*IZZM/IYY;
 B->m[9][1] = -xs[10]*IZZM/IYY+LENGTH*(2*F_B1*ws[1]+F_B2)/IXX;
 B->m[9][2] = xs[10]*IZZM/IYY;
 B->m[9][3] = -xs[10]*IZZM/IYY-LENGTH*(2*F_B1*ws[3]+F_B2)/IXX;

 B->m[10][0] = xs[9]*IZZM/IYY-LENGTH*(2*F_B1*ws[0]+F_B2)/IYY;
 B->m[10][1] = -xs[9]*IZZM/IYY;
 B->m[10][2] = xs[9]*IZZM/IYY+LENGTH*(2*F_B1*ws[2]+F_B2)/IYY;
 B->m[10][3] = -xs[9]*IZZM/IYY;

 B->m[11][0]=-(2*M_D1*ws[0]+M_D2)/IZZ;
 B->m[11][1]=(2*M_D1*ws[1]+M_D2)/IZZ;
 B->m[11][2]=-(2*M_D1*ws[2]+M_D2)/IZZ;
 B->m[11][3]=(2*M_D1*ws[3]+M_D2)/IZZ;

 return retval;
}
