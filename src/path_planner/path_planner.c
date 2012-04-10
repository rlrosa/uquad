#include "path_planner.h"
#include <mot_control.h>
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
    sp->x->m_full[0] = 0; 
    sp->w->m_full[0] = MOT_W_HOVER;
    sp->w->m_full[1] = MOT_W_HOVER;
    sp->w->m_full[2] = MOT_W_HOVER;
    sp->w->m_full[3] = MOT_W_HOVER;
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


int pp_update_setpoint(path_planner_t *pp, uquad_mat_t *x)
{
    if (pp->pt != HOVER)
    {
	err_check(ERROR_FAIL, "Not implemented!");
    }
    if(pp == NULL || x == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument.");
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

/* This functions returns the discrete LQR realimentation matrix */
/* Supose a continuous time system that verifies \dot{X}=AX+BU where X is the state vector and u the input of the system.  */

/* The discretization of the system is X(k+1)=phi*X(k)+gamma*U, */

/* Where phi=exp(A*Ts) and gama=int(exp(A*s),0,Ts) */
int pp_update_K(path_planner_t *pp)
{
    uquad_mat_t *phi = NULL;
    uquad_mat_t *gamma = NULL;
    
    //Discretization of the system;
    phi = uquad_mat_alloc(pp->A->r,pp->A->c);
    gamma = uquad_mat_alloc(pp->B->r,pp->B->c);
    pp_disc(phi, gamma, pp->A, pp->B, pp->Ts);
    
    //Obtains feedback matrix  
    pp_lqr(pp->K, phi, gamma, pp->Q, pp->R);
    return ERROR_OK;
}
/*Returns the discrete form of a system \dot{x} = AX+BU*/
int pp_disc(uquad_mat_t *phi,uquad_mat_t *gamma,uquad_mat_t *A,uquad_mat_t *B, double Ts)
{
    uquad_mat_t *aux0 = NULL;
    uquad_mat_t *aux1 = NULL;
    
    phi = uquad_mat_alloc(A->r,A->c);
    gamma = uquad_mat_alloc(B->r,B->c);
    aux0 = uquad_mat_alloc(A->r+B->c,A->c+B->c);
    aux1 = uquad_mat_alloc(A->r+B->c,A->c+B->c);

    uquad_mat_set_subm(aux0, 0,0,A);
    uquad_mat_set_subm(aux0, 0,A->c,B);
    uquad_mat_scalar_mul(aux0,aux0,Ts); 
    uquad_mat_exp(aux1,aux0);
    
    uquad_mat_get_subm(phi,0,0,aux1);
    uquad_mat_get_subm(gamma,0,A->c,aux1);

    //Cleaning
    uquad_mat_free(aux0);
    uquad_mat_free(aux1);

}
 //Solving riccati equation. Algorithm by Anders Friis Sorensen    
int pp_lqr(uquad_mat_t *K, uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *Q, uquad_mat_t *R)
{
    int norm;
    uquad_mat_t *aux0 = NULL;
    uquad_mat_t *aux1 = NULL;
    uquad_mat_t *aux2 = NULL;
    uquad_mat_t *aux3 = NULL;
    uquad_mat_t *aux4 = NULL;
    uquad_mat_t *P = NULL;
    uquad_mat_t *k = NULL;
    uquad_mat_t *Bt = NULL;

    P = uquad_mat_alloc(Q->r,Q->r);
    k = uquad_mat_alloc(K->r,K->c);

    uquad_mat_copy(P,Q);
    uquad_mat_copy(k,K);

    Bt = uquad_mat_alloc(B->c,B->r);
   
    uquad_mat_transpose(Bt,B);
    norm=1;
    while (norm>1e-5)
    {
        aux0 = uquad_mat_alloc(B->r,B->c);
	aux1 = uquad_mat_alloc(R->r,R->c);
        aux2 = uquad_mat_alloc(R->r,R->c);
	aux3 = uquad_mat_alloc(R->r,R->c);
	aux4 = uquad_mat_alloc(2*R->r,2*R->c);

	uquad_mat_copy(k,K);

	//Performs K=-(R+B'*P*B)^(-1)*B'*P*A; 
	uquad_mat_prod(aux0,P,B);
	uquad_mat_prod(aux1,Bt,aux0);
	uquad_mat_add(aux1,aux1,R);
	uquad_mat_inv(aux2,aux1,aux3,aux4);
	
	uquad_mat_free(aux0);
	uquad_mat_free(aux1);
	aux0 = uquad_mat_alloc(R->r,Q->c);
	aux1 = uquad_mat_alloc(R->r,Q->c);
	uquad_mat_prod(aux0,aux2,Bt);
	uquad_mat_prod(aux1,aux0,P);
	uquad_mat_prod(aux0,aux1,A);
	uquad_mat_scalar_mul(K,aux0,-1);
	
	uquad_mat_free(aux0);
	uquad_mat_free(aux1);
	uquad_mat_free(aux2);
	uquad_mat_free(aux3);
	uquad_mat_free(aux4);

	//Performs P=Q+K'*R*K+(A+B*K)'*P*(A+B*K);
	aux0 = uquad_mat_alloc(K->c,K->r);
	aux1 = uquad_mat_alloc(K->c,K->r);
	aux2 = uquad_mat_alloc(Q->r,Q->c);
	aux3 = uquad_mat_alloc(Q->r,Q->c);

	//K'*R*K'
	uquad_mat_transpose(aux0,K);
	uquad_mat_prod(aux1,aux0,R);
	uquad_mat_prod(aux2,aux1,K);
	
	uquad_mat_free(aux0);
	uquad_mat_free(aux1);

	//A+B*K
	uquad_mat_prod(aux3,B,K);
	uquad_mat_add(aux3,aux3,A);
	
	aux0 = uquad_mat_alloc(Q->c,Q->r);
	aux1 = uquad_mat_alloc(K->c,K->r);

	uquad_mat_transpose(aux0,aux3);
	uquad_mat_prod(aux1,aux0,P);
	uquad_mat_prod(aux0,aux1,aux3);
	
	uquad_mat_free(aux1);
	uquad_mat_free(aux3);

	uquad_mat_add(P,P,Q);
	uquad_mat_add(P,P,aux2);
	uquad_mat_add(P,P,aux0);

	uquad_mat_free(aux0);
	uquad_mat_free(aux1);
    }

    uquad_mat_scalar_mul(K,K,-1);
}

