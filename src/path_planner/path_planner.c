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
    int retval;
    uquad_mat_t *A = NULL;
    uquad_mat_t *B = NULL;
    uquad_mat_t *Aext = NULL;
    uquad_mat_t *Bext = NULL;
    uquad_mat_t *phi = NULL;
    uquad_mat_t *gamma = NULL;
    
    //Linearization of the system
    A = uquad_mat_alloc(STATES_CONTROLLED,STATES_CONTROLLED);
    B = uquad_mat_alloc(STATES_CONTROLLED,MOT_C);
    retval = pp_lin_model(A,B,pp->pt,pp->sp);
    cleanup_if(retval);

    //Extends the system to include the integrated states
    Aext = uquad_mat_alloc(2*STATES_CONTROLLED, 2*STATES_CONTROLLED);
    Bext = uquad_mat_alloc(2*STATES_CONTROLLED, MOT_C);
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
    retval = pp_disc(phi, gamma, Aext, Bext, pp->Ts);
    cleanup_if(retval);
    //Obtains feedback matrix  
    retval = pp_lqr(pp->K, phi, gamma, pp->Q, pp->R);
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
/*Returns the discrete form of a system \dot{x} = AX+BU*/
int pp_disc(uquad_mat_t *phi,uquad_mat_t *gamma,uquad_mat_t *A,uquad_mat_t *B, double Ts)
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
 //Solving riccati equation. Algorithm by Anders Friis Sorensen    
int pp_lqr(uquad_mat_t *K, uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *Q, uquad_mat_t *R)
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

/*
Performs the linearization of the physic model of the quadrotor
 */
int pp_lin_model(uquad_mat_t *A, uquad_mat_t *B, path_type_t pt, set_point_t *sp)
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

 B->m[8][0]=(2*F_B1*ws[0]+F_B2)/MASA;
 B->m[8][1]=(2*F_B1*ws[1]+F_B2)/MASA;
 B->m[8][2]=(2*F_B1*ws[2]+F_B2)/MASA;
 B->m[8][3]=(2*F_B1*ws[3]+F_B2)/MASA;

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
