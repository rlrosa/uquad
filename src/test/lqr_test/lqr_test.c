#include <uquad_aux_math.h>
#include <path_planner.h>
#include <stdlib.h>

int main(void){
    int retval = ERROR_OK;
    uquad_mat_t *A = NULL;
    uquad_mat_t *B = NULL;
    uquad_mat_t *Q = NULL;
    uquad_mat_t *R = NULL;
    uquad_mat_t *phi = NULL;
    uquad_mat_t *gamma = NULL;
    uquad_mat_t *K =NULL;
    FILE *matrix = NULL;

    matrix = fopen("A","r");
    A = uquad_mat_alloc(24,24);
    uquad_mat_load(A, matrix);

    matrix = fopen("B","r");
    B = uquad_mat_alloc(24,4);
    uquad_mat_load(B,matrix);
    
    matrix = fopen("Q","r"); 
    Q = uquad_mat_alloc(24,24);
    uquad_mat_load(Q,matrix);
       
    matrix = fopen("R","r"); 
    R = uquad_mat_alloc(4,4);
    uquad_mat_load(R,matrix);

    phi = uquad_mat_alloc(A->r, A->c);
    gamma = uquad_mat_alloc(B->r, B->c);
    K = uquad_mat_alloc(4,24);

    pp_disc(phi,gamma,A,B,10e-3);
    pp_lqr(K,phi,gamma,Q,R);

    uquad_mat_dump(K,0);
    
    }
