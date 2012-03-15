#include <uquad_error_codes.h>
#include "uquad_aux_math.h"

#define size 3

void main()
{
    int n = size;
    uquad_mat_t* A = NULL;
    uquad_mat_t* H = NULL;
    uquad_mat_t* S = NULL;
    uquad_mat_t* expA = NULL;
    double eigen_imag[size];
    double eigen_real[size];

    A = uquad_mat_alloc(size,size);
    H = uquad_mat_alloc(size,size);
    S = uquad_mat_alloc(size,size);
    expA = uquad_mat_alloc(size,size);

    A->m[0][0]=1;
    A->m[0][1]=1;
    A->m[0][2]=1;
    A->m[1][0]=1;
    A->m[1][1]=1;
    A->m[1][2]=1;
    A->m[2][0]=1;
    A->m[2][1]=1;
    A->m[2][2]=1;
    
    uquad_mat_dump(A,0);

    uquad_mat_copy(H,A);

    uquad_mat_eig(H,S,eigen_real,eigen_imag,n);	
    //uquad_mat_transpose_inplace(S);
    uquad_mat_dump(H,0);
    uquad_mat_dump(S,0);

    uquad_mat_exp(expA,A,H,eigen_real,n);
    uquad_mat_dump(expA,0);
    
    uquad_mat_free(A);
    uquad_mat_free(H);
    uquad_mat_free(S);
    uquad_mat_free(expA);

    
}
