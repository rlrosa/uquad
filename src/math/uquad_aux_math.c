#include "uquad_aux_math.h"
#include <stdlib.h>

/**
 * The following includes are code by:
 * Dick Horn - mathretprogr at gmail com 
 * http://www.mymathlib.com
 * 
 */
#include "copy_matrix.h"
#include "join_by_rows.c"
#include "join_by_cols.c"
#include "zero_matrix.c"
#include "identity_matrix.c"
#include "set_diagonal.c"
#include "get_diagonal.c"
#include "get_submatrix.c"
#include "set_submatrix.c"
#include "mul_3x3_matrix_by_scalar.h"
#include "add_matrices.c"
#include "add_matrices_3x3.h"
#include "subtract_matrices.c"
#include "subtract_matrices_3x3.h"
#include "multiply_matrices.c"
#include "multiply_matrices_3x3.c"
#include "gauss_elimination.c"
#include "gauss_aux_elimination.c"
#include "transpose_matrix.c"
#include "transpose_square_matrix.c"
#include "equilibrate_matrix.c"
//#include "hessenberg_orthog.c"
//#include "qr_hessenberg_matrix.c"
//#include "doolittle.c"
//#include "doolittle_pivot.c"


/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Matrix
 * -- -- -- -- -- -- -- -- -- -- -- --
 */

/** 
 * Performs:
 *   C = A*B
 * 
 * Checks dimensions before performing operation.
 * Memory for C,A and B must have been previously allocated.
 *
 * If matrices are 3x3, a macro is used (+ efficiency).
 *
 * @param C Result.
 * @param A First operand.
 * @param B Second operand.
 * 
 * @return Error code.
 */
int uquad_mat_prod(uquad_mat_t *C, uquad_mat_t *A,uquad_mat_t *B)
{
    if(A == NULL || B == NULL || C == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }

    // check dims
    if((A->c != B->r) || !((C->r == A->r) && (C->c == B->c)))
    {
	err_check(ERROR_MATH_MAT_DIM,"Cannot multiply matrices, dims do not match.");
    }
    if(C->c == 3 && C->r == 3)
	Multiply_Matrices_3x3(C->m_full,A->m_full,B->m_full);
    else
	Multiply_Matrices(C->m_full,A->m_full,A->r,A->c,B->m_full,B->c);
    return ERROR_OK;
}

int uquad_mat_det(uquad_mat_t *m, double *res)
{
    //TODO use LU
    if(m == NULL || res == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    err_check(ERROR_FAIL,"Not implemented!");
}

/** 
 * Multiplies matrix m by scalar value k.
 * If only one matrix is supplied, will multiply in place.
 *
 * If in place and matrix is 3x3, a macro is used (+efficiency)
 * 
 * @param Mk Result will be returned here.
 * @param M  NULL for in place multiplicacion. Otherwise, M will remain
 * unmodified, and the anwser will be Mk = M*k
 * @param k 
 * 
 * @return error code
 */
int uquad_mat_scalar_mul(uquad_mat_t *Mk, uquad_mat_t *M, double k)
{
    int i, size;
    if(Mk == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    size = Mk->r * Mk->c;
    if(M != NULL)
    {
	if(Mk->r != M->r || Mk->c != M->c)
	{
	    err_check(ERROR_MATH_MAT_DIM,"Cannot multiply matrices, dims do not match.");
	}
	for(i = 0; i < size; ++i)
	    Mk->m_full[i] = M->m_full[i]*k;
    }
    else
    {
	if((Mk->r == 3) && (Mk->c == 3))
	{
	    Multiply_3x3_Matrix_by_Scalar(Mk->m_full,k);
	}
	else
	    for(i = 0; i < size; ++i)
		Mk->m_full[i] *= k;	
    }
    return ERROR_OK;
}

/** 
 * Divides matrix m by scalar value k.
 * If only one matrix is supplied, will multiply in place.
 *
 * If in place and matrix is 3x3, a macro is used (+efficiency)
 * 
 * @param Mk Result will be returned here.
 * @param M  NULL for in place multiplicacion. Otherwise, M will remain
 * unmodified, and the anwser will be Mk = M*k
 * @param k 
 * 
 * @return error code
 */
int uquad_mat_scalar_div(uquad_mat_t *Mk, uquad_mat_t *M, double k)
{
    int retval;
    if(k == 0.0)
    {
	err_check(ERROR_MATH_DIV_0,"Cannot divide by 0!");
    }
    retval = uquad_mat_scalar_mul(Mk, M,1.0/k);
    err_propagate(retval);
    return ERROR_OK;
}

/** 
 * Performs:
 *   C = A-B
 * 
 * Checks dimensions before performing operation.
 * Memory for C,A and B must have been previously allocated.
 *
 * If matrices are 3x3, a macro is used (+ efficiency).
 *
 * @param C Result.
 * @param A First operand.
 * @param B Second operand.
 * 
 * @return Error code.
 */
int uquad_mat_sub(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B)
{
    if(C == NULL || A == NULL || B == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    if((C->r != B->r) ||
       (C->c != B->c) ||
       (C->r != A->r) ||
       (C->c != A->c))
    {
	err_check(ERROR_MATH_MAT_DIM,"Dimension mismatch, cannot sub");
    }
    if((C->r == 3) && (C->c == 3))
    {
	Subtract_Matrices_3x3(C->m_full,A->m_full,B->m_full);
    }
    else
    {
	Subtract_Matrices(C->m_full,A->m_full,B->m_full,A->r,A->c);
    }
    return ERROR_OK;
}

/** 
 * Performs:
 *   C = A+B
 * 
 * Checks dimensions before performing operation.
 * Memory for C,A and B must have been previously allocated.
 *
 * If matrices are 3x3, a macro is used (+ efficient).
 *
 * @param C Result.
 * @param A First operand.
 * @param B Second operand.
 * 
 * @return Error code.
 */
int uquad_mat_add(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B)
{
    if(C == NULL || A == NULL || B == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    if((C->r != B->r) ||
       (C->c != B->c) ||
       (C->r != A->r) ||
       (C->c != A->c))
    {
	err_check(ERROR_MATH_MAT_DIM,"Dimension mismatch, cannot add");
    }
    if((C->r == 3) && (C->c == 3))
    {
	Add_Matrices_3x3(C->m_full,A->m_full,B->m_full);
    }
    else
    {
	Add_Matrices(C->m_full,A->m_full,B->m_full,A->r,A->c);
    }
    return ERROR_OK;
}

/** 
 * Solves linear system using gaussian elimination:
 *   A*x = B
 * 
 * Checks dimensions before operating.
 * Requires aux matrix to build [A:B]. If none is supplied, will
 * allocate required memory, and then free it.
 *
 * @param A 
 * @param B 
 * @param x 
 * @param maux NULL or aux memory for [A:B], content destroyed.
 * 
 * @return Error code.
 */
int uquad_solve_lin(uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *x, uquad_mat_t *maux)
{
    int retval;
    uquad_bool_t local_mem = false;
    if(A == NULL || B == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }

    if(A->c != x->r ||
       B->r != A->r ||
       B->c != x->c)
    {
	err_check(ERROR_MATH_MAT_DIM,"Dimension mismatch, cannot solve");
    }

    if(maux == NULL)
    {
	// create aux matrix
	maux = uquad_mat_alloc(A->r,A->c + B->c);
	if(maux == NULL)
	{
	    err_check(ERROR_MALLOC,"Could not allocate aux mem for lin solve.");
	}
	local_mem = true;
    }

#if USE_EQUILIBRATE
    uquad_mat_t *R, *C, *Beq;
    R = uquad_mat_alloc(A->r, 1);
    C = uquad_mat_alloc(A->c, 1);
    Beq = uquad_mat_alloc(B->r,B->c);
    if (R == NULL || C == NULL || Beq == NULL)
    {
	err_log("Could not allocate aux mem for lin solve.");
	goto cleanup;
    }

    retval = uquad_mat_copy(Beq, B);
    if(retval != ERROR_OK)
    {
	err_log_num("Failed to copy B.",retval);
	goto cleanup;
    }

    // equilibrate matrices to reduce condition number
#warning "Equilibrate will break matrix A!"
    retval = Equilibrate_Matrix(A->m_full, A->r, A->c, R->m_full, C->m_full);
    if(retval != ERROR_OK)
    {
	err_log_num("Equilibrate_Matrix() failed!",retval);
	goto cleanup;
    }

    retval = Equilibrate_Right_Hand_Side(Beq->m_full, R->m_full, R->r);
    if(retval != ERROR_OK)
    {
	err_log_num("Equilibrate_Right_Hand_Side() failed!",retval);
	goto cleanup;
    }
    // we need [A:B]
    Join_Matrices_by_Row(maux->m_full,A->m_full,A->r,A->c,Beq->m_full,Beq->c);
#else
    // we need [A:B]
    Join_Matrices_by_Row(maux->m_full,A->m_full,A->r,A->c,B->m_full,B->c);
#endif

    // find inv
    retval = Gaussian_Elimination_Aux(maux->m_full,maux->r,maux->c);
    if (retval < 0)
    {
	err_log_num("Gaussian elimination failed, matrix is singular",retval);
	goto cleanup;
    }

    // prepare return datax
    Get_Submatrix(x->m_full, x->r, x->c,
		  maux->m_full, maux->c,
		  0, A->c);

    // unequilibrate solution
#if USE_EQUILIBRATE
    retval = Unequilibrate_Solution(x->m_full, C->m_full, C->r);
    if( retval != ERROR_OK)
    {
	err_log_num("Unequilibrate_Solution() failed!",retval);
	goto cleanup;
    }
    uquad_mat_free(R);
    uquad_mat_free(C);
    uquad_mat_free(Beq);
#endif
	
    cleanup:
    // cleanup
    if(local_mem)
	// free tmp memory
	uquad_mat_free(maux);
    
    return ERROR_OK;
}

/** 
 * Sets matrix to identity.
 * Assumes memory was previously allocated.
 * 
 * @param m 
 * 
 * @return Error code
 */
int uquad_mat_eye(uquad_mat_t *m)
{
    if(m == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    if(m->r != m->c)
    {
	err_check(ERROR_MATH_MAT_DIM,"Identity matrix must be square!");
    }
    Identity_Matrix(m->m_full,m->r);
    return ERROR_OK;
}

/** 
 * Fills a matrix with zeros.
 * 
 * @param m 
 * 
 * @return error code.
 */
int uquad_mat_zeros(uquad_mat_t *m)
{
    if(m == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    Zero_Matrix(m->m_full, m->r, m->c);
    return ERROR_OK;
}

/** 
 * Copies part of a matrix A to a smaller matrix, S.
 * 
 * @param S answer is returned here
 * @param r first row of A that will be copied to S
 * @param c first column of A that will be copied to S
 * @param A src matrix
 * 
 * @return 
 */
int uquad_mat_get_subm(uquad_mat_t *S, int r, int c, uquad_mat_t *A)
{
    if(S == NULL || A == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    if(A->r < r + S->r || A->c < c + S->c)
    {
	err_check(ERROR_MATH_MAT_DIM,"Cannot extract submatrix, too big!.");
    }

    Get_Submatrix(S->m_full, S->r, S->c,
		  A->m_full, A->c, r, c);
    return ERROR_OK;
}

/** 
 * Sets part of a matrix A to match a smaller matrix, S.
 * 
 * @param A answer is returned here.
 * @param r first row of A where S will be copied to.
 * @param c first column of A where S will be copied to.
 * @param S submatrix
 * 
 * @return 
 */
int uquad_mat_set_subm(uquad_mat_t *A, int r, int c, uquad_mat_t *S)
{
    if(A == NULL || S == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    if(A->r - r < S->r || A->c -c < S->c)
    {
	err_check(ERROR_MATH_MAT_DIM,"Cannot set submatrix, too big!.");
    }

    Set_Submatrix(A->m_full, A->c,
		  S->m_full, S->r, S->c, r, c);
    return ERROR_OK;
}

/** 
 * Copy matrix src to dest.
 * Assumes memory has been previously allocated for dest.
 * 
 * @param dest 
 * @param src 
 * 
 * @return error code.
 */
int uquad_mat_copy(uquad_mat_t *dest, uquad_mat_t *src)
{
    int retval = uquad_mat_get_subm(dest, 0, 0, src);
    err_propagate(retval);
    return retval;
}

/** 
 * Creates a diagonal matrix, setting diagonal of m to diag, and the
 * rest to zeros.
 * Assumes length of diag matches length of diagonal.
 * 
 * @param m 
 * @param diag 
 * 
 * @return 
 */
int uquad_mat_diag(uquad_mat_t *m, double *diag)
{
    int retval;
    if(m == NULL)
    {
	err_check(ERROR_MALLOC,"Could not allocate aux mem for inv.");
    }    
    retval = uquad_mat_eye(m);
    err_propagate(retval);
    Set_Diagonal(m->m_full, diag, m->r, m->c);
    return ERROR_OK;
}

int uquad_mat_get_diag(double v[],uquad_mat_t *m,int n)
{
    if(m == NULL)
    {
	err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
    }
    if(m->r != m->c)
    {
	err_check(ERROR_MATH_MAT_DIM, "Must be square!");
    }
    Get_Diagonal(v,m->m_full,n,n);
    return ERROR_OK;
}
/** 
 * Inverts matrix.
 * Assumes memory was previously allocated for minv.
 * Requires aux memory, two matrices. If not supplied, will allocate and
 * free after finishing.
 * 
 * @param Minv Input.
 * @param M Result.
 * @param Meye NULL or auxiliary matrix, size of M.
 * @param Maux NULL or auxiliary matrix, size of [M:M]
 * 
 * @return Error code.
 */
int uquad_mat_inv(uquad_mat_t *Minv, uquad_mat_t *M, uquad_mat_t *Meye, uquad_mat_t *Maux)
{
    int retval;
    uquad_bool_t local_Meye = false, local_Maux = false;
    if(M == NULL || Minv == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }

    if(M->r != M->c)
    {
	err_check(ERROR_MATH_MAT_DIM,"Cannot invert non-square matrix!");
    }

    if(Meye == NULL)
    {
	Meye= uquad_mat_alloc(M->r,M->c);
	if(Meye == NULL)
	{
	    err_log("Could not allocate aux mem for inv.");
	    retval = ERROR_MALLOC;
	    goto cleanup;
	}
	uquad_mat_eye(Meye);
	local_Meye = true;
    }

    if(Maux == NULL)
    {
	Maux = uquad_mat_alloc(M->r,(M->c)<<1);
	if(Maux == NULL)
	{
	    err_log("Could not allocate aux mem for inv.");
	    retval = ERROR_MALLOC;
	    goto cleanup;
	}
	local_Maux = true;
    }

    retval = uquad_solve_lin(M, Meye, Minv, Maux);
    if(retval != ERROR_OK)
    {
	goto cleanup;
    }

    cleanup:
    if(local_Meye)
	uquad_mat_free(Meye);
    if(local_Maux)
	uquad_mat_free(Maux);

    return retval;
}

/** 
 * Transposes a matrix
 * 
 * @param Mt transpose of M
 * @param M input
 * 
 * @return error code
 */
int uquad_mat_transpose(uquad_mat_t *Mt, uquad_mat_t *M)
{
    if(Mt == NULL || M == NULL)
    {
	err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
    }
    if(Mt->r != M->c || Mt->c != M->r)
    {
	err_check(ERROR_MATH_MAT_DIM,"Matrices must be mxn and nxm!");
    }
    Transpose_Matrix(Mt->m_full, M->m_full, M->r, M->c);
    return ERROR_OK;
}

/** 
 * Transposes a matrix, in place.
 * 
 * @param m is the input, and will be transposed after execution.
 * 
 * @return error code
 */
int uquad_mat_transpose_inplace(uquad_mat_t *m)
{
    if(m == NULL)
    {
	err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
    }
    if(m->r != m->c)
    {
	err_check(ERROR_MATH_MAT_DIM, "Must be square!");
    }
    Transpose_Square_Matrix(m->m_full, m->r);
    return ERROR_OK;
}

/** 
 * Performs element to element product of matrices:
 *  C[i][j] = A[i][j]*B[i][j] for all i,j
 * If called with A==NULL and B==NULL, will square each
 * element of C, in place (C will be modified after execution)
 * 
 * @param C answer
 * @param A input or NULL
 * @param B input or NULL
 * 
 * @return error code
 */
int uquad_mat_dot_product(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B)
{
    int i;
    if(C == NULL)
    {
	err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
    }
    if(A == NULL && B == NULL)
	for(i = 0; i < C->r*C->c; ++i)
	    C->m_full[i] *= C->m_full[i];
    else
    {
	if (A == NULL || B == NULL)
	{
	    err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
	}
	if (C->r != A->r ||
	    C->c != A->c ||
	    C->r != B->r ||
	    C->c != B->c)
	{
	    err_check(ERROR_MATH_MAT_DIM, "Must same size!");
	}
	for(i = 0; i < C->r*C->c; ++i)
	    C->m_full[i] = B->m_full[i] * A->m_full[i];
    }
    return ERROR_OK;
}



int uquad_mat_exp(uquad_mat_t *expA, uquad_mat_t *A)
{
    if(A == NULL)
    {
	err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
    }
    if(A->r !=A->c)
    {
	err_check(ERROR_MATH_MAT_DIM,"A is not a square matrix");
    }

    int i=1;
    uquad_mat_t *aux0 = NULL;
    uquad_mat_t *aux1 = NULL;
    uquad_mat_t *aux2 = NULL;
    double factor=1;
    int n=A->r;
    double norm = 1;
    
    aux0 = uquad_mat_alloc(n,n);   //aux0 is used to compute A^k in every step
    aux1 = uquad_mat_alloc(n,n);   //aux1 is used to compare the exponential matrix in k                                       step with the exponential matrix in the k+1 step
    aux2 = uquad_mat_alloc(n,n);   //aux2 is for general purpose;


    uquad_mat_eye(expA);
    uquad_mat_eye(aux0);
    //    uquad_mat_copy(aux0,A);
    //uquad_mat_sub(aux1,aux0,expA);
    //    uquad_mat_zeros(aux1
    //norm = uquad_mat_norm(aux1);


    while (norm > 1e-5)
	{
	    //Set aux1=expA 
	    uquad_mat_copy(aux1,expA); // save the value of expA in k step
	    
	    //Performs A^k
	    uquad_mat_prod(aux2,aux0,A);
	    uquad_mat_copy(aux0,aux2);
	    
	    //Adds A^k/k! to expA
	    uquad_mat_scalar_div(aux2,aux2,factor);
	    uquad_mat_add(expA,expA,aux2);
	    
	    i++;
	    factor*=i;
	    
	    //Performs the norm of expAk-expA(k+1)
	    uquad_mat_sub(aux2,expA,aux1);
	    norm =  uquad_mat_norm(aux2);
	}
    
   
    //Cleaning
    
    uquad_mat_free(aux0);
    uquad_mat_free(aux1);
    uquad_mat_free(aux2);      
    

    return ERROR_OK;
}


/**Performs the euclidean norm of a matrix
 **sqrt(sum(a[0]*a[0]+a[1]*a[1]...)
 */
double uquad_mat_norm(uquad_mat_t *A)
{
    
    if(A == NULL)
    {
	err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
    }
    int i;
    int n = A->c;
    double norm=0;
    for (i=0;i<n*n;i++)
    {
	norm+=A->m_full[i]*A->m_full[i];
    }

    norm=sqrt(norm);
    return norm;
}

/* Performs the integral of A^t for ti<t<tf with an integration step of step. */
/* The result is returned on B matrix */
int uquad_mat_int(uquad_mat_t *B, uquad_mat_t *A, double ti, double tf, double step)
{
   
    int i;
    double t=ti;
    uquad_mat_t* aux0;
    
    uquad_mat_zeros(B);
    aux0 = uquad_mat_alloc(A->r,A->c);

    while(t<tf)
    {
	for(i=1;i<A->r*A->c;i++)
	{
	    aux0->m_full[i] = exp(t*log(A->m_full[i]));
	}
	t+=step;
	uquad_mat_add(B,B,aux0);
	uquad_mat_scalar_mul(B,B,step);
    }

    uquad_mat_free(aux0);
    return ERROR_OK;
}

/**
 * Builds a rotation matrix R from phi,psi,theta, and performs Vr = R*V
 *
 *   R_psi:
 *   +-                        -+
 *   |  1,     0,         0     |
 *   |                          |
 *   |  0,  cos(psi), sin(psi)  |
 *   |                          |
 *   |  0, -sin(psi), cos(psi)  |
 *   +-                        -+
 *
 *   R_phi:
 *   +-                        -+
 *   |  cos(phi), 0, -sin(phi)  |
 *   |                          |
 *   |      0,    1,     0      |
 *   |                          |
 *   |  sin(phi), 0,  cos(phi)  |
 *   +-                        -+
 *
 *   R_theta:
 *   +-                            -+
 *   |   cos(theta), sin(theta), 0  |
 *   |                              |
 *   |  -sin(theta), cos(theta), 0  |
 *   |                              |
 *   |       0,           0,     1  |
 *   +-                            -+
 *
 *   R = R_psi*R_phi*R_theta:
 *   +-                                                                                                                           -+
 *   |                  cos(phi) cos(theta),                                cos(phi) sin(theta),                    -sin(phi)      |
 *   |                                                                                                                             |
 *   |  cos(theta) sin(phi) sin(psi) - cos(psi) sin(theta), cos(psi) cos(theta) + sin(phi) sin(psi) sin(theta), cos(phi) sin(psi)  |
 *   |                                                                                                                             |
 *   |  sin(psi) sin(theta) + cos(psi) cos(theta) sin(phi), cos(psi) sin(phi) sin(theta) - cos(theta) sin(psi), cos(phi) cos(psi)  |
 *   +-                                                                                                                           -+
 *
 * @param Vr
 * @param V
 * @param phi
 * @param psi
 * @param theta
 *
 * @return
 */
int uquad_mat_rotate(uquad_mat_t *Vr, uquad_mat_t *V, double phi, double psi, double theta)
{
    int retval;
    uquad_mat_t *R = uquad_mat_alloc(3,3);
    if(R == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate rotation matrix");
    }

    R->m[0][0] = cos(phi)*cos(theta);
    R->m[0][1] = cos(phi)*sin(theta);
    R->m[0][2] = -sin(phi);

    R->m[1][0] = cos(theta)*sin(psi)*sin(phi) - cos(psi)*sin(theta);
    R->m[1][1] = cos(psi)*cos(theta) + sin(psi)*sin(phi)*sin(theta);
    R->m[1][2] = cos(phi)*sin(psi);

    R->m[2][0] = sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi);
    R->m[2][1] = cos(psi)*sin(phi)*sin(theta) - cos(theta)*sin(psi);
    R->m[2][2] = cos(psi)*cos(phi);

    retval = uquad_mat_prod(Vr,R,V);
    cleanup_log_if(retval,"Failed to rotate vector!");

    cleanup:
    uquad_mat_free(R);
    return retval;
}

/** 
 * Will load matrix from text file.
 * Will ignore spaces, end of lines, tabs, etc.
 * Assumes memory has been previously allocated for matrix.
 * 
 * Example of a configuration file for a 7x3 matrix:
 *   26.9552751514508        0                       0
 *   0                       27.1044248763245        0
 *   0                       0                       25.9160276670631
 *   
 *   1                       0.00953542215888624       0.00955723285532017
 *   -0.000376784971686033   1                         -0.00677033632701969
 *   0.0105047129596497      -0.00456172876850397      1
 *   
 *   13.0716873433579
 *
 *   -1.07951072069862
 *   -40.723583688652
 * 
 * Note that elements are read row-wise (fills row, then next row, etc)
 *
 * @param imu 
 * @param input stream to load from, or NULL for stdin
 * 
 * @return 
 */
int uquad_mat_load(uquad_mat_t *m, FILE *input)
{
    int i;
    double dtmp;
    if(m == NULL)
    {
	err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
    }
    if(input == NULL)
	input = stdin;
    for(i=0; i < m->r*m->c; i++)
    {
	if(fscanf(input,"%lf",&dtmp) <= 0)
	{
	    err_check(ERROR_READ, "Failed to load data!");
	}
	m->m_full[i] = dtmp;
    }
    if(i < m->r*m->c)
    {
	err_check(ERROR_IO,"Failed to load matrix");
    }
    return ERROR_OK;
}

/** 
 * Prints matrix to any output.
 * 
 * @param m 
 * @param output
 */
void uquad_mat_dump(uquad_mat_t *m, FILE *output)
{
    int i,j;
    if(m == NULL)
    {
	err_log("Cannot dump.");
    }
    if(output == NULL)
	output = stdout;
    for(i=0;i<m->r;i++)
    {
	for(j=0;j<m->c;j++)
	    fprintf(output, "%0.15f\t",m->m[i][j]);
	fprintf(output,"\n");
    }

}

/** 
 * Allocates matrix of size r rows and c columns.
 * 
 * @param r Rows.
 * @param c Columns.
 * 
 * @return NULL or pointer to allocated memory.
 */
uquad_mat_t *uquad_mat_alloc(int r, int c)
{
    int i;
    if(!((r < UQUAD_MAT_MAX_DIM) && (c < UQUAD_MAT_MAX_DIM)))
    {
	err_log("Invalid matrix size!");
	return NULL;
    }
    uquad_mat_t *m;
    m = (uquad_mat_t *)malloc(sizeof(struct uquad_mat));
    mem_alloc_check(m);
    m->r = r;
    m->c = c;
    m->m = (double **)malloc(sizeof(double *)*m->r);
    mem_alloc_check(m->m);
    // consecutive data
    m->m_full = (double*)malloc(sizeof(double)*m->r*m->c);        
    for (i=1, m->m[0] = m->m_full; i < m->r; ++i)
    {
	m->m[i] = m->m[i-1] + m->c;
    }
    return m;
}

/** 
 * Free memory allocated my uquad_mat_alloc()
 * Will check if NULL argument is supplied.
 * 
 * @param m 
 */
void uquad_mat_free(uquad_mat_t *m)
{
    if(m == NULL)
	return;
    free(m->m_full);
    free(m->m);
    free(m);
}
