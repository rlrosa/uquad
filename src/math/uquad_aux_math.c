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
//#include "equilibrate_matrix.c"//TODO use?
#include "zero_matrix.c"
#include "identity_matrix.c"
#include "set_diagonal.c"
#include "get_diagonal.c"
#include "get_submatrix.c"
#include "set_submatrix.c"
#include "div_matrix_by_scalar.c"
#include "div_3x3_matrix_by_scalar.h"
#include "mul_matrix_by_scalar.c"
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
//#include "doolittle.c"
//#include "doolittle_pivot.c"

/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Vector
 * -- -- -- -- -- -- -- -- -- -- -- --
 */

/** 
 * Performs dot product of vectors.
 *
 * Will validate check dimensions.
 * 
 * @param vr Answer.
 * @param v1 
 * @param v2 
 * 
 * @return 
 */
int uquad_vec_dot(uquad_vec_t *vr, uquad_vec_t *v1, uquad_vec_t *v2)
{
    int i;
    if(v1 == NULL || v2 == NULL || vr == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    if(v1->l != v2->l || v1->l != vr->l)
    {
	err_check(ERROR_MATH_VEC_LEN,"Vectors must be of the same length!");
    }
    for(i=0; i < v1->l; ++i)
	vr->v[i] = v1->v[i]*v2->v[i];
    return ERROR_OK;
}

/** 
 * Performs cross product of vectors.
 *
 * Will validate check dimensions.
 * 
 * @param vr Answer.
 * @param v1 
 * @param v2 
 * 
 * @return 
 */
int uquad_vec_cross(uquad_vec_t *vr, uquad_vec_t *v1, uquad_vec_t *v2)
{
    int i;
    if(v1 == NULL || v2 == NULL || vr == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }
    if(v1->l != v2->l || v1->l != vr->l || v1->l != 3)
    {
	err_check(ERROR_MATH_VEC_LEN,"Vectors must be of length 3 for cross product!");
    }
    vr->v[0] = v1->v[1]*v2->v[2] - v1->v[2]*v2->v[1];
    vr->v[1] = v1->v[2]*v2->v[0] - v1->v[0]*v2->v[2];
    vr->v[2] = v1->v[0]*v2->v[1] - v1->v[1]*v2->v[0];
    return ERROR_OK;
}

/** 
 * Allocates memory for a vector of size l
 * 
 * @param l 
 * 
 * @return NULL or pointer to allocated memory.
 */
uquad_vec_t *uquad_vec_alloc(int l)
{
    uquad_vec_t *v;
    if(l > UQUAD_MATH_MAX_DIM)
    {
	err_log("Vector too long.");
	return NULL;
    }

    v = (uquad_vec_t *)malloc(sizeof(struct uquad_vec));
    mem_alloc_check(v);

    v->l = l;

    v->v = (double *)malloc(sizeof(double)*v->l);
    mem_alloc_check(v->v);

    return ERROR_OK;
}

void uquad_vec_free(uquad_vec_t *v)
{
    if(v == NULL)
	return;
    free(v->v);
    free(v);
}

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
    int i_r,i_c,i_t, retval = ERROR_OK;
    double tmp;
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
    err_check(ERROR_FAIL,"Not implemented!");
}

/** 
 * Divides matrix m by scalar value k.
 * Will check if attempt to divide by 0.0.
 *
 * If matrices are 3x3, a macro is used (+ efficiency).
 * 
 * @param m 
 * @param k 
 * 
 * @return 
 */
int uquad_mat_scalar_div(uquad_mat_t *m, double k)
{
    if(k == 0.0)
    {
	err_check(ERROR_MATH_DIV_0,"Cannot divide by 0!");
    }
    if(m == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }

    if((m->r == 3) && (m->c == 3))
    {
	Divide_3x3_Matrix_by_Scalar(m->m_full,k);
    }
    else
    {
	Divide_Matrix_by_Scalar(m->m_full,k,m->r,m->c);
    }
    return ERROR_OK;
}

/** 
 * Multiplies matrix m by scalar value k.
 *
 * If matrix is 3x3, a macro is used (+efficiency)
 * 
 * @param m 
 * @param k 
 * 
 * @return 
 */
int uquad_mat_scalar_mul(uquad_mat_t *m, double k)
{
    if(m == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }

    if((m->r == 3) && (m->c == 3))
    {
	Multiply_3x3_Matrix_by_Scalar(m->m_full,k);
    }
    else
    {
	Multiply_Matrix_by_Scalar(m->m_full,k,m->r,m->c);
    }
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

    // we need [A:B]
    Join_Matrices_by_Row(maux->m_full,A->m_full,A->r,A->c,B->m_full,B->c);

    // find inv
    retval = Gaussian_Elimination_Aux(maux->m_full,maux->r,maux->c);
    if (retval < 0)
    {
	err_check(ERROR_MATH_MAT_SING,"Gaussian elimination failed, matrix is singular");
    }

    // prepare return datax
    Get_Submatrix(x->m_full, x->r, x->c,
		  maux->m_full, maux->c,
		  0, A->c);

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
    int retval;
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

/** 
 * Inverts matrix.
 * Assumes memory was previously allocated for minv.
 * Requires aux memory, two matrices. If not supplied, will allocate and
 * free after finishing.
 * 
 * @param m1 Input.
 * @param minv Result.
 * @param meye NULL or auxiliary matrix, size of m1.
 * @param maux NULL or auxiliary matrix, size of [m1:m1]
 * 
 * @return Error code.
 */
int uquad_mat_inv(uquad_mat_t *m1, uquad_mat_t *minv, uquad_mat_t *meye, uquad_mat_t *maux)
{
    int retval;
    uquad_bool_t local_meye = false, local_maux = false;
    if(m1 == NULL || minv == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }

    if(m1->r != m1->c)
    {
	err_check(ERROR_MATH_MAT_DIM,"Cannot invert non-square matrix!");
    }

    if(meye == NULL)
    {
	meye= uquad_mat_alloc(m1->r,m1->c);
	if(meye == NULL)
	{
	    err_check(ERROR_MALLOC,"Could not allocate aux mem for inv.");
	}
	uquad_mat_eye(meye);
	local_meye = true;
    }

    if(maux == NULL)
    {
	maux = uquad_mat_alloc(m1->r,(m1->c)<<1);
	if(maux == NULL)
	{
	    err_check(ERROR_MALLOC,"Could not allocate aux mem for inv.");
	}
	local_maux = true;
    }

    retval = uquad_solve_lin(m1, meye, minv, maux);
    err_propagate(retval);

    if(local_meye)
	uquad_mat_free(meye);
    if(local_maux)
	uquad_mat_free(maux);

    return ERROR_OK;
}

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
    float ftmp;
    if(m == NULL)
    {
	err_check(ERROR_NULL_POINTER, "Cannot load, must allocate memory previously.");
    }
    if(input == NULL)
	input = stdin;
    for(i=0; i < m->r*m->c; i++)
    {
	fscanf(input,"%f",&ftmp);
	m->m_full[i] = (double) ftmp;
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
	    fprintf(output, "%f\t",m->m[i][j]);
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
    double *tmp;
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
    int i;
    if(m == NULL)
	return;
    free(m->m_full);
    free(m->m);
    free(m);
}
