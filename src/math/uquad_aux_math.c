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
#include "get_submatrix.c"
#include "set_submatrix.c"
#include "multiply_matrices.c"
#include "multiply_matrices_3x3.c"
#include "gauss_elimination.c"
#include "gauss_aux_elimination.c"
//#include "doolittle.c"
//#include "doolittle_pivot.c"

#define uquad_cp_mat_A2B(A,B) Copy_Matrix(A->m_full,B->m_full,A->r,A->c)

/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Vector
 * -- -- -- -- -- -- -- -- -- -- -- --
 */
int uquad_vec_dot(uquad_vec_t *v1, uquad_vec_t *v2, uquad_vec_t *vr)
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

int uquad_vec_cross(uquad_vec_t *v1, uquad_vec_t *v2, uquad_vec_t *vr)
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
int uquad_mat_prod(uquad_mat_t *m1,uquad_mat_t *m2,uquad_mat_t *mr)
{
    int i_r,i_c,i_t, retval = ERROR_OK;
    double tmp;
    if(m1 == NULL || m2 == NULL || mr == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }

    // check dims
    if((m1->c != m2->r) || !((mr->r == m1->r) && (mr->c == m2->c)))
    {
	err_check(ERROR_MATH_MAT_DIM,"Cannot multiply matrices, dims do not match.");
    }
    if(mr->c == 3 && mr->r == 3)
	Multiply_Matrices_3x3(mr->m_full,m1->m_full,m2->m_full);
    else
	Multiply_Matrices(mr->m_full,m1->m_full,mr->r,mr->c,m2->m_full,m2->c);
    return ERROR_OK;
}

int uquad_mat_det(uquad_mat_t *m, double *res)
{
    //TODO
    err_check(ERROR_FAIL,"Not implemented!");
}

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
    printf("\n");
    uquad_mat_dump(maux,stdout);
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

int uquad_mat_inv(uquad_mat_t *m1, uquad_mat_t *minv, uquad_mat_t *maux)
{
    uquad_mat_t *meye;
    int retval;
    if(m1 == NULL || minv == NULL)
    {
	err_check(ERROR_NULL_POINTER,"NULL pointer is invalid arg.");
    }

    meye = uquad_mat_alloc(m1->r,m1->c);
    if(meye == NULL)
    {
	err_check(ERROR_MALLOC,"Could not allocate aux mem for inv.");
    }
    Identity_Matrix(meye->m_full,meye->r);

    retval = uquad_solve_lin(m1, meye, minv, NULL);
    err_propagate(retval);

    return ERROR_OK;
}

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

void uquad_mat_free(uquad_mat_t *m)
{
    int i;
    if(m == NULL)
	return;
    free(m->m_full);
    free(m->m);
    free(m);
}
