#include "uquad_aux_math.h"
#include <stdlib.h>
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

    v->v = (float *)malloc(sizeof(float)*v->l);
    mem_alloc_check(v->v);

    return ERROR_OK;
}

int uquad_vec_free(uquad_vec_t *v)
{
    free(v->v);
    free(v);
    return ERROR_OK;
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

    // rows
    for(i_r=0; i_r < mr->r; ++i_r)
    {
	// cols
	for(i_c=0; i_c < mr->c; ++i_c)
	{
	    tmp = 0;
	    for(i_t=0; i_t < mr->c; ++i_t)
		tmp += m1->m[i_r][i_t]*m2->m[i_t][i_c];
	    mr->m[i_r][i_c] = tmp;
	}
    }
    return ERROR_OK;
}

int uquad_mat_det(uquad_mat_t *m, float *res)
{
    //TODO
    err_check(ERROR_FAIL,"Not implemented!");
}

int uquad_mat_inv(uquad_mat_t *m1, uquad_mat_t *mr)
{
    //TODO
    err_check(ERROR_FAIL,"Not implemented!");
}

uquad_mat_t *uquad_mat_alloc(int r, int c)
{
    int i;
    float *tmp;
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
    m->m = (float **)malloc(sizeof(float *)*m->r);
    mem_alloc_check(m->m);
    // consecutive data
    m->m_full = (float*)malloc(sizeof(float)*m->r*m->c);        
    for (i=1, m->m[0] = m->m_full; i < m->r; ++i)
    {
	m->m[i] = m->m[i-1] + m->c;
    }
    return m;
}

int uquad_mat_free(uquad_mat_t *m)
{
    int i;
    if(m == NULL)
	return ERROR_OK;
    free(m->m_full);
    free(m->m);
    free(m);
    return ERROR_OK;
}
