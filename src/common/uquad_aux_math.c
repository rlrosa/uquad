#include <uquad_aux_math.h>
#include <stdlib.h>

static double op[UQUAD_MAT_MAX_DIM];
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
    m->m = (float **)malloc(sizeof(float *)*m->r);
    mem_alloc_check(m->m);
    for (i=0; i<m->r; ++i)
    {
	m->m[i] = (float *)malloc(sizeof(float)*m->c);
	mem_alloc_check(m->m[i]);
    }
    return m;
}


int uquad_mat_free(uquad_mat_t *m)
{
    free(m);
    return ERROR_OK;
}	
