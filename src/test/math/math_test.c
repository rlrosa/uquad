#include <uquad_aux_math.h>
#include <stdlib.h>

#define wait_for_enter while(fread(tmp,1,1,stdin) == 0)

enum test_type{
    VECTOR_DOT = 0,
    VECTOR_CROSS,
    MATRIX_PROD,
    MATRIX_DET,
    MATRIX_INV,
    LIN_SOLVE,
    TEST_COUNT
};

uquad_mat_t *m1, *m2, *mr;
uquad_vec_t *v1, *v2, *vr;
int retval;
int r1,c1,r2,c2,l,i,j;
float tmp;

void matrix_stdin(uquad_mat_t *m)
{
    printf("Enter rows and columns of matrix, row wise:\n");
    for(i=0; i< m->r; i++)
	for(j=0; j < m->c; j++)
	{
	    scanf("%f",&tmp);
	    m->m[i][j] = (double)tmp;
	}
}

int alloc_m1_m2(void)
{
    printf("Enter number of rows and columns of first matrix (less than %d)\n",
	   UQUAD_MAT_MAX_DIM);
    scanf("%d%d",&r1,&c1);
    printf("Enter number of rows and columns of second matrix (less than %d)\n",
	   UQUAD_MAT_MAX_DIM);
    scanf("%d%d",&r2,&c2);

    m1 = uquad_mat_alloc(r1,c1);
    m2 = uquad_mat_alloc(r2,c2);
    if(m1 == NULL || m2 == NULL)
	return ERROR_FAIL;
    else
	return ERROR_OK;
}

int load_m1_m2(void)
{
    printf("First matrix \n");
    matrix_stdin(m1);
    printf("First Matrix is :\n");
    uquad_mat_dump(m1,NULL);

    printf("Second matrix \n");
    matrix_stdin(m2);
    printf("Second Matrix is:\n");
    uquad_mat_dump(m2,NULL);

    return ERROR_OK;
}

int vector_dot_test(void)
{
    printf("Enter vector length:(less than %d)\n",
	   UQUAD_MAT_MAX_DIM);
    scanf("%d",&l);
    v1 = uquad_vec_alloc(l);
    v2 = uquad_vec_alloc(l);
    vr = uquad_vec_alloc(l);

}

int lin_solve(void)
{
    if(alloc_m1_m2() != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    mr = uquad_mat_alloc(m1->c,m2->c);
    if(mr == NULL)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    printf("Will load A and B to solve Ax=B\n");
    if(load_m1_m2() != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Failed loading data");
    }

    retval = uquad_solve_lin(m1,m2,mr,NULL);
    err_propagate(retval);

    printf("Solution x of Ax=B:\n");
    uquad_mat_dump(mr,NULL);
}

int matrix_prod_test(void)
{
    if(alloc_m1_m2() != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    mr = uquad_mat_alloc(m1->r,m2->c);
    if(mr == NULL)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    if(m2->r==m1->c)
    {
	if(load_m1_m2() != ERROR_OK)
	{
	    err_check(ERROR_FAIL,"Failed loading data");
	}

	retval = uquad_mat_prod(m1,m2,mr);
	err_propagate(retval);

        printf("Multiplication of the Matrices:\n");
	uquad_mat_dump(mr,NULL);
    }
}

int main(int argc, char *argv[]){
    int retval = ERROR_OK;
    enum test_type sel_test;
    int cmd;
    printf("Select test:\n\t%d:Vector dot\n\t%d:Vector cross\n\t%d:Matrix product\n\t%d:Matrix det\n\t%d:Matrix inv\n\t%d:Linear system\n",
	   VECTOR_DOT,
	   VECTOR_CROSS,
	   MATRIX_PROD,
	   MATRIX_DET,
	   MATRIX_INV,
	   LIN_SOLVE,
	   TEST_COUNT);
    scanf("%d",&cmd);
    if(cmd<0 || cmd > TEST_COUNT)
    {
	err_check(ERROR_FAIL,"Invalid input.");
    }
    sel_test = cmd;
    switch(sel_test)
    {
    case VECTOR_DOT:
	retval = vector_dot_test();
	break;
    case MATRIX_PROD:
	retval = matrix_prod_test();
	break;
    case LIN_SOLVE:
	retval = lin_solve();
	break;
    default:
	err_check(ERROR_FAIL,"This shouldn't happen.");
	break;
    }

    uquad_mat_free(m1);
    uquad_mat_free(m2);
    uquad_mat_free(mr);

    uquad_vec_free(v1);
    uquad_vec_free(v2);
    uquad_vec_free(vr);

    return retval;
}
