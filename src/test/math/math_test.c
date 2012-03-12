#include <uquad_aux_math.h>
#include <stdlib.h>

#define wait_for_enter while(fread(tmp,1,1,stdin) == 0)

enum test_type{
    MATRIX_PROD = 0,
    MATRIX_DET,
    MATRIX_INV,
    MATRIX_ADD,
    MATRIX_SUBS,
    MATRIX_MUL_K,
    MATRIX_DIV_K,
    LIN_SOLVE,
    TEST_COUNT
};

uquad_mat_t *m1, *m2, *mr;
int retval;
int r1,c1,r2,c2,l,i,j;
float tmp;
double k;

void matrix_stdin(uquad_mat_t *m)
{
    printf("Enter rows and columns of matrix, row wise:\n");    
    retval = uquad_mat_load(m,stdin);
    if(retval != ERROR_OK)
    {
	err_log("WARN:Could not load matrix from stdin");
    }
}

static int alloc_counter = 1;
int alloc_m(uquad_mat_t **m)
{
    printf("Enter number of rows and columns of matrix %d (less than %d)\n",
	   alloc_counter++,
	   UQUAD_MAT_MAX_DIM);
    scanf("%d%d",&r1,&c1);
    *m = uquad_mat_alloc(r1,c1);
    if(*m == NULL)
	return ERROR_FAIL;
    else
	return ERROR_OK;
}   

int alloc_m1_m2(void)
{
    alloc_m(&m1);
    alloc_m(&m2);
}

static int counter = 1;
int load_m(uquad_mat_t *m)
{
    printf("Matrix %d\n",counter);
    matrix_stdin(m);
    printf("Matrix %d is :\n",counter);
    uquad_mat_dump(m,NULL);
    counter++;
    return ERROR_OK;
}

int load_m1_m2(void)
{
    retval = load_m(m1);
    err_propagate(retval);
    retval = load_m(m2);
    err_propagate(retval);
    return ERROR_OK;
}

int mat_inv_test(void)
{
    if(alloc_m(&m1) != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    mr = uquad_mat_alloc(m1->r,m1->c);
    if(mr == NULL)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    printf("Will load A to solve inv(A)\n");
    if(load_m(m1) != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Failed loading data");
    }

    retval = uquad_mat_inv(m1,mr,NULL,NULL);
    err_propagate(retval);

    printf("inv(A):\n");
    uquad_mat_dump(mr,NULL);
}

int matrix_add_sub_test(uquad_bool_t add_notsub)
{
    if(alloc_m1_m2() != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    mr = uquad_mat_alloc(m1->r,m1->c);
    if(mr == NULL)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    if(load_m1_m2() != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Failed loading data");
    }

    if(add_notsub)
    {
	printf("Will add matrices\n");
	retval = uquad_mat_add(mr,m1,m2);
    }
    else
    {
	printf("Will sub matrices\n");
	retval = uquad_mat_sub(mr,m1,m2);
    }
    err_propagate(retval);

    printf("Result:\n");
    uquad_mat_dump(mr,NULL);
}

int mat_scalar_test(uquad_bool_t mul_notdiv)
{
    if(alloc_m(&m1) != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Could not allocate mem, cannot continue");
    }

    printf("Will load A to solve inv(A)\n");
    if(load_m(m1) != ERROR_OK)
    {
	err_check(ERROR_FAIL,"Failed loading data");
    }

    printf("Scalar:\n");
    scanf("%f",&tmp);
    k = (double)tmp;
    if(mul_notdiv)
    {
	printf("Will multiply by k=%f\n",k);
	retval = uquad_mat_scalar_mul(m1,NULL,k);
    }
    else
    {
	printf("Will div by k=%f\n",k);
	retval = uquad_mat_scalar_div(m1,NULL,k);
    }
    err_propagate(retval);

    printf("A*k:\n");
    uquad_mat_dump(m1,NULL);
    return ERROR_OK;
}

int lin_solve_test(void)
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
    printf("Select test:\n\t%d:Matrix product\n\t%d:Matrix det\n\t%d:Matrix inv\n\t%d:Matrix add\n\t%d:Matrix sub\n\t%d:Matrix mul k\n\t%d:Matrix div k\n\t%d:Linear system\n",
	   MATRIX_PROD,
	   MATRIX_DET,
	   MATRIX_INV,
	   MATRIX_ADD,
	   MATRIX_SUBS,
	   MATRIX_MUL_K,
	   MATRIX_DIV_K,
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
    case MATRIX_PROD:
	retval = matrix_prod_test();
	break;
    case MATRIX_INV:
	retval = mat_inv_test();
	break;
    case MATRIX_ADD:
	retval = matrix_add_sub_test(true);
	break;
    case MATRIX_SUBS:
	retval = matrix_add_sub_test(false);
	break;
    case MATRIX_MUL_K:
	retval = mat_scalar_test(true);
	break;
    case MATRIX_DIV_K:
	retval = mat_scalar_test(false);
	break;
    case LIN_SOLVE:
	retval = lin_solve_test();
	break;
    default:
	err_check(ERROR_FAIL,"This shouldn't happen.");
	break;
    }

    uquad_mat_free(m1);
    uquad_mat_free(m2);
    uquad_mat_free(mr);

    return retval;
}
