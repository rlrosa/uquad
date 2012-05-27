/**
 * math_test: test program for math library
 * Copyright (C) 2012  Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file   math_test.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 10:02:32 2012
 *
 * @brief  test program for math library
 *
 */
#include <uquad_aux_math.h>
#include <macros_misc.h>
#include <stdlib.h>
#include <unistd.h>

enum test_type{
    MATRIX_PROD = 0,
    MATRIX_DET,
    MATRIX_INV,
    MATRIX_ADD,
    MATRIX_SUBS,
    MATRIX_MUL_K,
    MATRIX_DIV_K,
    LIN_SOLVE,
    MATRIX_EXP,
    ROTATE,
    POL2,
    ABS,
    TEST_COUNT
};

uquad_mat_t *m1, *m2, *mr;
int retval;
int r1,c1,r2,c2,l,i,j;
float tmp;
double k;

#define DO_CONTINUE 'y'
uquad_bool_t ask_continue()
{
    unsigned char ans;
    printf("Continue? (%c/n)\n",DO_CONTINUE);
    while(1)
    {
	ans = getchar();
	if(ans != '\n')
	    break;
    }
    return (ans == DO_CONTINUE)?
	true:
	false;
}

int matrix_stdin(uquad_mat_t *m)
{
    printf("Enter rows and columns of matrix, row wise:\n");    
    retval = uquad_mat_load(m,stdin);
    err_check(retval,"WARN:Could not load matrix from stdin");
    return retval;
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
    return ERROR_OK;
}

static int counter = 1;
int load_m(uquad_mat_t *m)
{
    int retval;
    printf("Matrix %d\n",counter);
    retval = matrix_stdin(m);
    err_propagate(retval);
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

#define print_abs(dtmp)					\
    {							\
	printf("abs(%0.4f)\t=\t%0.4f\n",dtmp,		\
	       uquad_abs(dtmp));			\
    }

int abs_test(void)
{
    printf("uquad_abs() test\n");
    print_abs(0.1);
    print_abs(-0.1);
    print_abs(1.1);
    print_abs(-1.1);
    print_abs(0.0);
    return ERROR_OK;
}

int pol2_test(void)
{
    double a,b,c,xp,xm;
    for(;;)
    {
	retval = fprintf(stdout, "a,b,c (a*x^2 + b*x + c):\n");
	if(retval < 0)
	{
	    err_log_stderr("fprintf()!");
	}
	retval = fscanf(stdin,"%lf %lf %lf", &a, &b, &c);
	if(retval < 0)
	{
	    err_log_stderr("fscanf()!");
	    err_check(ERROR_FAIL, "Fail!");
	}
	else
	{
	    if(retval < 3)
	    {
		err_log_num("Not enougth arguments!",retval);
		err_check(ERROR_INVALID_ARG, "Fail!");
	    }
	}
	retval = uquad_solve_pol2(&xp, &xm, a, b, c);
	err_propagate(retval);
	retval = fprintf(stdout, "\txp =\t%lf\n\txm =\t%lf\n\n", xp, xm);
	if(retval < 0)
	{
	    err_log_stderr("fprintf()!");
	}

	if(!ask_continue())
	    break;
    }
    return ERROR_OK;
}


int mat_inv_test(void)
{
    int retval = ERROR_OK;
    uquad_mat_t *maux1,*maux2;
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

    printf("Will use (NULL,NULL) mem:\n");
    sleep(2);
    retval = uquad_mat_inv(mr,m1,NULL,NULL);
    err_propagate(retval);

    printf("inv(A):\n");
    uquad_mat_dump(mr,NULL);

    printf("Will use (aux,NULL) mem:\n");
    sleep(2);
    maux1 = uquad_mat_alloc(mr->r,mr->c);
    maux2 = uquad_mat_alloc(mr->r,(mr->c)<<1);
    
    retval = uquad_mat_inv(mr,m1,maux1,NULL);
    cleanup_log_if(retval, "Failed (aux,NULL) test!)");
    uquad_mat_dump(mr,NULL);

    printf("Will use (NULL,aux) mem:\n");
    sleep(2);
    retval = uquad_mat_inv(mr,m1,NULL,maux2);
    cleanup_log_if(retval, "Failed (NULL,aux) test!)");
    uquad_mat_dump(mr,NULL);

    printf("Will use (aux1,aux2) mem:\n");
    sleep(2);
    retval = uquad_mat_inv(mr,m1,maux1,maux2);
    cleanup_log_if(retval, "Failed (aux1,aux2) test!)");
    uquad_mat_dump(mr,NULL);

    cleanup:
    uquad_mat_free(maux1);
    uquad_mat_free(maux2);

    return ERROR_OK;
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
    return ERROR_OK;
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
    return ERROR_OK;
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
    return ERROR_OK;
}

int mat_exp_test(void)
{

 
    //    int size = 3;
    uquad_mat_t* A = NULL;
    uquad_mat_t* expA = NULL;

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

    A=m1;
    expA=mr; 

      
    uquad_mat_exp(expA,A);
    uquad_mat_dump(expA,0);
   
    return ERROR_OK;
}

int mat_rot_test(void)
{
    int retval;

    uquad_mat_t *V    = uquad_mat_alloc(3,1);
    uquad_mat_t *Vr   = uquad_mat_alloc(3,1);
    uquad_mat_t *angs = uquad_mat_alloc(3,1);
    if(V == NULL || Vr == NULL || angs == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate mem for test!");
    }

    for(;;)
    {
	printf("Vector V:\n");
	retval = uquad_mat_load(V, NULL);
	cleanup_if(retval);

	printf("psi, phi, theta [deg]:\n");
	retval = uquad_mat_load(angs, NULL);
	cleanup_if(retval);

	retval = uquad_mat_rotate(true,
				  Vr,
				  V,
				  deg2rad(angs->m_full[0]),
				  deg2rad(angs->m_full[1]),
				  deg2rad(angs->m_full[2]),
				  NULL);
	cleanup_if(retval);

	printf("\nVr:\n\n");
	uquad_mat_dump(Vr,0);
    
	if(!ask_continue())
	    break;
    }

    cleanup:
    uquad_mat_free(V);
    uquad_mat_free(Vr);
    uquad_mat_free(angs);
    return retval;
}


int main(void){
    int retval = ERROR_OK;
    enum test_type sel_test;
    int cmd;
    printf("Select test:\n\t%d:Matrix product\n\t%d:Matrix det\n\t%d:Matrix inv\n\t%d:Matrix add\n\t%d:Matrix sub\n\t%d:Matrix mul k\n\t%d:Matrix div k\n\t%d:Linear system\n\t%d:Expnential Matrix\n\t%d:Rotate\n\t%d:Pol2\n\t%d:abs\n",
	   MATRIX_PROD,
	   MATRIX_DET,
	   MATRIX_INV,
	   MATRIX_ADD,
	   MATRIX_SUBS,
	   MATRIX_MUL_K,
	   MATRIX_DIV_K,
	   LIN_SOLVE,
	   MATRIX_EXP,
	   ROTATE,
	   POL2,
	   ABS);
    scanf("%d",&cmd);
    if(cmd<0 || cmd > TEST_COUNT)
    {
	err_check(ERROR_FAIL,"Invalid input.");
    }
    sel_test = cmd;
    printf("\n");
    printf("-- -- -- -- -- -- --");
    printf("-- -- -- -- -- -- --\tinit\t-- -- -- -- -- -- --");
    printf("-- -- -- -- -- -- --");
    printf("\n\n");
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
    case MATRIX_EXP:
	retval = mat_exp_test();
	break;
    case ROTATE:
	retval = mat_rot_test();
	break;
    case POL2:
	retval = pol2_test();
	break;
    case ABS:
	retval = abs_test();
	break;
    default:
	err_check(ERROR_FAIL,"This shouldn't happen.");
	break;
    }
    printf("\n");
    printf("-- -- -- -- -- -- --");
    printf("-- -- -- -- -- -- --\tend\t-- -- -- -- -- -- --");
    printf("-- -- -- -- -- -- --");
    printf("\n");

    uquad_mat_free(m1);
    uquad_mat_free(m2);
    uquad_mat_free(mr);

    return retval;
}
