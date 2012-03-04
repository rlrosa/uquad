#include <uquad_aux_math.h>
#include <stdlib.h>

#define wait_for_enter printf("ERROR!\n") //while(fread(tmp,1,1,stdin) == 0)

enum test_type{
    VECTOR_DOT = 0,
    VECTOR_CROSS,
    MATRIX_PROD,
    MATRIX_DET,
    MATRIX_INV,
    TEST_COUNT
};

int vector_dot_test(void)
{
    uquad_vec_t *v1, *v2, *vr;
    int l,i,j;
    printf("Enter vector length:(less than %d)\n",
	   UQUAD_MAT_MAX_DIM);
    scanf("%d",&l);
    v1 = uquad_vec_alloc(l);
    v2 = uquad_vec_alloc(l);
    vr = uquad_vec_alloc(l);

}

int matrix_prod_test(void)
{
    uquad_mat_t *m1, *m2, *mr;
    int retval;
    int r1,c1,r2,c2,i,j;
    float tmp;
    printf("Enter number of rows and columns of first matrix (less than %d)\n",
	   UQUAD_MAT_MAX_DIM);
    scanf("%d%d",&r1,&c1);
    printf("Enter number of rows and columns of second matrix (less than %d)\n",
	   UQUAD_MAT_MAX_DIM);
    scanf("%d%d",&r2,&c2);

    m1 = uquad_mat_alloc(r1,c1);
    m2 = uquad_mat_alloc(r2,c2);
    mr = uquad_mat_alloc(r1,c2);
    if((m1 == NULL) || (m2 == NULL) || (mr == NULL))
    {
	err_log("Cannot continue.");
	exit(1);
    }

    if(r2==c1)
    {
        printf("Enter rows and columns of First matrix \n");
        printf("Row wise\n");
        for(i=0;i<r1;i++)
            for(j=0;j<c1;j++)
	    {
		scanf("%f",&tmp);
		m1->m[i][j] = tmp;
	    }
        printf("First Matrix is :\n");
        for(i=0;i<r1;i++)
        {
            for(j=0;j<c1;j++)
                printf("%f\t",m1->m[i][j]);
            printf("\n");
        }
        printf("Enter rows and columns of Second matrix \n");
        printf("Row wise\n");
        for(i=0;i<r2;i++)
            for(j=0;j<c2;j++)
	    {
		scanf("%f",&tmp);
		m2->m[i][j] = tmp;
	    }
        printf("Second Matrix is:\n");
        for(i=0;i<r2;i++)
        {
            for(j=0;j<c2;j++)
                printf("%f\t",m2->m[i][j]);
            printf("\n");
        }

	if(uquad_mat_prod(m1,m2,mr) < 0)
	{
	    err_log("Failed!");
	    exit(-1);
	}

        printf("Multiplication of the Matrices:\n");
        for(i=0;i<mr->r;i++)
        {
            for(j=0;j<mr->c;j++)
                printf("%f\t",mr->m[i][j]);
            printf("\n");
        }
    }
    retval = uquad_mat_free(m1);
    retval = uquad_mat_free(m2);
    retval = uquad_mat_free(mr);
}

int main(int argc, char *argv[]){
    int retval = ERROR_OK;
    enum test_type sel_test;
    int tmp;
    printf("Select test:\n\t%d:Vector dot\n\t%d:Vector cross\n\t%d:Matrix product\n\t%d:Matrix det\n\t%d:Matrix inv\n",
	   VECTOR_DOT,
	   VECTOR_CROSS,
	   MATRIX_PROD,
	   MATRIX_DET,
	   MATRIX_INV,
	   TEST_COUNT);
    scanf("%d",&tmp);
    if(tmp<0 || tmp > TEST_COUNT)
    {
	err_check(ERROR_FAIL,"Invalid input.");
    }
    sel_test = tmp;
    switch(sel_test)
    {
    case VECTOR_DOT:
	retval = vector_dot_test();
	break;
    case MATRIX_PROD:
	retval = matrix_prod_test();
	break;
    default:
	err_check(ERROR_FAIL,"This shouldn't happen.");
	break;
    }
    return retval;
}
