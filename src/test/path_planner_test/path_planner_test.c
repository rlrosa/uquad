/**
 * path_planner_test: test program for path planner lib
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
 * @file   path_planner_test.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 10:02:32 2012
 *
 * @brief  test program for path_planner lib
 *
 */
#include <path_planner.h>
#include <uquad_aux_math.h>
#include <macros_misc.h>
#include <stdlib.h>
#include <unistd.h>

enum test_type{
    UPDATE_K = 0,
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

int setpoint_stdin(uquad_mat_t *m)
{
    printf("Enter setpoint:\n");    
    retval = uquad_mat_load(m,stdin);
    err_check(retval,"WARN:Could not load matrix from stdin");
    return retval;
}

int wset_stdin(uquad_mat_t *m)
{
    printf("Enter desired angular speeds:\n");    
    retval = uquad_mat_load(m,stdin);
    err_check(retval,"WARN:Could not load matrix from stdin");
    return retval;
}

int Q_stdin(uquad_mat_t *m)
{
    int i;
    double dtmp;
    double aux[m->c];
    
    printf("Enter diagonal of Q matrix:\n");    
    
    for(i=0; i < m->r; i++)
    {
	if(fscanf(stdin,"%lf",&dtmp) <= 0)
	{
	    err_check(ERROR_READ, "Failed to load data!");
	}
	aux[i] = dtmp;
    }
    err_check(retval,"WARN:Could not load matrix from stdin");

    retval = uquad_mat_zeros(m);
    err_propagate(retval);
    retval = uquad_mat_diag(m,aux);
    err_propagate(retval);

    return retval;
}

int R_stdin(uquad_mat_t *m)
{
    int i;
    double dtmp;
    double aux[m->c];
    
    printf("Enter diagonal of R matrix:\n");    
    
    for(i=0; i < m->r; i++)
    {
	if(fscanf(stdin,"%lf",&dtmp) <= 0)
	{
	    err_check(ERROR_READ, "Failed to load data!");
	}
	aux[i] = dtmp;
    }
    err_check(retval,"WARN:Could not load matrix from stdin");

    retval = uquad_mat_zeros(m);
    err_propagate(retval);
    retval = uquad_mat_diag(m,aux);
    err_propagate(retval);

    return retval;
}

int update_k_test(void)
{
    uquad_mat_t *sp = NULL;
    uquad_mat_t *w  = NULL;
    uquad_mat_t *Q  = NULL;
    uquad_mat_t *R  = NULL;
    path_planner_t *pp =NULL;
    double Ts = 10e-3;

    sp = uquad_mat_alloc(1,12);
    w = uquad_mat_alloc(1,4);
    Q = uquad_mat_alloc(24,24);
    R = uquad_mat_alloc(4,4);

    retval = setpoint_stdin(sp);    
    cleanup_if(retval);

    retval = wset_stdin(w);
    cleanup_if(retval);
    
    retval = Q_stdin(Q);
    cleanup_if(retval);

    retval = R_stdin(R);
    cleanup_if(retval);
    
    pp = pp_init();
    pp->sp->x = sp;
    pp->sp->w = w;
    pp->Q = Q;
    pp->R = R;
    pp->Ts = Ts;
    pp->K = uquad_mat_alloc(4,24);
    retval =  pp_update_K(pp,0);
    cleanup_if(retval);

    cleanup:
    uquad_mat_free(sp);
    uquad_mat_free(w);
    uquad_mat_free(Q);
    uquad_mat_free(R);
    
    return retval;
}

int main(void){
    int retval = ERROR_OK;
    enum test_type sel_test;
    int cmd;
    printf("Select test:\n\t%d:Update feedback matrix\n\t",
	  UPDATE_K);

    scanf("%d",&cmd);
    if(cmd<0 || cmd > TEST_COUNT)
    {
	err_check(ERROR_FAIL,"Invalid input.");
    }
    sel_test = cmd;
    switch(sel_test)
    {
    case UPDATE_K:
	retval = update_k_test();
	break;

    default:
	err_check(ERROR_FAIL,"This shouldn't happen.");
	break;
    }

    
    return retval;
}
