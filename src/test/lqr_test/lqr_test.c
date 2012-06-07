/**
 * lqr_test: test program for lqr implementation
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
 * @file   lqr_test.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 10:02:32 2012
 *
 * @brief  test program for lqr implementation
 *
 */
#include <uquad_aux_math.h>
#include <uquad_types.h>
#include <uquad_config.h>
#include <path_planner.h>
#include <control.h>
#include <stdlib.h>

int main(){
    int retval;
    uquad_mat_t *A = NULL;
    uquad_mat_t *B = NULL;
    uquad_mat_t *Q = NULL;
    uquad_mat_t *R = NULL;
    uquad_mat_t *phi = NULL;
    uquad_mat_t *gamma = NULL;
    uquad_mat_t *K =NULL;
    FILE *matrix = NULL;

    matrix = fopen("A","r");
    if(matrix == NULL)
    {
	err_log_stderr("fopen(A)");
	cleanup_if(ERROR_IO);
    }
    A = uquad_mat_alloc(STATES_CONTROLLED + STATES_INT_CONTROLLED,
			STATES_CONTROLLED + STATES_INT_CONTROLLED);
    retval = uquad_mat_load(A, matrix);
    cleanup_if(retval);

    matrix = fopen("B","r");
    if(matrix == NULL)
    {
	err_log_stderr("fopen(B)");
	cleanup_if(ERROR_IO);
    }
    B = uquad_mat_alloc(STATES_CONTROLLED + STATES_INT_CONTROLLED,LENGTH_INPUT);
    retval = uquad_mat_load(B,matrix);
    cleanup_if(retval);	
    
    matrix = fopen("Q","r"); 
    if(matrix == NULL)
    {
	err_log_stderr("fopen(Q)");
	cleanup_if(ERROR_IO);
    }
    Q = uquad_mat_alloc(STATES_CONTROLLED + STATES_INT_CONTROLLED,
			STATES_CONTROLLED + STATES_INT_CONTROLLED);
    retval = uquad_mat_load(Q,matrix);
    cleanup_if(retval);
       
    matrix = fopen("R","r"); 
    if(matrix == NULL)
    {
	err_log_stderr("fopen(R)");
	cleanup_if(ERROR_IO);
    }
    R = uquad_mat_alloc(LENGTH_INPUT,LENGTH_INPUT);
    retval = uquad_mat_load(R,matrix);
    cleanup_if(retval);

    phi = uquad_mat_alloc(A->r, A->c);
    gamma = uquad_mat_alloc(B->r, B->c);
    K = uquad_mat_alloc(LENGTH_INPUT,
			STATES_CONTROLLED + STATES_INT_CONTROLLED);

    retval = control_disc(phi,gamma,A,B,10e-3);
    cleanup_if(retval);
    retval = control_lqr(K,phi,gamma,Q,R);
    cleanup_if(retval);

    uquad_mat_dump(K,0);
    
    
    cleanup:
    uquad_mat_free(A);
    uquad_mat_free(B);
    uquad_mat_free(Q);
    uquad_mat_free(R);
    uquad_mat_free(phi);
    uquad_mat_free(gamma);
    uquad_mat_free(K);

    return retval;
    }
