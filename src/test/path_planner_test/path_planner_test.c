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
#include <control.h>
#include <uquad_aux_math.h>
#include <macros_misc.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

enum test_type{
    TEST_SETPOINTS = 0,
    TEST_FLOW,
    TEST_COUNT
};

uquad_mat_t *x = NULL, *x3x1 = NULL;
path_planner_t *pp = NULL;

void quit()
{
    if(x != NULL)
	uquad_mat_free(x);
    if(x3x1 != NULL)
	uquad_mat_free(x3x1);
    if(pp != NULL)
	pp_deinit(pp);
    exit(ERROR_OK);
}

void sig_handler(int signal_num)
{
    err_log_num("Caught signal:",signal_num);
    quit();
}

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

int test_setpoints(const char *filename)
{
    pp = pp_init(filename,300.0);
    if(pp == NULL)
    {
	err_log("Failed!");
    }
    else
    {
	err_log("Success!");
	pp_deinit(pp);
    }
    return ERROR_OK;
}

int test_flow(const char *filename)
{
    uquad_bool_t
	ctrl_must_update = false;
    int ret = ERROR_OK;
    x    = uquad_mat_alloc(1,STATE_COUNT);   // State vector
    x3x1 = uquad_mat_alloc(1,3);   // State vector
    if(x == NULL || x3x1 == NULL)
    {
	err_check(ERROR_MALLOC, "Failed to allocate aux mem!");
    }
    ret = uquad_mat_zeros(x);
    err_propagate(ret);
    ret = uquad_mat_zeros(x3x1);
    err_propagate(ret);

    pp = pp_init(filename,300.0);
    if(pp == NULL)
    {
	err_log("Failed!");
	return ERROR_FAIL;
    }
    else
    {
	err_log("Waypoints loaded. Current setpoint:");
	uquad_mat_dump_vec(pp->sp->x,0, true);
	fflush(stderr);fflush(stdout);
	while(true)
	{
	    err_log("Input current position [x,y,z]:");fflush(stderr);
	    ret = uquad_mat_load(x3x1, NULL);
	    err_propagate(ret);
	    x->m_full[SV_X] = x3x1->m_full[0];
	    x->m_full[SV_Y] = x3x1->m_full[1];
	    x->m_full[SV_Z] = x3x1->m_full[2];

	    pp_check_progress(pp, x, &ctrl_must_update);
	    err_propagate(ret);
	    if(ctrl_must_update)
	    {
		// control should be updated before the next call
		pp_update_setpoint(pp);
		err_log("-- -- -- -- --");
		err_log("Waypoint reached. Next waypoint:");
		err_log("");
		uquad_mat_dump_vec(pp->sp->x,0, true);
		err_log("-- -- -- -- --");
		ctrl_must_update = false;
	    }
	    else
	    {
		err_log("Waypoint out of range. Current waypoint:");
		uquad_mat_dump_vec(pp->sp->x,0, true);
	    }

	    fflush(stderr);fflush(stdout);
	}
	pp_deinit(pp);
	return ERROR_OK;
    }
    // never gets here
}

int main(int argc, char *argv[]){
    int retval = ERROR_OK;
    enum test_type sel_test;
    int cmd;

    // Catch signals
    signal(SIGINT, sig_handler);
    signal(SIGQUIT, sig_handler);


    if(TEST_COUNT > 1)
    {
	printf("Select test:\n\t%d:Update feedback matrix\n\t%d:Waypoint flow\n\t",
	       TEST_SETPOINTS,
	       TEST_FLOW);

	scanf("%d",&cmd);
	if(cmd<0 || cmd > TEST_COUNT)
	{
	    err_check(ERROR_FAIL,"Invalid input.");
	}
	sel_test = cmd;
    }
    else
	sel_test = 0;
    switch(sel_test)
    {
    case TEST_SETPOINTS:
	retval = test_setpoints((argc > 1)?argv[1]:NULL);
	break;
    case TEST_FLOW:
	retval = test_flow((argc > 1)?argv[1]:NULL);
	break;

    default:
	err_check(ERROR_FAIL,"This shouldn't happen.");
	break;
    }

    quit();
    // never gets here
    return ERROR_FAIL;
}
