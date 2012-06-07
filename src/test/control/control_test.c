/**
 * control_test: test program for control lib
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
 * @file   control_test.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  test program for control lib.
 *
 */
#include <control.h>
#include <path_planner.h>
#include <uquad_aux_math.h>
#include <imu_comm.h>           // For TS_DEFAULT_US

#include <sys/signal.h>         // for SIGINT and SIGQUIT
#include <uquad_error_codes.h>
#include <uquad_types.h>

#define USAGE "./control_test x_hat.log"

static uquad_mat_t
*x_hat = NULL,
*w     = NULL;

static ctrl_t *ctrl       = NULL;
static path_planner_t *pp = NULL;

void quit()
{
    // Cleanup goes here
    uquad_mat_free(x_hat);
    uquad_mat_free(w);
    control_deinit(ctrl);
    pp_deinit(pp);
    printf("Exit successful!\n");
    exit(0);
}

void uquad_sig_handler(int signal_num){
    err_log_num("Caught signal:",signal_num);
    quit();
}

int main(int argc, char *argv[])
{
    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    int
	i,
	retval;
    FILE *file;
    uquad_mat_t *x, *w;
    uquad_bool_t ctrl_outdated = false;
    double w_hover;

    if(argc < 2)
    {
	err_log_str("Invalid arguments!",USAGE)
	quit();
    }
    else
    {
	file = fopen(argv[1],"r");
	if(file == NULL)
	{
	    err_log_stderr("Failed to open log!");
	    quit();
	}
    }
    x = uquad_mat_alloc(STATE_COUNT,1);
    w = uquad_mat_alloc(LENGTH_INPUT,1);
    if(x == NULL || w == NULL)
    {
	quit_log_if(ERROR_MALLOC,"Failed to allocate tmp mem!");
    }

    /**
     * The following should be done by calling mot_update_w_hover(), this
     * is just a test program.
     */
    retval = uquad_solve_pol2(&w_hover, NULL, F_B1, F_B2, -GRAVITY*MASA_DEFAULT/4.0);
    quit_log_if(retval, "Failed to get w_hover!");
    for(i=0; i<LENGTH_INPUT; ++i)
	w->m_full[i] = w_hover;

    retval = uquad_mat_zeros(x);
    quit_if(retval);

    ctrl = control_init();
    if(ctrl == NULL)
    {
	quit_if(ERROR_FAIL);
    }

    pp = pp_init();
    if(pp == NULL)
    {
	quit_if(ERROR_FAIL);
    }

    retval = pp_new_setpoint(pp, x, w);
    quit_if(retval);

    retval = control_update_K(ctrl, pp, MASA_DEFAULT);
    quit_log_if(retval, "Failed to update control matrix! Aborting...");

    for(;;)
    {
	retval = uquad_mat_load(x,file);
	if(retval != ERROR_OK)
	{
	    quit_log_if(retval, "End of log?");
	}
	#warning "w_hover should use mot_control.h!!"

	retval = pp_update_setpoint(pp, x, w_hover, &ctrl_outdated);
	quit_if(retval);

	if(ctrl_outdated)
	{
	    retval = control_update_K(ctrl, pp, MASA_DEFAULT);
	    quit_log_if(retval, "Failed to update control matrix! Aborting...");
	    retval = control_dump(ctrl, NULL);
	    quit_log_if(retval, "Failed to dump new control matrix! Aborting...");
	}
	retval = control(ctrl, w, x, pp->sp, TS_DEFAULT_US);
	quit_if(retval);
	ctrl_outdated = false;
    }
    // Never gets here
    quit();

}
