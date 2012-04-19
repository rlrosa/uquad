#include <control.h>
#include <path_planner.h>
#include <uquad_aux_math.h>
#include <imu_comm.h>           // For TS_DEFAULT_US

#include <sys/signal.h>         // for SIGINT and SIGQUIT
#include <uquad_error_codes.h>
#include <uquad_types.h>

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

    int retval;
    FILE *file;

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

    for(;;)
    {
	retval = uquad_mat_load(x,file);
	if(retval != ERROR_OK)
	{
	    quit_log_if(retval, "End of log?");
	}
	retval = pp_update_setpoint(pp, x);
	quit_if(retval);
	retval = control(ctrl, w, x, pp->sp, TS_DEFAULT_US);
	quit_if(retval);
    }
    // Never gets here
    quit();

}
