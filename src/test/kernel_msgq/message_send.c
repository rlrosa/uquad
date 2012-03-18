#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdint.h>

#define SERVER_KEY 1
#define CLIENT_KEY 2

#define SLEEP_DEFAULT_US 2000
#define SLEEP_STEP_US 100


static uquad_kmsgq_t *server;

void uquad_sig_handler(int signal_num)
{
    fprintf(stdout,"Caught signal %d.\n",signal_num);
    if( server != NULL )
    {
	fprintf(stdout,"Cleaning up uquad_mmsgq...\n");
	uquad_kmsgq_deinit(server);
    }
    exit(0);
}

int main()
{
    int sleep_time = SLEEP_DEFAULT_US;
    int retval;
    unsigned char msg[MSGSZ] = {50, 60, 70, 90};
    uint8_t mark = 0;
    int error_count = 0;

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    server = uquad_kmsgq_init(SERVER_KEY, CLIENT_KEY);

    while(1)
    {
	msg[0] = (unsigned char) mark++;
	retval = uquad_kmsgq_send(server, msg, MSGSZ);
	if(retval != ERROR_OK)
	{
	    sleep_time += SLEEP_STEP_US;
	    error_count++;
	    err_log("Error! Sleeping...");
	}
	else if(sleep_time > SLEEP_DEFAULT_US)
	{
	    err_log_num("Recovered!",sleep_time);
	    err_log_num("Error count:",error_count);
	    error_count = 0;
	    sleep_time = SLEEP_DEFAULT_US;
	}
	usleep(sleep_time);
    }
    /// never gets here
    exit(0);
}
