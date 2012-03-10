#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <stdint.h>

#define SERVER_KEY 1
#define CLIENT_KEY 2

#define SLEEP_DEFAULT_US 2000
#define SLEEP_STEP_US 100

main()
{
    uquad_kmsgq_t *server;
    int sleep_time = SLEEP_DEFAULT_US;
    int retval;
    char msg[MSGSZ] = {50, 60, 70, 90};
    uint8_t mark = 0;
    server = uquad_kmsgq_init(SERVER_KEY, CLIENT_KEY);

    while(1)
    {
	msg[0] = (char) mark++;
	retval = uquad_kmsgq_send(server, msg, MSGSZ);
	if(retval != ERROR_OK)
	{
	    sleep_time += SLEEP_STEP_US;
	    err_log("Error! Sleeping...");
	}
	else if(sleep_time > SLEEP_DEFAULT_US)
	{
	    err_log_num("Recovered!",sleep_time);
	    sleep_time = SLEEP_DEFAULT_US;
	}
	usleep(sleep_time);
    }
    /// never gets here
    exit(0);
}
