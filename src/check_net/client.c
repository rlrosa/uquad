#include <uquad_check_net.h>

#define USAGE "usage:\n\t./client <host IP> <port> [<UDP/!TCP>]\n"

int main(int argc, char *argv[])
{
    int
	portno,
	hold_on_sec = 2,
	retval;
    uquad_bool_t
	udp = true;

    if (argc < 3 || argc > 4)
    {
	err_check(ERROR_INVALID_ARG, USAGE);
    }
    if (argc > 3)
    {
	udp = atoi(argv[3]);
    }

    portno = atoi(argv[2]);

    retval = uquad_check_net_client(argv[1], portno, udp);
    if(retval < 0)
    {
	err_check(retval, "client() failed!");
    }
    else
    {
	err_log_num("Client running! Child pid:",retval);
	err_log_num("Client running, parent will die in [sec]...", hold_on_sec);
	sleep(hold_on_sec);
	retval = ERROR_OK;
    }
    return retval;
}
