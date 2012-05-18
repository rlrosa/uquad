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

    if (argc > 4)
    {
	err_check(ERROR_INVALID_ARG, USAGE);
    }
    if (argc > 3)
    {
	udp = atoi(argv[3]);
    }
    if (argc > 2)
    {
	portno = atoi(argv[2]);
    }
    else
    {
	err_log_num("Using default port:", CHECK_NET_PORT);
	portno = CHECK_NET_PORT;
    }
    if (argc <= 1)
    {
	err_log_str("Using default checknet IP:", CHECK_NET_SERVER_IP);
    }

    retval = uquad_check_net_client((argc > 1)?argv[1]:CHECK_NET_SERVER_IP,
				    portno,
				    udp);
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
