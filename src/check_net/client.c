#include <uquad_check_net.h>
#include <sys/wait.h>     // for waitpid()
#include <limits.h>       // for INT_MAX


#define USAGE "usage:\n\t./client <host IP> <port> [<UDP/!TCP>]\n"

static pid_t child_pid = -1;

void uquad_conn_lost_handler(int signal_num)
{
    pid_t p;
    int status;
    p = waitpid(-1, &status, WNOHANG);
    if(p == child_pid)
    {
	err_log_num("Client died, caught signal",signal_num);
	exit(0);
    }
}

int main(int argc, char *argv[])
{
    int
	portno,
	retval;
    uquad_bool_t
	udp = false;

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
	child_pid = retval;
	retval = ERROR_OK;
	err_log_num("Client running! Child pid:",child_pid);
	for(;;)
	    sleep(INT_MAX);
    }
    return retval;
}
