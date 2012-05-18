#include <uquad_check_net.h>

#define USAGE "usage:\n\t./server <portno> [<UDP/!TCP>]\n"

int main(int argc, char *argv[])
{
     int
	 portno,
	 retval;
     uquad_bool_t
	 udp = false;

     if (argc > 3)
     {
	 err_check(ERROR_INVALID_ARG, USAGE);
     }
     if (argc > 2)
     {
	 udp = atoi(argv[2]);
     }
     if (argc > 1)
     {
	 portno = atoi(argv[1]);
     }
     else
     {
	 err_log_num("Using default port:", CHECK_NET_PORT);
	 portno = CHECK_NET_PORT;
     }

     retval = uquad_check_net_server(portno, udp);
     err_check(retval, "server() failed!");

     exit(retval);
}
