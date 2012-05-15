#include <uquad_check_net.h>

#define USAGE "usage:\n\t./server <portno> [<TCP/!UDP>]\n"

int main(int argc, char *argv[])
{
     int
	 portno,
	 retval;
     uquad_bool_t
	 tcp = true;

     if (argc < 2 || argc > 3)
     {
	 err_check(ERROR_INVALID_ARG, USAGE);
     }
     if (argc > 2)
     {
	 tcp = atoi(argv[2]);
     }
     portno = atoi(argv[1]);

     retval = uquad_check_net_server(portno, tcp);
     err_check(retval, "server() failed!");

     exit(retval);
}
