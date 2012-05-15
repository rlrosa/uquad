#include <uquad_check_net.h>

#define USAGE "usage:\n\t./server portno\n"

int main(int argc, char *argv[])
{
     int
	 portno,
	 retval;

     if (argc < 2)
     {
	 err_check(ERROR_INVALID_ARG, USAGE);
     }
     portno = atoi(argv[1]);

     retval = uquad_check_net_server(portno);
     err_check(retval, "server() failed!");

     exit(retval);
}
