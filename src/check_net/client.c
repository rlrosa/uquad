#include <uquad_check_net.h>

#define USAGE "usage:\n\t./client <host IP> <port>\n"

int main(int argc, char *argv[])
{
    int
	portno,
	retval;

    if (argc < 3)
    {
	err_check(ERROR_INVALID_ARG, USAGE);
    }

    portno = atoi(argv[2]);

    retval = uquad_check_net_client(argv[1], portno);
    err_check(retval, "client() failed!");
    return retval;
}
