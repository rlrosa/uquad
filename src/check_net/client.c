#include <uquad_check_net.h>

#define USAGE "usage:\n\t./client <host IP> <port> [<TCP/!UDP>]\n"

int main(int argc, char *argv[])
{
    int
	portno,
	retval;
    uquad_bool_t
	tcp = true;

    if (argc < 3 || argc > 4)
    {
	err_check(ERROR_INVALID_ARG, USAGE);
    }
    if (argc > 3)
    {
	tcp = atoi(argv[3]);
    }

    portno = atoi(argv[2]);

    retval = uquad_check_net_client(argv[1], portno, tcp);
    err_check(retval, "client() failed!");
    return retval;
}
