/**
 * server: Example usage of check_net library, server side. will run "forever".
 * Copyright (C) 2012  Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
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
