/**
 * message_send: test program for uquad_kernel_msgq lib
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
 *
 * @file   message_send.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 10:02:32 2012
 *
 * @brief  test program for uquad_kernel_msgq lib
 *
 */
#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdint.h>

#define SERVER_KEY 1
#define CLIENT_KEY 2

#define SLEEP_DEFAULT_US 2000
#define SLEEP_STEP_US 100


static uquad_kmsgq_t *server;

void uquad_sig_handler(int signal_num)
{
    fprintf(stdout,"Caught signal %d.\n",signal_num);
    if( server != NULL )
    {
	fprintf(stdout,"Cleaning up uquad_mmsgq...\n");
	uquad_kmsgq_deinit(server);
    }
    exit(0);
}

int main()
{
    int sleep_time = SLEEP_DEFAULT_US;
    int retval;
    unsigned char msg[MSGSZ] = {50, 60, 70, 90};
    uint8_t mark = 0;
    int error_count = 0;

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    server = uquad_kmsgq_init(SERVER_KEY, CLIENT_KEY);

    while(1)
    {
	msg[0] = (unsigned char) mark++;
	retval = uquad_kmsgq_send(server, msg, MSGSZ);
	if(retval != ERROR_OK)
	{
	    sleep_time += SLEEP_STEP_US;
	    error_count++;
	    err_log("Error! Sleeping...");
	}
	else if(sleep_time > SLEEP_DEFAULT_US)
	{
	    err_log_num("Recovered!",sleep_time);
	    err_log_num("Error count:",error_count);
	    error_count = 0;
	    sleep_time = SLEEP_DEFAULT_US;
	}
	usleep(sleep_time);
    }
    /// never gets here
    exit(0);
}
