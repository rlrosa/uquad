/**
 * forker: test program for uquad_logger lib
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
 * @file   forker.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 10:02:32 2012
 *
 * @brief  test program for uquad_logger lib
 *
 */
#include <uquad_logger.h>
#include <stdlib.h>
#include <uquad_error_codes.h>
#include <uquad_aux_time.h>

#include <signal.h> // for SIGINT, SIGQUIT
#include <limits.h> // for PATH_MAX

#define READ_STDIN 0

#define USAGE					\
    "Not enough arguments!\n"			\
    "Usage:\n\t./forker pipe_name"

FILE *log_file = NULL;
char *new_line = NULL;

void quit(void)
{
    uquad_logger_remove(log_file);
    exit(0);
}

void uquad_sig_handler(int signal_num)
{
    err_log_num("Caught signal! Will finsh and die",signal_num);
    quit();
}

int main(int argc, char *argv[])
{
    //    struct timeval tv_new,tv_old,tv_diff;
    char *pipe_name;
    int
	retval;

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    if(argc < 2)
    {
	err_log(USAGE);
	quit();
    }
    else
    {
	pipe_name = argv[1];
    }

    log_file = uquad_logger_add(pipe_name,"./", 0);
    if(log_file == NULL)
    {
	quit_log_if(ERROR_OPEN,"Failed to start pipe!");
    }

    //    gettimeofday(&tv_old,NULL);
    for(;;)
    {
#if READ_STDIN
	ssize_t read;
	size_t len;
	read = getline(&new_line, &len, stdin);
	if(read < 0)
	{
	    err_log_stderr("getline");
	    quit_log_if(ERROR_IO,"Failed to get new line!, terminating...");
	}
	retval = fprintf(log_file, "%s", new_line);
	if(retval < 0)
	{
	    err_log_stderr("fprintf");
	    quit_log_if(ERROR_IO,"Failed to write log logger, terminating...");
	}
	fflush(log_file);
#else
	int i;
	for(i=0; i < 10; ++i)
	{
	    retval = fprintf(log_file,"%d\n",i);
	    if(retval < 0)
	    {
		err_log_stderr("fprintf");
		quit_log_if(ERROR_IO,"Failed to write log logger, terminating...");
	    }
	    fflush(log_file);
	}
	break;
#endif
    }
    quit();
    return 0;
}
