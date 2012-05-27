/**
 * writer: test program for pipe IPC, writer side
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
 * @file   writer.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 10:02:32 2012
 *
 * @brief  test program for pipe IPC, writer side
 *
 */
#include <stdio.h>
#include <sys/stat.h> // for mkfifo()
#include <signal.h> // for SIGINT, SIGQUIT
#include <uquad_aux_io.h>
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>

#define PIPE_NAME "wr_pipe"
#define DEF_PERM 0666

static FILE *pipe_f = NULL;

void quit(void)
{
    if(pipe_f != NULL)
	fclose(pipe_f);
    exit(0);
}

void uquad_sig_handler(int signal_num)
{
    err_log_num("Caught signal!",signal_num);
    quit();
}

int main(int argc, char *argv[])
{
    struct timeval tv_new,tv_old,tv_diff;
    int retval;
    unsigned long counter = 0;
    char *pipe_name = PIPE_NAME;

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    if(argc>=2)
    {
	pipe_name = argv[1];
    }

    pipe_f = fopen(pipe_name, "w");
    if(pipe_f == NULL)
    {
	retval = mkfifo(pipe_name,DEF_PERM);
	if(retval != ERROR_OK)
	{
	    err_log_stderr("Failed to create pipe!");
	    quit();
	}
	pipe_f = fopen(pipe_name, "w");
	if(pipe_f == NULL)
	{
	    err_log_stderr("Failed to open pipe!");
	    quit();
	}
    }

    gettimeofday(&tv_old,NULL);
    for(;;)
    {
	gettimeofday(&tv_new,NULL);
	retval = uquad_timeval_substract(&tv_diff,tv_new,tv_old);
	if(retval < 0)
	{
	    err_log("TIMING: absurd!");
	}
	fprintf(pipe_f,"%lu\n",counter++);
	if(tv_diff.tv_usec > 2)
	    fprintf(stderr,"TX:\t%lu\n",tv_diff.tv_usec);
	gettimeofday(&tv_old,NULL);	
    }
}
