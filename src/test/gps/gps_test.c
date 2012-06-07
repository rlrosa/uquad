/**
 * gps_test: test for uquad_gps_comm lib
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
 * @file   gps_test.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  test for uquad_gps_comm lib
 *
 * Features;
 *   - Allows to test a simple interaction with GPSD, or the uquad_gps_comm lib.
 *
 */
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#ifndef UQUAD_GPS_LIB_TEST // test the uquad interface to gpsd
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#include <uquad_gps_comm.h>
#include <uquad_aux_io.h>
#include <uquad_error_codes.h>
#include <uquad_config.h>
#include <sys/signal.h> // for SIGINT and SIGQUIT
#include <unistd.h>     // For STDIN_FILENO

#define DUMP_STDOUT  1
#define UTM_CONV     0
#define ERR_MAX      50

FILE *input = NULL,*output = NULL, *output_file = NULL;

gps_t *gps = NULL;
io_t *io   = NULL;

void quit()
{
    int ret;
    ret = io_deinit(io);
    if(ret != ERROR_OK)
    {
	err_log("Failed to deinit io.");
    }
    gps_comm_deinit(gps);
    if(input != NULL && input != stdin)
	fclose(input);
    if(output != NULL && output != stdout)
	fclose(output);
    if(output_file != NULL)
	fclose(output_file);
    exit(0);
}

void uquad_sig_handler(int signal_num)
{
    err_log_num("Caught signal:",signal_num);
    quit();
}

void utm_only(int argc, char *argv[])
{
    int ret;
    utm_t utm;
    double
	dtmp,
	lat,
	lon;

    if(argc < 2)
    {
	input = stdin;
	output = stdout;
    }
    else
    {
	input = fopen(argv[1],"r");
	if(input == NULL)
	{
	    err_log_stderr("Failed to open file!");
	    quit();
	}
	output = fopen("output.log","w");
	if(output == NULL)
	{
	    err_log_stderr("Failed to open file!");
	    quit();
	}
    }

    for(;;)
    {
	if(input == stdin)
	{
	    ret = fprintf(stdout,"\nlat lon:");
	    if(ret <= 0)
	    {
		err_log_stderr("Failed to write to stdout!");
		quit();
	    }
	}
	ret = fscanf(input,"%lf",&dtmp);
	if(ret <= 0)
	{
	    err_log_stderr("Failed to read from input!");
	    quit();
	}
	lat = dtmp;
	ret = fscanf(input,"%lf",&dtmp);
	if(ret <= 0)
	{
	    err_log_stderr("Failed to read from input!");
	    quit();
	}
	lon = dtmp;
	ret = gps_comm_deg2utm(&utm,lat,lon);
	quit_log_if(ret,"Failed to convert to UTM!");
	ret = fprintf(output,"%lf\t%lf\n",utm.easting,utm.northing);
	if(ret <= 0)
	{
	    err_log_stderr("Failed to write to output!");
	    quit();
	}
    }
}

int main(int argc, char *argv[])
{
    int ret;
    uquad_bool_t
	got_fix   = false,
	using_log = false;
    char *dev = NULL;
    struct timeval
	t_out,
	tv_start,
	tv_tmp,
	tv_diff;
    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    gettimeofday(&tv_start,NULL);

    if(UTM_CONV)
    {
	utm_only(argc, argv);
	err_log("Should never get here!");
	quit();
    }
    if(argc > 1)
    {
	using_log = true;
	dev = argv[1];	
    }
    if(argc > 2)
    {
	output_file = fopen(argv[2],"w");
	if(output_file == NULL)
	{
	    err_log_stderr("Failed to open log file!");
	    quit_if(ERROR_WRITE);
	}
    }

    gps = gps_comm_init(dev);
    if(gps == NULL)
    {
	quit_log_if(ERROR_MALLOC,"GPS test failed.");
    }

    if(!using_log)
    {
	err_log("Will attempt to get GPS fix...");
	t_out.tv_usec = 0;
	t_out.tv_sec  = 1;
	ret = gps_comm_wait_fix(gps,&got_fix,NULL);
	quit_log_if(ret,"Error waiting for gps!");
	if(!got_fix)
	{
	    quit_log_if(ERROR_GPS,"Failed to get GPS fix!");
	}
    }
    else
    {
	ret = gps_comm_read(gps, &got_fix, &tv_start);
	quit_if(ret);
	if(!got_fix)
	    quit_log_if(ERROR_READ, "Failed to read from log file!");
    }

    io = io_init();
    if(io == NULL){
	quit_log_if(ERROR_MALLOC,"GPS test failed.");
    }

    // add GPS
    ret = io_add_dev(io, gps_comm_get_fd(gps));
    quit_log_if(ret,"Failed to add device to io.");

    // add stdin to support clean exit
    ret = io_add_dev(io,STDIN_FILENO);
    quit_log_if(ret,"Failed to add STDIN to dev list");

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Poll n read loop
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    uquad_bool_t
	read  = false,
	write = false,
	ok    = false;
    int err_count = 0;
    uquad_bool_t reg_gps = true, reg_stdin = true;
    gps_comm_data_t *gps_data;
    gps_data = gps_comm_data_alloc();
    if(gps_data == NULL)
    {
	quit_log_if(ERROR_MALLOC, "Failed to allocate gps_data!");
    }
    unsigned char tmp_buff[2];
    //    poll_n_read:
    while(1){
	if(ret != ERROR_OK)
	{
	    if(err_count++ > ERR_MAX)
		quit_log_if(ERROR_FAIL,"Too many errors, aborting...");
	}
        ret = io_poll(io);
        quit_log_if(ret,"io_poll() error");
        // gps
        if(reg_gps){
            ret = io_dev_ready(io,gps_comm_get_fd(gps),&read,&write);
            quit_log_if(ret,"io_dev_ready() error");
            if(read)
	    {
		gettimeofday(&tv_tmp,NULL);
		if(using_log)
		{
		    ret = gps_comm_read(gps, &ok, &tv_tmp);
		    log_n_continue(ret, "Failed to read from log file!");
		}
		else
		{
		    ret = gps_comm_read(gps, &ok, NULL);
		    log_n_continue(ret, "Fail to get expected data from GPS!");
		}

		uquad_timeval_substract(&tv_diff, tv_tmp, tv_start);

		if(ok)
		{
		    if(gps_comm_3dfix(gps))
		    {
			ret = gps_comm_get_data(gps, gps_data, NULL);
			log_n_continue(ret, "Failed to get data!");
			if(output_file != NULL)
			{
			    log_tv_only(output_file,tv_diff);
			    gps_comm_dump(gps, gps_data, output_file);
			}
			if(DUMP_STDOUT)
			{
			    log_tv_only(stdout,tv_diff);
			    gps_comm_dump(gps, gps_data, stdout);
			}
		    }
		    else
		    {
			err_log("Ignoring GPS data, no 3D fix!");
			sleep_ms(250);
		    }
		}
            }
        }
        // stdin
        if(reg_stdin){
            ret = io_dev_ready(io,STDIN_FILENO,&read,&write);
            quit_log_if(ret,"io_dev_ready() error");
            if(read){
                ret = fread(tmp_buff,1,1,stdin);
                if(ret<=0)
                    fprintf(stdout,"\nNo user input!!\n\n");
                else
		{
                    // Any user input will terminate program
                    quit();
		}
            }
        }
        fflush(stdout);
	err_count = 0;
    }


    // Failure...
    quit();
}

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#else //UQUAD_GPS_LIB_TEST defined, testing gpsd library directly.
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <ctype.h>

#include "gpsd.h"

#ifndef S_SPLINT_S
#include <unistd.h>
#endif /* S_SPLINT_S */
#include <getopt.h>
#include <signal.h>

#define sleep_ms(time_ms) usleep(1000*time_ms)

static struct gps_data_t gpps_data;

int main(int argc, char *argv[])
{
    int ret;
    
    ret = gps_open(NULL, 0, &gpps_data);

    (void) gps_stream(&gpps_data, WATCH_ENABLE | WATCH_JSON, NULL);

    while(1)
    {
	/* Put this in a loop with a call to a high resolution sleep () in it. */
	if (gps_waiting (&gpps_data, 500)) {
	    errno = 0;
	    if (gps_read (&gpps_data) == -1) {
		//TODO die in a handsome way
		printf("ERROR!\n");
		goto close_and_die;	    
	    } else {
		/* Display data from the GPS receiver. */
		if (gpps_data.set)
		    printf("fix mode:%d\n",gpps_data.fix.mode);
	    }
	}
	sleep_ms(1000);
    }

    /* When you are done... */
    close_and_die:
    (void) gps_stream(&gpps_data, WATCH_DISABLE, NULL);
    (void) gps_close (&gpps_data);
}

#endif //UQUAD_GPS_LIB_TEST
