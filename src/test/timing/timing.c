#include <uquad_aux_time.h>
#include <uquad_error_codes.h>
#include <uquad_logger.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/signal.h>   // for SIGINT and SIGQUIT
#include <unistd.h>       // for STDIN_FILENO

#define TIME_LOG_NAME "time"
#define LOOP_SLEEP_US 0

FILE * log_time;

void quit()
{
    uquad_logger_remove(log_time);
    exit(0);
}

void uquad_sig_handler(int signal_num){
    err_log_num("Caught signal:",signal_num);
    quit();
}


int main(int argc, char *argv[])
{
    int retval;
    int counter = 0;
    int sleep_time_us;
    struct timeval tv_diff,t1,t2;
    
    if(argc < 2)
	sleep_time_us = 0;
    else
	sleep_time_us = atoi(argv[1]);

    

    err_log_num("Using sleep of:",sleep_time_us);

    log_time = uquad_logger_add(TIME_LOG_NAME);
    if(log_time == NULL)
    {
	quit_log_if(ERROR_IO, "Failed to add time log!");
    }

    retval = gettimeofday(&t2,NULL);
    if(retval != ERROR_OK)
    {
	err_log_std("Failed to get t2!");
	quit_if(ERROR_FAIL);
    }

    for(;;)
    {
	retval = gettimeofday(&t1,NULL);
	if(retval != ERROR_OK)
	{
	    err_log_std("Failed to get t1!");
	    quit_if(ERROR_FAIL);
	}
	retval = uquad_timeval_substract(&tv_diff, t1, t2);
	if(retval < 0)
	{
	    quit_log_if(ERROR_TIMING,"Absurd timing!");
	}
	log_int_only(log_time, counter++);
	log_tv_only(log_time, tv_diff);
	log_eol(log_time);

	retval = gettimeofday(&t2,NULL);
	if(retval != ERROR_OK)
	{
	    err_log_std("Failed to get t2!");
	    quit_if(ERROR_FAIL);
	}
	if(sleep_time_us > 0)
	    usleep(sleep_time_us);
    }

    // never gets here
    return ERROR_OK;
}
