#include <uquad_logger.h>
#include <stdlib.h>
#include <uquad_error_codes.h>
#include <uquad_aux_time.h>

#include <signal.h> // for SIGINT, SIGQUIT
#include <limits.h> // for PATH_MAX

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
    int retval;
    ssize_t read;
    size_t len;

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

    log_file = uquad_logger_add(pipe_name);
    if(log_file == NULL)
    {
	quit_log_if(ERROR_OPEN,"Failed to start pipe!");
    }

    //    gettimeofday(&tv_old,NULL);
    for(;;)
    {
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
    }
    // Never gets here
    return ERROR_FAIL;
}
