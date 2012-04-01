#include <stdio.h>
#include <sys/stat.h> // for mkfifo()
#include <signal.h> // for SIGINT, SIGQUIT
#include <uquad_aux_io.h>
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>

#define PIPE_NAME "wr_pipe"
#define DEF_PERM 0666

#define LOG_STD_ERR 1
#define LOG_FILE 1

static FILE *pipe_f = NULL;
static FILE *log_file = NULL;

void quit(void)
{
    if(pipe_f != NULL)
	fclose(pipe_f);
    if(log_file != NULL)
	fclose(log_file);
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
    char *pipe_name = PIPE_NAME;
    unsigned long ul = 0;

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    if(argc>=2)
    {
	pipe_name = argv[1];
    }

    pipe_f = fopen(pipe_name, "r");
    if(pipe_f == NULL)
    {
	retval = mkfifo(pipe_name,DEF_PERM);
	if(retval != ERROR_OK)
	{
	    err_log_stderr("Failed to create pipe!");
	    quit();
	}
	pipe_f = fopen(pipe_name, "r");
	if(pipe_f == NULL)
	{
	    err_log_stderr("Failed to open pipe!");
	    quit();
	}
    }

    log_file = fopen("read.log","w");
    if(log_file == NULL)
    {
	err_log_stderr("Failed to open log file!");
	quit();
    }

    gettimeofday(&tv_old,NULL);
    for(;;)
    {
	retval = fscanf(pipe_f,"%lu",&ul);
	if(retval > 0)
	{
	    gettimeofday(&tv_new,NULL);
	    retval = uquad_timeval_substract(&tv_diff,tv_new,tv_old);
	    if(retval < 0)
	    {
		err_log("TIMING: absurd!");
	    }
#if LOG_STD_ERR
	    fprintf(stderr,"RX stamp:\t%lu\t",ul);
	    log_tv(stderr,"RX time:\t",tv_diff);
#endif
#if LOG_FILE
	    fprintf(log_file,"RX stamp:\t%lu\t",ul);
	    log_tv(log_file,"RX time:\t",tv_diff);
#endif

	    gettimeofday(&tv_old,NULL);	
	}
    }
}
