#include <uquad_logger.h>
#include <uquad_error_codes.h>
#include <uquad_aux_time.h>
#include <uquad_types.h>
#include <uquad_aux_io.h>
#include <sys/stat.h>   // for mkfifo()
#include <limits.h>     // for PATH_MAX
#include <stdlib.h>
#include <fcntl.h>      // for open()
#include <sys/signal.h> // for SIGINT, SIGQUIT and SIGPIPE
#include <time.h>

#define LINE_PER_FLUSH 10
#define READ_SIZE      128
#define BUFF_SIZE      (READ_SIZE<<8)
#define FLUSH_SIZE     (BUFF_SIZE - READ_SIZE)
#define IO_STUCK_S     1
#define IO_FAIL_SLP_US 500

uquad_bool_t die = false;

void uquad_logger_die(int signal)
{
    err_log_num("logger caught signal, will finish and die...",signal);
    die = true;
}

void uquad_logger_read(int pipefd, char *log_name, char *path)
{
    int retval = ERROR_OK,
	log_fd = -1,
	buff_index = 0;
    char file_name[PATH_MAX];
    char buff[BUFF_SIZE];
    FILE *log_file  = NULL;
    char *new_line = NULL;
    uquad_bool_t write_ok, read_ok;
    struct timeval
	tv_old,
	tv_new,
#if DEBUG
	tv_pre,
#endif
	tv_diff;
    retval = sprintf(file_name,"%s%s.log",path,log_name);
    if(retval < 0)
    {
	err_log_stderr("Failed to create full log name!");
	cleanup_if(ERROR_FAIL);
    }

    retval = remove(file_name);
    if(retval < 0 && errno != ENOENT)
    {
	err_log_stderr("Failed to open log file!");
	cleanup_if(ERROR_OPEN);
    }

    log_fd = open(file_name,O_WRONLY | O_CREAT | O_NONBLOCK | S_IRUSR | S_IWUSR);
    if(log_fd < 0)
    {
	err_log_stderr("Failed to open log file!");
	cleanup_if(ERROR_OPEN);
    }

    gettimeofday(&tv_old,NULL);
    for(;;)
    {
	retval = check_io_locks(pipefd,NULL,&read_ok,NULL);
	if(retval != ERROR_OK)
	{
	    err_log("logger failed to check_io_locks(read)!");
	    usleep(IO_FAIL_SLP_US);
	    continue;
	}
	if(read_ok)
	{
	    gettimeofday(&tv_pre,NULL);
	    retval = read(pipefd, (void *)(buff+buff_index), READ_SIZE);
	    if(retval > 0)
	    {
		buff_index += retval;
#if DEBUG
		gettimeofday(&tv_new,NULL);
		retval = uquad_timeval_substract(&tv_diff, tv_new, tv_pre);
		if(tv_diff.tv_sec > 0)
		{
		    err_log_tv("read() got stuck!",tv_diff);
		}
#endif
		if(buff_index < FLUSH_SIZE)
		    continue;
		retval = check_io_locks(log_fd,NULL,NULL,&write_ok);
		if(retval != ERROR_OK)
		{
		    err_log("logger failed to check_io_locks(write)!");
		}
		if(write_ok)
		{
		    gettimeofday(&tv_pre,NULL);
		    retval = write(log_fd, buff, buff_index);
		    if(retval < 0)
		    {
			err_log_stderr("Failed to write to log file!");
		    }
		    else
		    {
			buff_index -= retval;
#if DEBUG
			gettimeofday(&tv_new,NULL);
			retval = uquad_timeval_substract(&tv_diff, tv_new, tv_pre);
			if(tv_diff.tv_sec > 0)
			{
			    err_log_tv("write() got stuck!",tv_diff);
			}
#endif
		    }
		    fdatasync(log_fd);
		    gettimeofday(&tv_old,NULL);
		}
	    }
	    else
	    {
		if(retval < 0)
		{
		    err_log_std("logger read error!");
		    usleep(IO_FAIL_SLP_US);
		}
		else
		{
		    /// Parent closed pipe, work is done.
		    if(buff_index > 0)
		    {
			retval = write(log_fd, buff, buff_index);
			if(retval < 0)
			{
			    err_log_stderr("Failed to write to log file!");
			}
		    }
		    goto cleanup;
		}
	    }
	}
	else
	{
	    // !read_ok
	    usleep(IO_FAIL_SLP_US);
	}
    }
    cleanup:
    err_log_str("Closing logger:",file_name);
    if(log_file != NULL)
    {
	retval = fflush(log_file);
	if(retval < 0)
	{
	    err_log_stderr("Failed to flush log file!");
	}
	retval = fclose(log_file);
	if(retval < 0)
	{
	    err_log_stderr("Failed to close log file!");
	}
	retval = close(pipefd);
	if(retval < 0)
	{
	    err_log_stderr("Failed to close READ end of pipe");
	}
    }
    if(new_line != NULL)
	free(new_line);
    exit(0);
}

FILE *uquad_logger_add(char *log_name, char *path)
{
    int retval = ERROR_OK;
    FILE *pipe_f;
    int pipefd[2];
    pid_t pid;

    if(path == NULL || log_name == NULL)
    {
	err_log("Invalid arguments!");
	return NULL;
    }

    /// Create pipe for IPC
    retval = pipe(pipefd);
    if(retval < 0)
    {
	err_log_stderr("Failed to create pipe()!");
	return NULL;
    }

    /// Launch logger, will read from pipe
    pid = fork();
    if(pid == 0)
    {
	/// Child - Reads from pipe
	// Close write end of pipe
	close(pipefd[1]);
	// Let logger finish before dying.
	signal(SIGINT, uquad_logger_die);
	signal(SIGQUIT,uquad_logger_die);
	signal(SIGPIPE,uquad_logger_die);
	uquad_logger_read(pipefd[0], log_name, path);
	return NULL;
    }
    else
    {
	/// Parent - Writes to pipe
	// Close read end of pipe
	close(pipefd[0]);
	// Get file descriptor to pass upstream
	pipe_f = fdopen(pipefd[1], "w");
	if(pipe_f == NULL)
	{
	    err_log_stderr("Failed to open write end of pipe!");
	    return NULL;
	}
	return pipe_f;	
    }
}

void uquad_logger_remove(FILE *pipe_f)
{
    if(pipe_f != NULL)
    {
	int pipefd = fileno(pipe_f);
	if(pipefd < 0)
	{
	    err_log_stderr("Failed to get pipefd to close write end of pipe!");
	}
	else
	{
	    close(pipefd);
	}
	//	fclose(pipe_f);
    }
    return;
}

int uquad_log_generate_log_name(char * log_name, char * start_string){
    time_t rawtime;
    struct tm * tm;
    int retval;
    time(&rawtime);
    tm = localtime (&rawtime);
    retval = sprintf(log_name,"%04d_%02d_%02d_xx_%02d_%02d_%02d", 1900 + tm->tm_year, tm->tm_mon + 1,tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
    if(retval < 0)
	return ERROR_FAIL;
    if(start_string != NULL)
	retval = sprintf(log_name,"%s%s",start_string,log_name);
    return retval;
}

int uquad_log_rew_and_clear(FILE * stream, int num_lines){
    int retval,i;
    if(stream == NULL){
	err_check(ERROR_FAIL,"Cannot clear null stream");
    }
    for(i=0;i<num_lines;++i){
	retval = fputs(UQUAD_LOG_UP_ONE_LINE,stdout);
	if(retval == EOF){
	    err_check(ERROR_WRITE,"fputs() returned error!");
	}
    }
    return ERROR_OK;
}
