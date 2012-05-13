#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <mot_control.h> // for KILL_MOTOR_CMD
#include <uquad_types.h>
#include <uquad_aux_time.h>

static int sockfd = -1;

void quit()
{
    if(sockfd > 0)
	close(sockfd);
    exit(-1);
}

int main(int argc, char *argv[])
{
    int
	portno,
	n,
	retval,
	kill_retries = 0;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    struct timeval
	tv_out,
	tv_sent,
	tv_diff,
	tv_tmp;

    char
	buff_i[CHECK_NET_MSG_LEN],
	buff_o[CHECK_NET_MSG_LEN] = CHECK_NET_PING;

    tv_out.tv_usec = 0;
    tv_out.tv_sec  = 1;

    if (argc < 3)
    {
	err_log("Usage:\n\t./client hostname port");
	quit();
    }
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
    {
	err_log_stderr("ERROR opening socket");
	quit();
    }
    server = gethostbyname(argv[1]);
    if (server == NULL)
    {
	err_log_str("ERROR, no such host", argv[1]);
	quit();
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
	err_log_stderr("ERROR connecting");
	quit();
    }
    bzero(buff_i,CHECK_NET_MSG_LEN);
    while(1)
    {
	/// ping server
	n = write(sockfd,buff_o,strlen(buff_o));
	if (n < 0)
	{
	    err_log_stderr("ERROR writing to socket");
	    goto kill_motors;
	}
	bzero(buff_i,CHECK_NET_MSG_LEN);
	gettimeofday(&tv_sent, NULL);

	/// wait for ack
	while(1)
	{
	    n = read(sockfd,buff_i,CHECK_NET_MSG_LEN);
	    if (n < 0)
	    {
		err_log_stderr("ERROR reading from socket");
		goto kill_motors;
	    }
	    if (n > 0)
	    {
		// ack received
		sleep_ms(CHECK_NET_MSG_T_MS);
		break;
	    }

	    /// check timeout
	    gettimeofday(&tv_tmp, NULL);
	    (void) uquad_timeval_substract(&tv_diff, tv_tmp, tv_sent);
	    retval = uquad_timeval_substract(&tv_diff, tv_out, tv_diff);
	    if(retval > 0)
	    {
		/// timed out, game over
		kill_motors:
		if(kill_retries == 0)
		{
		    err_log("check_net: Connection lost! Killing motors...");
		}
		retval = system(KILL_MOTOR_CMD);
		if(retval < 0 && kill_retries++ < CHECK_NET_KILL_RETRIES)
		{
		    usleep(1000);
		    goto kill_motors;
		}
		quit();
	    }
	    
	}
    }
}
