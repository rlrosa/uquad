#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <mot_control.h> // for KILL_MOTOR_CMD
#include <uquad_types.h>
#include <uquad_aux_time.h>
#include <uquad_aux_io.h>

#define USAGE "usage:\n\t./client <host IP> <port>\n"

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
    uquad_bool_t
	read_ok = false,
	server_ok = false;
    struct sockaddr_in
	servaddr;
    struct hostent
	*server;
    struct timeval
	tv_out,
	tv_sent,
	tv_diff,
	tv_tmp;
    socklen_t
	len;
    char
	buff_i[CHECK_NET_MSG_LEN],
	buff_o[CHECK_NET_MSG_LEN] = CHECK_NET_PING;

    tv_out.tv_usec = 0;
    tv_out.tv_sec  = CHECK_NET_TO_S;

    if (argc < 3)
    {
	quit_log_if(ERROR_INVALID_ARG, USAGE);
    }

    portno = atoi(argv[2]);

    server = gethostbyname(argv[1]);
    if(server == NULL)
    {
	err_log_str("ERROR, no such host", argv[1]);
	quit();
    }

    sockfd=socket(AF_INET,SOCK_DGRAM,0);

    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=inet_addr(argv[1]);
    servaddr.sin_port=htons(portno);

    bzero(buff_i,CHECK_NET_MSG_LEN);
    while(1)
    {
	/// ping server
	n = sendto(sockfd,buff_o,CHECK_NET_MSG_LEN,0,
		   (struct sockaddr *)&servaddr,sizeof(servaddr));
	if (n < 0)
	{
	    err_log_stderr("sendto()");
	    goto kill_motors;
	}
	bzero(buff_i,CHECK_NET_MSG_LEN);
	gettimeofday(&tv_sent, NULL);

	/// wait for ack
	while(1)
	{
	    len = sizeof(servaddr);
	    retval = check_io_locks(sockfd, NULL, &read_ok, NULL);
	    quit_if(retval);
	    if(read_ok)
	    {
		n=recvfrom(sockfd,buff_i,CHECK_NET_MSG_LEN,
			   0,(struct sockaddr *)&servaddr,&len);
		if (n < 0)
		{
		    err_log_stderr("recvfrom()");
		    goto kill_motors;
		}
		if (n == CHECK_NET_MSG_LEN &&
		    (strcmp(buff_i,CHECK_NET_ACK) == 0))
		{
		    // ack received
		    server_ok = true;
		    sleep_ms(CHECK_NET_MSG_T_MS);
		    break;
		}
		sleep_ms(CHECK_NET_RETRY_MS);
	    }

	    /// check timeout
	    gettimeofday(&tv_tmp, NULL);
	    (void) uquad_timeval_substract(&tv_diff, tv_tmp, tv_sent);
	    retval = uquad_timeval_substract(&tv_diff, tv_out, tv_diff);
	    if(retval < 0)
	    {
		/// timed out, game over
		kill_motors:
		if(kill_retries == 0)
		{
		    if(!server_ok)
		    {
			err_log("");
			err_log("");
			err_log("");
			err_log("");
			err_log("");
			err_log("-- -- -- -- --");
			err_log("-- -- -- -- -- -- -- -- -- --");
			err_log("WARN: Will NOT run checknet, server never acked...");
			err_log("-- -- -- -- -- -- -- -- -- --");
			err_log("-- -- -- -- --");
			err_log("");
			err_log("");
			err_log("");
			err_log("");
			err_log("");
			quit();
		    }
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
