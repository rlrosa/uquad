/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include <uquad_types.h>
#include <uquad_error_codes.h>
#include <uquad_aux_time.h>

#define USAGE "usage:\n\t./server portno\n"

static int sockfd = -1, newsockfd = -1;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

void quit()
{
    if(newsockfd > 0)
	close(newsockfd);
    if(sockfd > 0)
	close(sockfd);
    exit(-1);
}

int main(int argc, char *argv[])
{
     int
	 portno,
	 n;
     char
	 buff_o[CHECK_NET_MSG_LEN] = CHECK_NET_ACK,
	 buff_i[CHECK_NET_MSG_LEN];
     struct sockaddr_in servaddr, cliaddr;
     socklen_t len;

     if (argc < 2)
     {
	 quit_log_if(ERROR_INVALID_ARG, USAGE);
     }
     portno = atoi(argv[1]);

     sockfd=socket(AF_INET,SOCK_DGRAM,0);

     bzero(&servaddr,sizeof(servaddr));
     servaddr.sin_family = AF_INET;
     servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
     servaddr.sin_port=htons(portno);
     if(bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) < 0)
     {
	 err_log_stderr("bind()");
	 quit();
     }

     while(1)
     {
	 len = sizeof(cliaddr);
	 n = recvfrom(sockfd,buff_i,CHECK_NET_MSG_LEN,0,(struct sockaddr *)&cliaddr,&len);
	 if (n < 0)
	 {
	     /// Something went wrong, die.
	     err_log_stderr("recvfrom()");
	     quit();
	 }
	 if(n == CHECK_NET_MSG_LEN &&
	    (strcmp(buff_i,CHECK_NET_PING) == 0))
	 {
	     /// Got msg from client, ack to inform we're alive
	     n = sendto(sockfd,buff_o,CHECK_NET_MSG_LEN,
			0,(struct sockaddr *)&cliaddr,sizeof(cliaddr));
	     if (n < 0)
	     {
		 err_log_stderr("sendto()");
		 quit();
	     }
	     /**
	      * Client pings at a CHECK_NET_MSG_T_MS rate.
	      * We'll read more often, since timing is not
	      * a problem on the server side (PC)
	      */
	     sleep_ms(CHECK_NET_MSG_T_MS >> 1);
	 }
	 /// nothing new...
	 usleep(100);
	 continue;

     }
}
