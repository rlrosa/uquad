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
     socklen_t clilen;
     char
	 buff_i[CHECK_NET_MSG_LEN];
     struct sockaddr_in serv_addr, cli_addr;

     if (argc < 2)
     {
	 quit_log_if(ERROR_INVALID_ARG, "ERROR, no port provided");
     }
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0)
     {
	 err_log_stderr("ERROR opening socket");
	 quit();
     }
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0)
     {
	 err_log_stderr("ERROR on binding");
	 quit();
     }
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0)
     {
	 err_log_stderr("ERROR on accept");
	 quit();
     }
     bzero(buff_i,CHECK_NET_MSG_LEN);
     while(1)
     {
	 n = read(newsockfd,buff_i,CHECK_NET_MSG_LEN);
	 if (n < 0)
	 {
	     /// Something went wrong, die.
	     err_log_stderr("Failed to read from socket! Terminating...");
	     quit();
	 }
	 if( n == 0)
	 {
	     /// nothing new...
	     usleep(100);
	     continue;
	 }
	 /// Got msg from client, ack to inform we're alive
	 n = write(newsockfd,CHECK_NET_ACK,CHECK_NET_MSG_LEN);
	 if (n < 0)
	 {
	     err_log_stderr("ERROR writing to socket");
	     quit();
	 }
	 sleep_ms(CHECK_NET_MSG_T_MS >> 1);
     }
}
