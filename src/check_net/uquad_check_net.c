#include <uquad_check_net.h>
#include <uquad_aux_io.h>
#include <sys/prctl.h>
#include <signal.h> // for SIGHUP

int uquad_check_net_server(int portno, uquad_bool_t udp)
{
    int
	sockfd = -1,
	newsockfd = -1,
	n,
	retval;
    uquad_bool_t
	read_ok;
    char
	buff_o[CHECK_NET_MSG_LEN] = CHECK_NET_ACK,
	buff_i[CHECK_NET_MSG_LEN];
    struct sockaddr_in servaddr, cliaddr;
    socklen_t len;

    sockfd=socket(AF_INET,
		  (!udp)?SOCK_STREAM:SOCK_DGRAM,
		  0);

    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=INADDR_ANY;
    servaddr.sin_port=htons(portno);
    len = sizeof(cliaddr);
    if(bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) < 0)
    {
	err_log_stderr("bind()");
	cleanup_if(ERROR_FAIL);
    }

    err_log_num(udp?"UDP server running on port:":"TCP server running on port:"
		,portno);

    if(!udp)
    {
	if(listen(sockfd,5) < 0)
	{
	    err_log_stderr("listen()");
	    cleanup_if(ERROR_FAIL);
	}
	newsockfd = accept(sockfd,(struct sockaddr *) &cliaddr,
			   &len);
	if(newsockfd < 0)
	{
	    err_log_stderr("accept()");
	    cleanup_if(ERROR_FAIL);
	}
	else
	{
	    err_log("Connection established!");
	}
    }

    while(1)
    {
	retval = check_io_locks(!udp?newsockfd:sockfd, NULL, &read_ok, NULL);
	cleanup_log_if(retval, "Failed to check locks on socket!");
	if(!read_ok)
	    goto nothing_new;

	n = recvfrom(!udp?newsockfd:sockfd,buff_i,CHECK_NET_MSG_LEN,0,(struct sockaddr *)&cliaddr,&len);
	if (n <= 0)
	{
	    if(n < 0)
	    {
		err_log_stderr("recvfrom()");
	    }
	    if(udp)
	    {
		cleanup_log_if(ERROR_IO, "Terminating server...");
	    }
	    else
	    {
		/**
		 * We already used select() to check for data, so if we are
		 * at this point, it should be n>0.
		 * If n == 0, then the client has closed his side of the socket.
		 * When using TCP, we must also close our side, to enable another
		 * client to replace the dead one.
		 */
		if(newsockfd > 0)
		{
		    err_log("Client closed socket, so will server...");
		    close(newsockfd);
		    newsockfd = accept(sockfd,(struct sockaddr *) &cliaddr,
				       &len);
		    if(newsockfd < 0)
		    {
			err_log_stderr("accept()");
			cleanup_if(ERROR_FAIL);
		    }
		    else
		    {
			err_log("Connection established!");
		    }
		    continue;
		}
	    }
	}
	if(n == CHECK_NET_MSG_LEN &&
	   (memcmp(buff_i,CHECK_NET_PING,CHECK_NET_MSG_LEN) == 0))
	{
	    /// Got msg from client, ack to inform we're alive
#ifdef DEBUG_CHECK_NET
	    err_log("server(): ping received!");
#endif
	    n = sendto(!udp?newsockfd:sockfd,buff_o,CHECK_NET_MSG_LEN,
		       0,(struct sockaddr *)&cliaddr,sizeof(cliaddr));
	    if (n < 0)
	    {
		err_log_stderr("sendto()");
		cleanup_if(ERROR_IO);
	    }
	    /**
	     * Client pings at a CHECK_NET_MSG_T_MS rate.
	     * We'll read more often, since timing is not
	     * a problem on the server side (PC)
	     */
	    sleep_ms(CHECK_NET_MSG_T_MS >> 1);
	}
	/// nothing new...
	nothing_new:
	usleep(100);
	continue;
    }
    cleanup:
    if(newsockfd > 0)
	close(newsockfd);
    if(sockfd > 0)
	close(sockfd);
    /// server should never die, any reason is an error
    return ERROR_FAIL;
}

static int sockfd = -1;
void client_sig_handler(int signal_num)
{
    if(signal_num == SIGHUP ||
       signal_num == SIGINT ||
       signal_num == SIGQUIT)
    {
	err_log_num("Parent died, so will client... Signal:", signal_num);
	sleep(1);
	if(sockfd > 0)
	    close(sockfd);
	_exit(0);
    }
}

int uquad_check_net_client(const char *hostIP, int portno, uquad_bool_t udp)
{
    int
	n,
	retval,
	pid,
	err_output = -1,
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

    server = gethostbyname(hostIP);
    if(server == NULL)
    {
	err_log_str("ERROR, no such host", hostIP);
	cleanup_if(ERROR_FAIL);
    }

    sockfd=socket(AF_INET,
		  (!udp)?SOCK_STREAM:SOCK_DGRAM,
		  0);

    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
         (char *)&servaddr.sin_addr.s_addr,
         server->h_length);
    servaddr.sin_port=htons(portno);
    if(!udp)
    {
	if(connect(sockfd,(struct sockaddr *) &servaddr,sizeof(servaddr)) < 0)
	{
	    err_log_stderr("connect()");
	    cleanup_if(ERROR_FAIL);
	}
    }

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
	/// give server some time to answer
	sleep_ms(CHECK_NET_RETRY_MS);

	/// wait for ack
	while(1)
	{
	    len = sizeof(servaddr);
	    retval = check_io_locks(sockfd, NULL, &read_ok, NULL);
	    cleanup_if(retval);
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
		    (memcmp(buff_i,CHECK_NET_ACK,CHECK_NET_MSG_LEN) == 0))
		{
		    // ack received
#ifdef DEBUG_CHECK_NET
		    err_log("client(): ack received!");
#endif
		    if(!server_ok)
		    {
			server_ok = true;
			err_log_num(udp?
				    "UDP client running on port:":
				    "TCP client running on port:"
				    ,portno);
			pid = fork();
			if(pid < 0)
			{
			    err_log_stderr("fork()");
			    cleanup_if(ERROR_FAIL);
			}
			if(pid > 0)
			{
			    /// Parent process, go back to being usefull
			    err_output = (int)pid;
			    goto cleanup;
			}
			else
			{
			    /**
			     * Child process.
			     * Ask kernel to tell us when daddy dies.
			     */
			    prctl(PR_SET_PDEATHSIG, SIGHUP);
			    signal(SIGHUP, client_sig_handler);
			    signal(SIGINT, client_sig_handler);
			    signal(SIGQUIT, client_sig_handler);
			}
		    }
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
			err_log("WARN: Will NOT run checknet, server never acked...");
			goto cleanup;
		    }
		    err_log("check_net: Connection lost! Killing motors...");
		}
		retval = system(KILL_MOTOR_CMD);
		if(retval < 0 && kill_retries++ < CHECK_NET_KILL_RETRIES)
		{
		    usleep(1000);
		    goto kill_motors;
		}
		goto cleanup;
	    }
	}
    }
    cleanup:
    if(sockfd > 0)
	close(sockfd);
    /// Client should never die, any reason is an error
    return err_output;
}
