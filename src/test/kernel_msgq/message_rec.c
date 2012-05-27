/**
 * message_rec: test program for uquad_kernel_msgq lib
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
 * @file   message_rec.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 10:02:32 2012
 *
 * @brief  test program for uquad_kernel_msgq lib
 *
 */
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // for usleep()
#include <uquad_error_codes.h>

#define MSGSZ     128
#define LISTEN_T_US 500

#define CLIENT_LOG_DATA "kq_c_data.log"
#define CLIENT_LOG_ACK "kq_c_ack.log"
#define USE_STDOUT 1

static FILE *kq_c_data, *kq_c_ack;

void uquad_sig_handler(int signal_num)
{
    fprintf(stdout,"Caught signal %d.\n",signal_num);
    fprintf(stdout,"Cleaning up log files...\n");
    if(kq_c_ack != NULL)
	fclose(kq_c_ack);
    if(kq_c_data != NULL)
	fclose(kq_c_data);
    exit(0);
}

/*
 * Declare the message structure.
 */

typedef struct msgbuf {
    long    mtype;
    char    mtext[MSGSZ];
} message_buf;

int main()
{
    int msqid, i, msg_rx;
    key_t key = 1, key_tx = 2;
    int msgflg = IPC_CREAT | 0666;
    message_buf  rbuf, sbuf;
    char ack_buf[4] = "RXOK";
    size_t buf_length = 4;
    struct timeval detail_time;

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

#if !USE_STDOUT
    kq_c_data = fopen(CLIENT_LOG_DATA,"w");
    kq_c_ack = fopen(CLIENT_LOG_ACK,"w");
    if(kq_c_data == NULL || kq_c_ack == NULL)
    {
	err_check(ERROR_FAIL,"Failed to open log files.");
    }
#else
    kq_c_data = stdout;
    kq_c_ack = stdout;
#endif
    /*
     * Get the message queue id for the
     * "name" 1234, which was created by
     * the server.
     */
    while(1)
    {
	// key_1
	if ((msqid = msgget(key, 0666)) < 0)
	{
	    usleep(LISTEN_T_US);
	    continue;
	}

	/*
	 * Receive an answer of message type 1.
	 */
	msg_rx = 0;
	while (msgrcv(msqid, &rbuf, buf_length, 1, IPC_NOWAIT) >= 0)
	    msg_rx++;
	if(msg_rx == 0)
	    continue;
	/*
	 * Print the answer.
	 */
	gettimeofday(&detail_time,NULL);
	fprintf(kq_c_data,"%d\t%d\t%d\t%d\t%d\n",
		(int) (0xff & rbuf.mtext[0]),
		(int) (0xff & rbuf.mtext[1]),
		(int) (0xff & rbuf.mtext[2]),
		(int) (0xff & rbuf.mtext[3]),
		(int) detail_time.tv_usec); /* microseconds */
	

	/// send ACK
	while(1)
	{
	    if ((msqid = msgget(key_tx, msgflg )) < 0)
	    {
		usleep(LISTEN_T_US);
		continue;
	    }

	    /*
	     * We'll send message type 1
	     */ 
	    sbuf.mtype = 1;
	    for (i = 0; i < 4; ++i)
		sbuf.mtext[i] = ack_buf[i];
	    buf_length = 4;

	    /*
	     * Send a message.
	     */
	    if (msgsnd(msqid, &sbuf, buf_length, IPC_NOWAIT) < 0)
	    {
		usleep(LISTEN_T_US);
		continue;
	    }
	    gettimeofday(&detail_time,NULL);
	    fprintf(kq_c_ack,"%d\n",
		   (int)detail_time.tv_usec); /* microseconds */
	    break;
	} // ACK
    }
    exit(0);
}
