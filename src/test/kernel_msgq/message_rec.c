#include <sys/types.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <stdlib.h>

#define MSGSZ     128
#define LISTEN_T_US 500

/*
 * Declare the message structure.
 */

typedef struct msgbuf {
    long    mtype;
    char    mtext[MSGSZ];
} message_buf;

main()
{
    int msqid, i;
    key_t key, key_tx = 50;
    int msgflg = IPC_CREAT | 0666;
    message_buf  rbuf, sbuf;
    char ack_buf[4] = "RXOK";
    size_t buf_length = 4;
    struct timeval detail_time;

    /*
     * Get the message queue id for the
     * "name" 1234, which was created by
     * the server.
     */
    key = 1;
    while(1)
    {
	key ^= 2;
	// key_1
	if ((msqid = msgget(key, 0666)) < 0)
	{
	    usleep(LISTEN_T_US);
	    continue;
	}

	/*
	 * Receive an answer of message type 1.
	 */
	if (msgrcv(msqid, &rbuf, buf_length, 1, IPC_NOWAIT) < 0)
	{
	    usleep(LISTEN_T_US);
	    continue;
	}

	/*
	 * Print the answer.
	 */
	gettimeofday(&detail_time,NULL);
	printf("%02X\t%02X\t%02X\t%02X\t%d\n",
	       (int) (0xff & rbuf.mtext[0]),
	       (int) (0xff & rbuf.mtext[1]),
	       (int) (0xff & rbuf.mtext[2]),
	       (int) (0xff & rbuf.mtext[3]),
	       (int)detail_time.tv_usec); /* microseconds */
	

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
	    printf("%d\n",
		   (int)detail_time.tv_usec); /* microseconds */
	    break;
	} // ACK
    }
    exit(0);
}
