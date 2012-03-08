#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

#define MSGSZ     128

#define SEND_T_US 1000
#define ACK_WAIT_SLEEP_US 100
/*
 * Declare the message structure.
 */
typedef struct msgbuf {
         long    mtype;
         char    mtext[MSGSZ];
         } message_buf;

main()
{
    int msqid;
    int msgflg = IPC_CREAT | 0666;
    key_t key, key_rx = 50;
    struct timeval time_tx, time_aux;
    message_buf sbuf, rbuf;
    size_t buf_length;
    int i;

    /*
     * Get the message queue id for the
     * "name" 1234, which was created by
     * the server.
     */
    key = 1;

    while(1)
    {
	key ^= 2;
	if ((msqid = msgget(key, msgflg )) < 0)
	    continue;

	/*
	 * We'll send message type 1
	 */ 
	sbuf.mtype = 1;
    	uint8_t data[4] = {1,2,254,7};
	char cbuf[4] = "teta";
	for(i=0; i < 4; ++i)
	    if(key == 3)
		sbuf.mtext[i] = (char) data[i];
	    else
		sbuf.mtext[i] = cbuf[i];
	buf_length = 4;

	/*
	 * Send a message.
	 */
	if (msgsnd(msqid, &sbuf, buf_length, IPC_NOWAIT) < 0)
	    continue;
	gettimeofday(&time_tx,NULL);
	printf("%d\n",
	       (int)time_tx.tv_usec); /* microseconds */

	usleep(SEND_T_US);

	/// wait answer
	while(1)
	{		
	    // key_rx
	    if ((msqid = msgget(key_rx, 0666)) < 0)
	    {
		usleep(ACK_WAIT_SLEEP_US);
		continue;
	    }

	    /*
	     * Receive an answer of message type 1.
	     */
	    if (msgrcv(msqid, &rbuf, 4, 1, IPC_NOWAIT) < 0)
	    {
		usleep(ACK_WAIT_SLEEP_US);
		continue;
	    }

	    /*
	     * Print the answer.
	     */
	    gettimeofday(&time_aux,NULL);
	    printf("ACK received!:\t%c%c%c%c\t%d\n",
		   rbuf.mtext[0],
		   rbuf.mtext[1],
		   rbuf.mtext[2],
		   rbuf.mtext[3],
		   (int)time_aux.tv_usec); /* microseconds */
	    /// if we got here then we received an ACK
	    break;
	}
    }
    /// never gets here
    exit(0);
}
