#include <sys/types.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <stdlib.h>

#define MSGSZ     128


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
    key_t key;
    message_buf  rbuf;
    int good;
    struct timeval detail_time;

    /*
     * Get the message queue id for the
     * "name" 1234, which was created by
     * the server.
     */
    key = 1234;
    while(1)
    {
	good = 1;
	if ((msqid = msgget(key, 0666)) < 0) {
	    good = 0;
	}

	/*
	 * Receive an answer of message type 1.
	 */
	if (good && msgrcv(msqid, &rbuf, 4, 1, 0) < 0) {
	    good = 0;
	}

	/*
	 * Print the answer.
	 */
	if(good)
	{
	    //	    printf("%s\n", rbuf.mtext);
	    gettimeofday(&detail_time,NULL);
	    printf("%02X\t%02X\t%02X\t%02X\t%d\n",
		   0xff & rbuf.mtext[0],
		   0xff & rbuf.mtext[1],
		   0xff & rbuf.mtext[2],
		   0xff & rbuf.mtext[3],
		   (int)detail_time.tv_usec); /* microseconds */
	}
    }
    exit(0);
}
