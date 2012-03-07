#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

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
    int msgflg = IPC_CREAT | 0666;
    key_t key;
    struct timeval detail_time;
    message_buf sbuf;
    size_t buf_length;
    int good = 1, i;

    /*
     * Get the message queue id for the
     * "name" 1234, which was created by
     * the server.
     */
    key = 1234;

    while(1)
    {
	good = 1;	
	if ((msqid = msgget(key, msgflg )) < 0) {
	    good = 0;
	}

	/*
	 * We'll send message type 1
	 */ 
	sbuf.mtype = 1;
    	uint8_t data[4] = {1,2,254,7};
	for(i=0; i < 4; ++i)
	    sbuf.mtext[i] = (char) data[i];
	buf_length = 4;

	/*
	 * Send a message.
	 */
	if (good && msgsnd(msqid, &sbuf, buf_length, IPC_NOWAIT) < 0) {
	    good = 0;
	}
	else
	{
	    gettimeofday(&detail_time,NULL);
	    printf("%d\n",
		   (int)detail_time.tv_usec); /* microseconds */
	}
	sleep(1);
    }
    exit(0);
}
