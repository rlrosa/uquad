#include "uquad_kernel_msgq.h"
#include <uquad_aux_math.h>

uquad_kmsgq_t *uquad_kmsgq_init(int key_s, int key_c)
{
    uquad_kmsgq_t *server = (uquad_kmsgq_t *)malloc(sizeof(uquad_kmsgq_t));
    mem_alloc_check(server);
    server->k_s = key_s;
    server->k_c = key_c;
    server->acks_pend = 0;
    server->mbuf.mtype = 1;//UQUAD_MSGTYPE;
    server->msgflg = IPC_CREAT | 0666;
    server->tx_counter = 0;
    server->s_log_data = fopen(UQUAD_KQ_S_LOG_DATA,"w");
    if(server->s_log_data == NULL)
    {
	err_log("Failed to open data log file, using stderr.");
	server->s_log_data = stderr;
    }
    server->s_log_ack = fopen(UQUAD_KQ_S_LOG_ACK,"w");
    if(server->s_log_ack == NULL)
    {
	err_log("Failed to open ack log file, using stderr.");
	server->s_log_ack = stderr;
    }
    if((uquad_kmsgq_clear(server,server->k_s) != ERROR_OK) ||
       (uquad_kmsgq_clear(server,server->k_c) != ERROR_OK))
    {
	uquad_kmsgq_deinit(server);
	server = NULL;
    }
    return server;
}

int uquad_kmsgq_clear(uquad_kmsgq_t *server, key_t key)
{
    int msqid, i, watchdog = 0;
    struct timeval time_aux;
    while(watchdog++ < UQUAD_KQ_MAX_CLEARS)
    {
	if ((msqid = msgget(key, 0666)) < 0)
	    break;
	if(msgctl(msqid, IPC_RMID, NULL) < 0)
	{
	    err_log_stderr("Failed to remove IPC queue!");
	}
    }
    return ERROR_OK;
}

int uquad_kmsgq_get_ack(uquad_kmsgq_t *server)
{
    int msqid, i;
    struct timeval time_ack;
    int ack_msg_flg = IPC_NOWAIT;
    if ((msqid = msgget(server->k_c, 0666)) < 0)
    {
	return ERROR_KQ_NO_ACKS_AVAIL;
    }

    /// Receive an answer of message type UQUAD_MSGTYPE.
    if (msgrcv(msqid, &server->mbuf,
	       MSGSZ,
	       0, /* get any message */
	       IPC_NOWAIT) < 0)
    {
	return ERROR_KQ_NO_ACKS_AVAIL;
    }

    /// Print the answer.
    gettimeofday(&time_ack,NULL);
    for(i = 0; i < MSGSZ; ++i)
	fprintf(server->s_log_ack,"%d\t",(int)server->mbuf.mtext[i]);
    fprintf(server->s_log_ack,"%d\n",(int)time_ack.tv_usec);
    return ERROR_OK;
}

int uquad_kmsgq_check_stat(uquad_kmsgq_t *server)
{
    int retval = ERROR_OK, ack_cnt = 0, watchdog = 0;
    if(server->acks_pend == 0)
	return ERROR_OK;
    while(watchdog++ < 10)
    {
	retval = uquad_kmsgq_get_ack(server);
	if(retval == ERROR_KQ_NO_ACKS_AVAIL)
	    // nothing left to read
	    break;
	// push up weird errors
	err_propagate(retval);
	++ack_cnt;
	if(ack_cnt > UQUAD_KQ_MAX_ACKS)
	{
	    err_check(ERROR_KQ_ACK_TOO_MANY,"Received too many acks! Client broken?");
	}
    }
    server->acks_pend -= ack_cnt;
    if(server->acks_pend < 0)
    {
	err_log("ACK count lost!");
	server->acks_pend = 0;
    }

    if(ack_cnt == 1)
    {
	// ack received correctly
	retval = ERROR_OK;
    }
    else
    {
	// clean up
	if (ack_cnt == 0)
	{
	    // report data was not received by client
	    err_log("WARN: client did not ack!");
	    retval = uquad_kmsgq_clear(server,server->k_s);
	    err_propagate(retval);
	    server->acks_pend = 0;
	    retval = ERROR_KQ_ACK_NONE;
	}
	if (ack_cnt > 1)
	{
	    // report weird stuff comming
	    err_log_num("WARN: recieved more acks than expected! Will clear ack queue. ACKs:",ack_cnt);
	    retval = uquad_kmsgq_clear(server, server->k_c);
	    err_propagate(retval);
	    server->acks_pend = 0;
	    retval = ERROR_KQ_ACK_MORE;
	}
    }
    return retval;
}

int uquad_kmsgq_send(uquad_kmsgq_t *server, uint8_t *msg, int msg_len)
{
    int retval = ERROR_OK,i, msqid;
    struct timeval tsent;
    static int acks_not_recv = 0;
    retval = uquad_kmsgq_check_stat(server);
    switch (retval)
    {
    case ERROR_OK:
	acks_not_recv = 0;
	break;
    case ERROR_KQ_ACK_NONE:
	if(++acks_not_recv > UQUAD_KQ_MAX_ACKS_MISSED)
	{
	    err_check(ERROR_KQ_ACK_NONE,"Missed too many acks!");
	}
	break;
    case ERROR_KQ_ACK_MORE:
	// propagate it
    case ERROR_KQ_ACK_TOO_MANY:
	// propagate it
    default:
	err_propagate(retval);
    }
    /// if here, then msg should be sent
    if ((msqid = msgget(server->k_s, server->msgflg )) < 0)
    {
	err_check(ERROR_KQ_SEND,"msqid failed!");
    }
    msg_len = uquad_min(msg_len, MSGSZ);
    for(i = 0; i < msg_len; ++i)
	server->mbuf.mtext[i] = msg[i];
    /// send msg
    if (msgsnd(msqid, &server->mbuf, msg_len, IPC_NOWAIT) < 0)
    {
	err_check(ERROR_KQ_SEND,"msgsnd failed!");
    }
    server->acks_pend++;
    /// log sent data
    gettimeofday(&tsent,NULL);
    fprintf(server->s_log_data,"%lu\t",server->tx_counter++);
    for(i = 0; i < msg_len; ++i)
	fprintf(server->s_log_data,"%d\t",(int)server->mbuf.mtext[i]);
    fprintf(server->s_log_data,"%d\n",(int)tsent.tv_usec);
    return retval;
}


void uquad_kmsgq_deinit(uquad_kmsgq_t *server)
{
    int retval;
    if(server == NULL)
	return;
    retval = uquad_kmsgq_clear(server, server->k_s);
    if(retval != ERROR_OK)
    {
	err_log("WARN: Failed to clear server queue");
    }
    retval = uquad_kmsgq_clear(server, server->k_c);
    if(retval != ERROR_OK)
    {
	err_log("WARN: Failed to clear client queue");
    }
    if(server->s_log_data != stderr)
	fclose(server->s_log_data);
    if(server->s_log_ack != stderr)
	fclose(server->s_log_ack);
    free(server);
}
