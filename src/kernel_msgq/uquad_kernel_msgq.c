#include "uquad_kernel_msgq.h"
#include <uquad_aux_math.h>

uquad_kmsgq_t *uquad_kmsgq_init(int key_s, int key_c)
{
    uquad_kmsgq_t *server = (uquad_kmsgq_t *)malloc(sizeof(uquad_kmsgq_t));
    mem_alloc_check(server);
    server->k_s = key_s;
    server->k_c = key_c;
    server->acks_pend = 0;
    server->mbuf.mtype = UQUAD_MSGTYPE;
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
    return server;
}

int uquad_kmsgq_clear(uquad_kmsgq_t *server)
{
    int msqid, i, watchdog = 0;
    struct timeval time_aux;
    while(watchdog++ < UQUAD_KQ_MAX_CLEARS)
    {
	if ((msqid = msgget(server->k_c, 0666)) < 0)
	    break;
	/// Receive an answer of message type UQUAD_MSGTYPE.
	if (msgrcv(msqid, &server->mbuf, MSGSZ, UQUAD_MSGTYPE, IPC_NOWAIT) < 0)
	    break;
	/// Print the answer.
	gettimeofday(&time_aux,NULL);
	fprintf(server->s_log_data,"REMOVED:");
	for(i = 0; i < MSGSZ; ++i)
	    fprintf(server->s_log_data,"%d\t",(int)server->mbuf.mtext[i]);
	fprintf(server->s_log_data,"%d\n",(int)time_aux.tv_usec);
    }
    return ERROR_OK;
}

int uquad_kmsgq_get_ack(uquad_kmsgq_t *server)
{
    int msqid, i;
    struct timeval time_ack;
    if ((msqid = msgget(server->k_c, 0666)) < 0)
    {
	return ERROR_KQ_NO_ACKS_AVAIL;
    }

    /// Receive an answer of message type UQUAD_MSGTYPE.
    if (msgrcv(msqid, &server->mbuf, MSGSZ, UQUAD_MSGTYPE, IPC_NOWAIT) < 0)
    {
	return ERROR_KQ_NO_ACKS_AVAIL;
    }

    --server->acks_pend;
    if(server->acks_pend < 0)
    {
	err_log("ACK count lost!");
	server->acks_pend = 0;
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
    int retval = ERROR_OK, ack_cnt = 0;
    if(server->acks_pend == 0)
	return ERROR_OK;
    while(server->acks_pend > 0)
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
    if(ack_cnt == 1)
    {
	// ack received correctly
	retval = ERROR_OK;
    }
    else
    {
	// clean up
	retval = uquad_kmsgq_clear(server);
	server->acks_pend = 0;
	err_propagate(retval);
	if (ack_cnt == 0)
	{
	    // report data was not received by client
	    err_log("WARN: client did not ack!");
	    retval = ERROR_KQ_ACK_NONE;
	}
	else
	{
	    // report weird stuff comming
	    err_log_num("WARN: recieved too many acks from client! ACKs:",ack_cnt);
	    retval = ERROR_KQ_ACK_TOO_MANY;
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
	if(acks_not_recv++ > UQUAD_KQ_MAX_ACKS_MISSED)
	{
	    err_check(ERROR_KQ_ACK_NONE,"Missed too many acks!");
	}
	break;
    case ERROR_KQ_ACK_TOO_MANY:
	err_check(ERROR_KQ_ACK_TOO_MANY,"Client sent too many acks!");
	break;
    default:
	err_propagate(retval);
    }
    /// if here, then msg should be sent
    if ((msqid = msgget(server->k_s, server->msgflg )) < 0)
    {
	err_check(ERROR_KQ,"msqid failed!");
    }
    msg_len = uquad_min(msg_len, MSGSZ);
    for(i = 0; i < msg_len; ++i)
	server->mbuf.mtext[i] = msg[i];
    /// send msg
    if (msgsnd(msqid, &server->mbuf, msg_len, IPC_NOWAIT) < 0)
    {
	err_check(ERROR_KQ,"msgsnd failed!");
    }
    server->acks_pend++;
    /// log sent data
    gettimeofday(&tsent,NULL);
    fprintf(server->s_log_data,"%lu\t",server->tx_counter++);
    for(i = 0; i < msg_len; ++i)
	fprintf(server->s_log_data,"%d\t",(int)server->mbuf.mtext[i]);
    fprintf(server->s_log_data,"%d\n",(int)tsent.tv_usec);
    return ERROR_OK;
}


void uquad_kmsgq_deinit(uquad_kmsgq_t *server)
{
    int retval;
    retval = uquad_kmsgq_clear(server);
    if(retval != ERROR_OK)
    {
	err_log("WARN: Nothing to clear");
    }
    if(server->s_log_data != stderr)
	fclose(server->s_log_data);
    if(server->s_log_ack != stderr)
	fclose(server->s_log_ack);
    free(server);
}
