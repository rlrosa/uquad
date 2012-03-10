#ifndef UQUAD_KERNEL_MSGQ_H
#define UQUAD_KERNEL_MSGQ_H

#include <uquad_error_codes.h>
#include <uquad_aux_time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define MSGSZ 4
#define UQUAD_MSGTYPE 1
#define UQUAD_KQ_MAX_ACKS 100
#define UQUAD_KQ_MAX_ACKS_MISSED 20
#define UQUAD_KQ_MAX_CLEARS 100

#define UQUAD_KQ_S_LOG_DATA "kq_s_data.log"
#define UQUAD_KQ_S_LOG_ACK "kq_s_ack.log"
#define UQUAD_KQ_ACK_STR "AKOK"

typedef struct msgbuf {
    long    mtype;
    uint8_t    mtext[MSGSZ];
}message_buf_t;

typedef struct uquad_kmsgq{
    message_buf_t mbuf;
    key_t k_s;     // server key
    key_t k_c;     // client key
    int msgflg;
    int acks_pend; // pending acks
    unsigned long tx_counter;
    FILE *s_log_data;   // server log
    FILE *s_log_ack;   // server log
}uquad_kmsgq_t;

uquad_kmsgq_t *uquad_kmsgq_init(int key_s, int key_c);

int uquad_kmsgq_send(uquad_kmsgq_t *server, uint8_t *msg, int msg_len);

void uquad_kmsgq_deinit(uquad_kmsgq_t *server);

#endif //UQUAD_KERNEL_MSGQ_H
