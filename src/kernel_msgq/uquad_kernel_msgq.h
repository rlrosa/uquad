/**
 * uquad_kernel_msgq: lib for IPC using kernel message queues.
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
 * @file   uquad_kernel_msgq.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for IPC using kernel message queues.
 *
 * Examples:
 *   - src/motor/mot_control.c
 *   - src/test/kernel_msgq/message_send.c
 *   - src/test/kernel_msgq/message_rec.c
 */
#ifndef UQUAD_KERNEL_MSGQ_H
#define UQUAD_KERNEL_MSGQ_H

#include <uquad_error_codes.h>
#include <uquad_aux_time.h>
#include <uquad_config.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#if DEBUG
#define LOG_KQ_S_DATA 0
#define LOG_KQ_S_ACK  0
#endif


#define MSGSZ                    4
#define UQUAD_MSGTYPE            1L
#define UQUAD_KQ_WARN_ACKS       0   // # of errors to allow before logging errors
#define UQUAD_KQ_MAX_ACKS        100
#define UQUAD_KQ_MAX_ACKS_MISSED 20
#define UQUAD_KQ_MAX_CLEARS      100

#define UQUAD_KQ_S_LOG_DATA      "kq_s_data.log"
#define UQUAD_KQ_S_LOG_ACK       "kq_s_ack.log"
#define UQUAD_KQ_ACK_STR         "AKOK"

typedef struct msgbuf {
    long    mtype;
    uint8_t    mtext[MSGSZ];
}message_buf_t;

typedef struct uquad_kmsgq{
    message_buf_t mbuf;
    key_t k_s;       // server key
    key_t k_c;       // client key
    int msgflg;
    int acks_pend;   // pending acks
    int acks_failed; // Count of acks errors (too many/not enough)
    unsigned long tx_counter;
#if LOG_KQ_S_DATA
    FILE *s_log_data;   // server log
#endif
#if LOG_KQ_S_ACK
    FILE *s_log_ack;   // server log
#endif
}uquad_kmsgq_t;

uquad_kmsgq_t *uquad_kmsgq_init(int key_s, int key_c);

int uquad_kmsgq_send(uquad_kmsgq_t *server, uint8_t *msg, int msg_len);

void uquad_kmsgq_deinit(uquad_kmsgq_t *server);

#endif //UQUAD_KERNEL_MSGQ_H
