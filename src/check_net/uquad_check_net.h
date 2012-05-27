/**
 * check_net: Connectivity monitoring library.
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
 * @file   uquad_check_net.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  library to allow detection of connectivity issues.
 *
 * Will establish a TCP or UDP connection between a server and a client. The client will
 * send a message to the server every CHECK_NET_MSG_T_MS milliseconds, and die if an ack is
 * not received within CHECK_NET_TO_S seconds.
 *
 * Examples:
 *   - src/check_net/client.c
 *   - src/check_net/server.c
 *   - src/main/main.c
 */
#ifndef UQUAD_CHECK_NET_H
#define UQUAD_CHECK_NET_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <mot_control.h> // for KILL_MOTOR_CMD
#include <uquad_types.h>
#include <uquad_aux_time.h>
#include <uquad_aux_io.h>

#if DEBUG
//#define DEBUG_CHECK_NET
#endif

/**
 * In a worst case scenario, check_net will take a max
 * of CHECK_NET_TO_S [s] + CHECK_NET_MSG_T_MS [ms] to
 * detect loss of connectivity.
 */
#define CHECK_NET_ACK          "OK"
#define CHECK_NET_PING         "HI"
#define CHECK_NET_KILL_RETRIES 10
#define CHECK_NET_MSG_LEN      2    // [bytes]
#define CHECK_NET_MSG_T_MS     950  // [ms] - time between pings
#define CHECK_NET_RETRY_MS     50   // [ms] - time between requests for ack
#define CHECK_NET_TO_S         1    // [s]  - timeout
#define CHECK_NET_SERVER_IP    "10.42.43.1"
#define CHECK_NET_PORT         12341234

/**
 * Will launch a UDP server listening portno.
 * Should be forked or launched as in server.c, since
 * it will not (except if an error occurs) stop running.
 *
 * @param portno Port on which server should listen
 * @param udp If true, use UDP, else TCP
 *
 * @return error code
 */
int uquad_check_net_server(int portno, uquad_bool_t udp);

/**
 * Will launch a client that will periodically ping a server.
 * If does not recieve any answers from the server, it will die
 * without doing anything. If at least 1 ack is received, then
 * if connection is lost, it will kill the motor driver.
 *
 * @param hostIP IP of the host where the server is running.
 * @param portno Port used by the server.
 * @param udp If true, use UDP, else TCP
 *
 * @return error code
 */
int uquad_check_net_client(const char *hostIP, int portno, uquad_bool_t udp);

#endif // UQUAD_CHECK_NET_H
