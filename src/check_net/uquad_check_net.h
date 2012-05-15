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
 * Will launch a UDP server listening portno.
 * Should be forked or launched as in server.c, since
 * it will not (except if an error occurs) stop running.
 *
 * @param portno Port on which server should listen
 * @param tcp Use a connection oriented socket (TCP)
 *
 * @return error code
 */
int uquad_check_net_server(int portno, uquad_bool_t tcp);

/**
 * Will launch a client that will periodically ping a server.
 * If does not recieve any answers from the server, it will die
 * without doing anything. If at least 1 ack is received, then
 * if connection is lost, it will kill the motor driver.
 *
 * @param hostIP IP of the host where the server is running.
 * @param portno Port used by the server.
 * @param tcp Use a connection oriented socket (TCP)
 *
 * @return error code
 */
int uquad_check_net_client(const char *hostIP, int portno, uquad_bool_t tcp);

#endif // UQUAD_CHECK_NET_H
