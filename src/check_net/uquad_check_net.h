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
 *
 * @return error code
 */
int uquad_check_net_server(int portno);

/**
 * 
 *
 * @param hostIP 
 * @param portno 
 *
 * @return error code
 */
int uquad_check_net_client(const char *hostIP, int portno);

#endif // UQUAD_CHECK_NET_H
