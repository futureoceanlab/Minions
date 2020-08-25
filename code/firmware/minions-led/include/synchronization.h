/**
 * synchronization: a set of  client functions to help TPSN synchronization 
 * with the server
 * 
 * Author: Junsu Jang (junsuj@mit.edu)
 * Date: Aug 25, 2020
 * 
 */
#ifndef SYNC_H
#define SYNC_H

#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <sys/types.h>
#include <signal.h>
#include <time.h>

#include <network.h>

#define BILLION 1000000000LL
#define NUM_TPSN_AVG 25

#define TERMINATE 1
#define STOP_IMAGING 2 

/**
 * timeinfo: custom struct to hold time values relevant to communication and
 * synchronization with the server. 
 * 
 * T_skew_n: skew between client and server in nanoseconds
 * T_start_n: time (server) at which the next trigger occurs
 * T_stop_n: time (server) at which the current session will end
 * T_wait_n: waiting duration until the next session start
 */
struct timeinfo
{
    long long T_skew_n;
    long long T_start_n;
    long long T_stop_n;
    long long T_wait_n;
};

/* Convert timespec to nanoseconds (long long) */
long long asNanosec(struct timespec *T);
/* Convert the incoming byte array into nanoseconds */
long long bytesToNanosec(char *buffer);
/* Convert long long nanoseconds to timespec */
void asTimespec(long long t, struct timespec *T);
/* synchronize the time with the server */
int synchronize(struct timeinfo* TI, uint8_t is_first);

#endif
