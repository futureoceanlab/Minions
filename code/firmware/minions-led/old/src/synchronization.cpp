#include <stdlib.h>
#include "synchronization.h"


/**
 * asNanosec: convert timespec to long long (nsec)
 */
long long asNanosec(struct timespec *T)
{
    return ((long long) T->tv_sec) * BILLION + (long long) T->tv_nsec;
}

/**
 * bytesToNanosec: convert 16-byte char array to long long (nsec)
 */
long long bytesToNanosec(char *buffer)
{
    time_t sec = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];  
    int nsec = (buffer[7] << 24) | (buffer[6] << 16) | (buffer[5] << 8) | buffer[4];
    return ((long long) sec) * BILLION + (long long) nsec;
}

/**
 * asTimespec: convert long long to timespec
 */
void asTimespec(long long t, struct timespec *T)
{
    T->tv_sec = (long) (t / BILLION);
    T->tv_nsec = (long) (t % BILLION);
    return;
}
