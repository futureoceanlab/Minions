#include <stdlib.h>
#include "synchronization.h"


/**
 * as_nsec: convert timespec to long long (nsec)
 */
long long as_nsec(struct timespec *T)
{
    return ((long long) T->tv_sec) * BILLION + (long long) T->tv_nsec;
}

/**
 * bytes_to_nsec: convert 16-byte char array to long long (nsec)
 */
long long bytes_to_nsec(char *buffer)
{
    time_t sec = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];  
    int nsec = (buffer[7] << 24) | (buffer[6] << 16) | (buffer[5] << 8) | buffer[4];
    return ((long long) sec) * BILLION + (long long) nsec;
}

/**
 * as_timespec: convert long long to timespec
 */
void as_timespec(long long t, struct timespec *T)
{
    T->tv_sec = (long) (t / BILLION);
    T->tv_nsec = (long) (t % BILLION);
    return;
}
