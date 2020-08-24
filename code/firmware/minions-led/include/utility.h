#ifndef UTILITY_H
#define UTILITY_H

#include <stdio.h>  
#include <string.h>         //strlen  
#include <stdlib.h>  
#include <iostream>
#include <errno.h>  
#include <unistd.h>         //close  
#include <signal.h>         // timer interrupt

#include <sys/time.h>       // FD_SET, FD_ISSET, FD_ZERO macros  
#include <time.h>           // time_sepc

std::string runScript(std::string command);
int makeTimer(timer_t *timerID, struct timespec *T_timer_start, int it_sec, int it_nsec, void (*handler)(int, siginfo_t*, void*));
#endif
