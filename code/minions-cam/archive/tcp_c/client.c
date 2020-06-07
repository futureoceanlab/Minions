// Client side C/C++ program to demonstrate Socket programming 
#include <stdio.h> 
#include <stdlib.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <signal.h>
#include <wiringPi.h>

#define PORT 8080 

#define BILLION 1000000000

#define TRIG_PIN 24


uint32_t counter = 0;

void handler(int signo)
{
    digitalWrite(TRIG_PIN, HIGH);
    digitalWrite(TRIG_PIN, LOW);
    printf("Client: tick\n");
    counter++;
    if (counter > 100)
    {
        exit(0);
    }
}

void timeSubtract(struct timespec* Ta, struct timespec* Tb, struct timespec* Tc)
{
    // subtract b from a
    // long sec = Tb->tv_sec - Ta->tv_sec;
    // long nsec = Tb->tv_nsec - Ta->tv_nsec;
    // if (sec > 0 && nsec < 0)
    // {
    //     nsec = BILLION + nsec;
    //     sec -= 1;
    // } 
    // else if (sec < 0 && nsec > 0)
    // {
    //     sec += 1;
    //     nsec = BILLION - nsec;
    // }
    // Tc->tv_sec = sec;
    // Tc->tv_nsec = nsec;
    struct Tb_m = {.tv_sec = -Tb->tv_sec, .tv_nsec=-Tb->tv_nsec };
    timeAdd(Ta, &Tb_m, Tc);
    return;
}


void timeAdd(struct timespec* Ta, struct timespec* Tb, struct timespec* Tc)
{
    // add a and b
    long sec = Tb->tv_sec + Ta->tv_sec;
    long nsec = Tb->tv_nsec + Ta->tv_nsec;
    if (nsec > BILLION)
    {
        nsec -= BILLION;
        sec += 1;
    }
    else if (nsec < -BILLION)
    {
        nsec += BILLION;
        sec -= 1;
    }
    Tc->tv_sec = sec;
    Tc->tv_nsec = nsec;
    return;
}

inline uint64_t as_nanoseconds(struct timespec* ts) {
    return ts->tv_sec * (uint64_t)1000000000L + ts->tv_nsec;
}

void timeDivide(struct timespec* T, int divisor)
{
    T->tv_nsec = T->tv_nsec/divisor;
    if ((T->tv_sec % divisor) == 0)
    {
        if (T->tv_nsec < 0)
        {
            T->tv_nsec -= 500000000;
        }
        else 
        {
            T->tv_nsec += 500000000;
        }
    }
    T->tv_sec = T->tv_sec/divisor;
    return;
}

int main(int argc, char const *argv[]) 
{   
    wiringPiSetup();
    pinMode(TRIG_PIN, OUTPUT);

    int sock = 0, valread; 
    struct sockaddr_in serv_addr; 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return -1; 
    } 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, "192.168.4.1", &serv_addr.sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
        return -1; 
    } 
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
        return -1; 
    } 
    char timeData[8];
    char buffer[16] = {0}; 

    struct timespec T1 = {.tv_sec = 0, .tv_nsec = 0};
    struct timespec T4 = {.tv_sec = 0, .tv_nsec = 0};

    clock_gettime(CLOCK_REALTIME, &T1);
    // convert time_t to byte array
    char *T1_arr = (char *) &T1; // RPI is 32-bit so time_t is 32bit long
    // char *T1_sec = (char *) &(T1.tv_sec);
    // char *T1_nsec = (char *) &(T1.tv_nsec);
    // for (int i = 0; i < 4; i++) {
    //     timeData[i] = T1_sec[i];
    //     timeData[4+i] = T1_nsec[i];
    // }
    send(sock, T1_arr, 8, 0); 
    printf("T1 sent"); 

    valread = read( sock , buffer, 16); 
    // Timestamp T4
    clock_gettime(CLOCK_REALTIME, &T4);
    // Receive T2 and T3
    time_t T2_sec_i = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];  
    int T2_nsec_i = (buffer[7] << 24) | (buffer[6] << 16) | (buffer[5] << 8) | buffer[4];
    time_t T3_sec_i = (buffer[11] << 24) | (buffer[10] << 16) | (buffer[9] << 8) | buffer[8];  
    int T3_nsec_i = (buffer[15] << 24) | (buffer[14] << 16) | (buffer[13] << 8) | buffer[12];
    struct timespec T_diff21, T_diff43, T_skew, T3_client, T_start;
    struct timespec T_delay = {.tv_sec = 2, .tv_nsec = 0};
    struct timespec T2 = {.tv_sec = T2_sec_i, .tv_nsec=T2_nsec_i};
    struct timespec T3 = {.tv_sec = T3_sec_i, .tv_nsec=T3_nsec_i};
    timeSubtract(&T2, &T1, &T_diff21);
    timeSubtract(&T4, &T3, &T_diff43);
    timeSubtract(&T_diff21, &T_diff43, &T_skew);
    timeDivide(&T_skew, 2);

    // Server is ahead of the client by T_skew (positive)
    // Want to set timer to start T3 - T_skew + start_delay
    timeSubtract(&T3, &T_skew, &T3_client);
    timeAdd(&T3_client, &T_delay, &T_start);
    printf("skew: %d.%d\n", T_start.tv_sec, T_start.tv_nsec);

    timer_t t_id;
    struct itimerspec tim_spec = {.it_interval= {.tv_sec=1,.tv_nsec=0},
                    .it_value = T_start};

    struct sigaction act;
    sigset_t set;

    sigemptyset( &set );
    sigaddset( &set, SIGALRM );

    act.sa_flags = 0;
    act.sa_mask = set;
    act.sa_handler = &handler;

    sigaction( SIGALRM, &act, NULL );

    if (timer_create(CLOCK_REALTIME, NULL, &t_id))
        perror("timer_create");
    
    if (timer_settime(t_id, TIMER_ABSTIME, &tim_spec, NULL))
        perror("timer_settime"); 
    while(1);
    
    return 0; 
} 

