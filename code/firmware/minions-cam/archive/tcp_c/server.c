// Server side C/C++ program to demonstrate Socket programming 
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 

#include <time.h>
#include <sys/types.h>
#include <stdlib.h>

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
    Tc->tv_sec = sec;
    Tc->tv_nsec = nsec;
    return;
}

int main(int argc, char const *argv[]) 
{ 
    wiringPiSetup();
    pinMode(TRIG_PIN, OUTPUT);
    int server_fd, new_socket, valread; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    char buffer[8] = {0}; 
    char *hello = "Hello from server"; 
       
    // Creating socket file descriptor 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    // Forcefully attaching socket to the port 8080 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 
       
    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(server_fd, 3) < 0) 
    { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,  
                       (socklen_t*)&addrlen))<0) 
    { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    } 
    valread = read( new_socket , buffer, 8); 
    struct timespec T2 = {.tv_sec = 0, .tv_nsec = 0};
    struct timespec T3 = {.tv_sec = 0, .tv_nsec = 0};
    clock_gettime(CLOCK_REALTIME, &T2);
    usleep(5000);
    clock_gettime(CLOCK_REALTIME, &T3);
    
    char *T2_arr = (char *) &(T2.tv_sec);
    char *T3_arr = (char *) &(T3.tv_sec);
    char timeData[16] = {0};
    for (int i = 0; i < 8; i++) {
        timeData[i] = T2_arr[i];
        timeData[8+i] = T3_arr[i];
    }
    send(new_socket , timeData , 16 , 0 );
    printf("T2 & T3 message sent\n"); 
    
    // Server is ahead of the client by T_skew (positive)
    // Want to set timer to start T3 start_delay
    struct timespec T_start;
    struct timespec T_delay = {.tv_sec = 2, .tv_sec = 0};
    timeAdd(&T3, &T_delay, &T_start);
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