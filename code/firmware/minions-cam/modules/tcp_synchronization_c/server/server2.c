/**
 *   server: module component to test time synchronization 
 *   May 24, 2020
 *   Authors: Junsu Jang, FOL/MIT
 *      Description: 
 *   
 *   This server code is a tentative module to understand the feasilibity
 *   of wireless time synchronization using RPI Zero W. Our worry was that
 *   given the low performance of RPI, the synchroinzation might be difficult
 *   with the wireless communication. 
 * 
 *   This code opens a socket for TCP communication. It uses select to allow
 *   multiple clients to connect and communicate with the server in series. 
 *   The reference code is from 
 *   https://www.geeksforgeeks.org/socket-programming-in-cc-handling
 *   -multiple-clients-on-server-without-multi-threading/
 * 
 *   For our application, we assume at most two clients (cameras). This code
 *   is meant to be used by the LED RPI.
 * 
 *   Once the network is ready, the clients and the server talk in the following
 *   procedure:
 *      .stepA
 *          - Clients A + B sends their time T1, and server responsds with its 
 *          T2 and T3. 
 *          - Repeat .stepA until all clients sends 0 in data buffer. 
 *          This menas that the clients have accumulated sufficient number 
 *          of data for averaging (100).
 * 
 *      .stepB  
 *          - all clients are done averaging --> server sends time to start
 *          trigger cameras
 *          - clients acknowledges with response data "1"
 * 
 *      .stepC
 *          - sockets are closed and LED strobing begins
 * 
 * 
 *      
 */


#include <stdio.h>  
#include <string.h>         //strlen  
#include <stdlib.h>  
#include <errno.h>  
#include <unistd.h>         //close  
#include <arpa/inet.h>      //close  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <sys/time.h>       // FD_SET, FD_ISSET, FD_ZERO macros  
#include <time.h>           // time_sepc
#include <wiringPi.h>       // for GPIO
#include <signal.h>         // for 1 second interrupt
     
#define PORT 8080               // Port number for communication
#define BILLION 1000000000LL    // nanoseconds conversion
#define TRIG_PIN 24             // pin number for LED trigger


/**
 * Code that runs at timer interrupt
 */
void handler(int signo)
{
    digitalWrite(TRIG_PIN, HIGH);
    // wait for very brief cycles of lopps
    for (int i =0; i < 1000; i++) {}
    digitalWrite(TRIG_PIN, LOW);
}


/**
 * Conversion from timespec to 64-bit long nanoseconds for easy arithematic
 */
long long asNanosec(struct timespec *T)
{
    return ((long long) T->tv_sec)*BILLION + (long long) T->tv_nsec;
}


/**
 * Convert bytes received by clients into 64-bit long nanoseconds
 */
long long bytesToNanosec(char *buffer)
{
    time_t sec = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
    int nsec = (buffer[7] << 24) | (buffer[6] << 16) | (buffer[5] << 8) | buffer[4];
    return ((long long) sec) * BILLION + (long long) nsec;
}

/**
 * Convert 64-bit long nanoseconds into timespec
 */
void asTimespec(long long t, struct timespec *T)
{
    T->tv_sec = (long) (t/BILLION);
    T->tv_nsec = (long) (t % BILLION);
    return;
}     


/**
 * Entrance to the entire code. 
 */
int main(int argc , char *argv[])   
{   
    // Set up wiring Pi for controlling the RPI
    wiringPiSetup();
    pinMode(TRIG_PIN, OUTPUT);

    int opt = 1;   
    int master_socket , addrlen , new_socket , client_socket[30] ,  
          max_clients = 2 , activity, i , valread , sd;   
    int max_sd;   
    struct sockaddr_in address;   
         
    char buffer[1025];  //data buffer of 1K  
         
    //set of socket descriptors  
    fd_set readfds;   
         
    //a message  
    char *message = "ECHO Daemon v1.0 \r\n";   
     
    //initialise all client_socket[] to 0 so not checked  
    for (i = 0; i < max_clients; i++)   
    {   
        client_socket[i] = 0;   
    }   
         
    //create a master socket  
    if( (master_socket = socket(AF_INET , SOCK_STREAM , 0)) == 0)   
    {   
        perror("socket failed");   
        exit(EXIT_FAILURE);   
    }   
     
    //set master socket to allow multiple connections ,  
    //this is just a good habit, it will work without this  
    if( setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt,  
          sizeof(opt)) < 0 )   
    {   
        perror("setsockopt");   
        exit(EXIT_FAILURE);   
    }   
     
    //type of socket created  
    address.sin_family = AF_INET;   
    address.sin_addr.s_addr = INADDR_ANY;   
    address.sin_port = htons( PORT );   
         
    //bind the socket to localhost port 8888  
    if (bind(master_socket, (struct sockaddr *)&address, sizeof(address))<0)   
    {   
        perror("bind failed");   
        exit(EXIT_FAILURE);   
    }   
    printf("Listener on port %d \n", PORT);   
         
    //try to specify maximum of 3 pending connections for the master socket  
    if (listen(master_socket, 3) < 0)   
    {   
        perror("listen");   
        exit(EXIT_FAILURE);   
    }   
         
    //accept the incoming connection  
    addrlen = sizeof(address);   
    puts("Waiting for connections ...");   

    // Starting time of the trigger that is sent to clients eventually
    struct timespec T_start = {.tv_sec=0, .tv_nsec=0};
    // T2 and T3 for synchronization
    struct timespec T2, T3;

    // simple status indication array for each client
    // 0: time is not informed
    // 1: start time is sent
    // 2. start time is acknowledged and ready to trigger
    uint8_t sync[2] = {0};    

    // Handle network until both cameras have responded saying that they are
    // synchronized and ready to trigger cameras.
    while(sync[0] != 2 || sync[1] != 2)   
    {   
        //clear the socket set  
        FD_ZERO(&readfds);   
     
        //add master socket to set  
        FD_SET(master_socket, &readfds);   
        max_sd = master_socket;   
             
        //add child sockets to set  
        for ( i = 0 ; i < max_clients ; i++)   
        {   
            //socket descriptor  
            sd = client_socket[i];   
                 
            //if valid socket descriptor then add to read list  
            if(sd > 0)   
                FD_SET( sd , &readfds);   
                 
            //highest file descriptor number, need it for the select function  
            if(sd > max_sd)   
                max_sd = sd;   
        }   
     
        //wait for an activity on one of the sockets , timeout is NULL ,  
        //so wait indefinitely  
        activity = select( max_sd + 1 , &readfds , NULL , NULL , NULL);   
       
        if ((activity < 0) && (errno!=EINTR))   
        {   
            printf("select error");   
        }   
        //If something happened on the master socket ,  
        //then its an incoming connection  
        if (FD_ISSET(master_socket, &readfds))   
        {   
            if ((new_socket = accept(master_socket,  
                    (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)   
            {   
                perror("accept");   
                exit(EXIT_FAILURE);   
            }   
             
            //inform user of socket number - used in send and receive commands  
            printf("New connection , socket fd is %d , ip is : %s , port : %d\n", 
                new_socket , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));   

            //add new socket to array of sockets  
            for (i = 0; i < max_clients; i++)   
            {   
                //if position is empty  
                if( client_socket[i] == 0 )   
                {   
                    client_socket[i] = new_socket;   
                    printf("Adding to list of sockets as %d\n" , i);   
                         
                    break;   
                }   
            }   
        }   
        //else its some IO operation on some other socket 
        for (i = 0; i < max_clients; i++)   
        {   
            sd = client_socket[i];   
                 
            if (FD_ISSET( sd , &readfds))   
            {   
                //Check if it was for closing , and also read the  
                //incoming message  
                if ((valread = read( sd , buffer, 1024)) == 0)   
                {   
                    //Somebody disconnected , get his details and print  
                    getpeername(sd , (struct sockaddr*)&address, (socklen_t*)&addrlen);   
                    printf("Host disconnected , ip %s , port %d \n",  
                          inet_ntoa(address.sin_addr) , ntohs(address.sin_port));   
                         
                    //Close the socket and mark as 0 in list for reuse  
                    close( sd );   
                    client_socket[i] = 0;   
                }
                // handle synchronization 
                else 
                {   
                    clock_gettime(CLOCK_REALTIME, &T2);
                    long long rec_T = bytesToNanosec(buffer);
                    // clients are responding with its T1 value to receive
                    // more time information for synchronization
                    if (rec_T > 1) 
                    {
                        usleep(1000);
                        clock_gettime(CLOCK_REALTIME, &T3);
                        
                        char timeData[16] = {0};
                        char *T2_arr = (char *) &(T2.tv_sec);
                        char *T3_arr = (char *) &(T3.tv_sec);
                        for (int i = 0; i < 8; i++) {
                            timeData[i] = T2_arr[i];
                            timeData[8+i] = T3_arr[i];
                        }
                        send(sd, timeData, 16, 0);   
                    }
                    // client sends 0 to let server know that sychronization is
                    // complete and it is ready to receive trigger start time.
                    // convert the status to 1.
                    else if (rec_T == 0)
                    {
                        sync[i] = 1;
                        // Only when both clients are ready does the server
                        // set the start time of the trigger
                        if (sync[0] == 1 && sync[1] == 1)
                        {
                            if (T_start.tv_sec == 0 && T_start.tv_nsec == 0)
                            {
                                long long T2n = asNanosec(&T2);
                                T2n += 2*BILLION;
                                asTimespec(T2n, &T_start);
                            }
                        }
                        send(sd, (char *) &T_start, 8, 0);
                    }
                    // client is ready to trigger cameras, convert the status
                    // to 2
                    else if (rec_T == 1)
                    {
                        sync[i] = 2;
                        char timeData[8] = {0};
                        timeData[4] = 1;
                        send(sd, timeData, 8, 0); 
                    }
                }
            }   
        }   
    }   
    
    // Create a repeated timer interrupt that happens every second starting at
    // T_start
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
