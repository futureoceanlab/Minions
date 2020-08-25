#include <stdlib.h>
#include <iostream>
#include "synchronization.h"


long long asNanosec(struct timespec *T)
{
    return ((long long) T->tv_sec) * BILLION + (long long) T->tv_nsec;
}

long long bytesToNanosec(char *buffer)
{
    // The host sends a little endian, second first then nanoseconds
    time_t sec = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];  
    int nsec = (buffer[7] << 24) | (buffer[6] << 16) | (buffer[5] << 8) | buffer[4];
    return ((long long) sec) * BILLION + (long long) nsec;
}

void asTimespec(long long t, struct timespec *T)
{
    T->tv_sec = (long) (t / BILLION);
    T->tv_nsec = (long) (t % BILLION);
    return;
}


int getTPSNData(int sock, struct timespec *T_data)
{
    // TPSN requires receiving T2 and T3 of the server time to compute the
    // clock difference. On top of that, we decided to encode some more
    // communication protocol using the variables T2 and T3. 
    // T2 is realistically bigger than 1. The protocol is as follows:
    // (T2, T3) = (>1, >0) = expected T2 and T3 from the server received
    // (T2, T3) = (1, ~) = TERMINATE
    // (T2, T3) = (0, 0) = STOP IMAGING: Failed communication
    // (T2, T3) = (0, >0) = STOP IMAGING:
    //                      Session not started, wait for T3 amount
    int valread;
    char buffer[16] = {0}; 

    struct timespec T1 = {.tv_sec = 0, .tv_nsec = 0};
    struct timespec T4 = {.tv_sec = 0, .tv_nsec = 0};
    long long T1n, T2n, T3n, T4n;
    long long T_data_n = 0;
    for (int i = 0; i < NUM_TPSN_AVG; i++) {
        clock_gettime(CLOCK_MONOTONIC, &T1);
        // convert time_t to byte array
        char *T1_arr = (char *) &T1; // RPI is 32-bit so time_t is 32bit long
        send(sock, T1_arr, 8, 0); 
        valread = read( sock , buffer, 16);
        if (valread != 16)
        {
            std::cout << valread << std::endl;
            printf("getting T3 and T4 from server failed\n");
            return -1;
        }
        // Receive T2 and T3
        T2n = bytesToNanosec(buffer);
        T3n = bytesToNanosec(buffer+8);
        if (T2n == TERMINATE)
        {
            return TERMINATE;
        }
        else if (T2n == 0)
        { 
            if (T3n != 0)
            {
                // We have not begun operation yet
                asTimespec(T3n, T_data);
            }
            return STOP_IMAGING;
        }
        // Timestamp T4
        clock_gettime(CLOCK_MONOTONIC, &T4);
        T1n = asNanosec(&T1);
        T4n = asNanosec(&T4);
        T_data_n += ((T2n - T1n) - (T4n - T3n));
    }
    // compute average time skew
    T_data_n /= (NUM_TPSN_AVG * 2);
    asTimespec(T_data_n, T_data);
    return 0;
}


int synchronize(struct timeinfo *TI, uint8_t is_sync)
{
    /* Setup a socket communication */
    int sock = 0; 
    struct sockaddr_in serv_addr; 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return -1; 
    } 

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
    close(sock);
        return -1; 
    } 
   

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
    close(sock);
        return -1; 
    } 

    /* Variable Configuration */
    char timeData[8];
    char status_buf[8] = {0};
    unsigned char ones[8] = {0xFF};
    char buffer[16] = {0}; 

    /* get TPSN data for synchroniztion, check if other signals provided */
    struct timespec T_start;
    struct timespec T_data = {.tv_sec=0, .tv_nsec=0};
    int tpsn_status = getTPSNData(sock, &T_data);
    if (tpsn_status == -1 || tpsn_status == TERMINATE || tpsn_status == STOP_IMAGING)
    {
        long long T_wait_n = asNanosec(&T_data);
        TI->T_wait_n = T_wait_n;
        close(sock);
        return tpsn_status;
    }
    long long T_skew_n = asNanosec(&T_data);
    TI->T_skew_n = T_skew_n;

    /* This was intended for skew calculation, stop here*/
    if (!is_sync)
    {
        close(sock);
        return 0;
    }

    /* Ping the server to about start and stop time */
    int start = 0, valread;
    long long temp_n=0, T_start_n = 0, T_stop_n=0;
    while (!T_start_n)
    {
        usleep(10);
        send(sock, status_buf, 8, 0);
        valread = read(sock, buffer, 16);
        temp_n = bytesToNanosec(buffer);
        if (temp_n > 1)
        {
            T_start_n = temp_n;
            T_stop_n = bytesToNanosec(buffer+8);
            break;
        }
    }
    close(sock);
    
    T_start_n -= T_skew_n;
    T_stop_n -= T_skew_n;
    // Start 1ms after the LED trigger time to account for the 
    // jitter in pulses (expected to be around 300us at most)
    TI->T_start_n = T_start_n + 1000000;
    TI->T_stop_n = T_stop_n;
    return 0;
}