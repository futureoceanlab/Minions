#include <stdlib.h>
#include <iostream>
#include "synchronization.h"


long long asNanosec(struct timespec *T)
{
    return ((long long) T->tv_sec) * BILLION + (long long) T->tv_nsec;
}

long long bytesToNanosec(char *buffer)
{
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

long long get_TPSN_data(int sock)
{
    int valread;
    char buffer[16] = {0}; 

    struct timespec T1 = {.tv_sec = 0, .tv_nsec = 0};
    struct timespec T4 = {.tv_sec = 0, .tv_nsec = 0};
    long long T1n, T2n, T3n, T4n;
    long long T_skew_n = 0;
    for (int i = 0; i < NUM_AVG; i++) {
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
        // Timestamp T4
        clock_gettime(CLOCK_MONOTONIC, &T4);
        // Receive T2 and T3
        T2n = bytesToNanosec(buffer);
        T3n = bytesToNanosec(buffer+8);
        //time_t T3_sec_i = (buffer[11] << 24) | (buffer[10] << 16) | (buffer[9] << 8) | buffer[8];  
        //int T3_nsec_i = (buffer[15] << 24) | (buffer[14] << 16) | (buffer[13] << 8) | buffer[12];
        //struct timespec T2 = {.tv_sec = T2_sec_i, .tv_nsec=T2_nsec_i};
        //T3.tv_sec = T3_sec_i;
        //T3.tv_nsec = T3_nsec_i;
        T1n = asNanosec(&T1);
        //T2n = asNanosec(&T2);
        //T3n = asNanosec(&T3);
        T4n = asNanosec(&T4);
        T_skew_n += ((T2n - T1n) - (T4n - T3n));
    }
    // compute average time skew
    T_skew_n /= (NUM_AVG * 2);
    return T_skew_n;
}

int synchronize(struct timeinfo *TI, uint8_t is_first)
{
    int sock = 0; 
    struct sockaddr_in serv_addr; 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return -1; 
    } 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr)<=0)  
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
    char status_buf[8] = {0};
    char ones[8] = {0xFF};
    char buffer[16] = {0}; 

    struct timespec T_start;
    struct timespec T_skew;
    long long T_skew_n = get_TPSN_data(sock);
    asTimespec(T_skew_n, &T_skew);
    printf("%lld skew: %d.%d\n", T_skew_n, T_skew.tv_sec, T_skew.tv_nsec);

    // Ping the server to about start time
    int start = 0, valread;
    long long temp_n=0, T_start_n = 0;
    while (!start)
    {
        usleep(10);
        send(sock, status_buf, 8, 0);
        valread = read(sock, buffer, 16);
        if (valread == 0 && status_buf[4] == 1)
        {
            // the server has moved onto timer, so we will break
            break;
        }
        temp_n = bytesToNanosec(buffer);
        //printf("%lld\n", temp_n);
        if (temp_n > 1)
        {
            T_start_n = temp_n;
            if (is_first) status_buf[4] = 1;
        }
        start = (temp_n == 1) || !is_first;
    }
    close(sock);
    
//    printf("start: %lld\n", T_start_n);
    T_start_n -= T_skew_n;
    TI->T_skew_n = T_skew_n;
    TI->T_start_n = T_start_n + 1000000;
    //struct timespec T_delay = {.tv_sec = 5, .tv_nsec = 0};
    //long long T_delay_n = asNanosec(&T_delay);
    //long long T_start_n = T3n - T_skew_n + T_delay_n;
    // asTimespec(T_start_n, &T_start);
}

// TODO: FIgure out

int getSkew(struct timeinfo* TI)
{
    int sock = 0; 
    struct sockaddr_in serv_addr; 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return -1; 
    } 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
        return -1; 
    } 

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
        return -1; 
    }
    

    // Compute the skew upon averaging using TPSN
    struct timespec T_start;
    struct timespec T_skew;
    long long T_skew_n = get_TPSN_data(sock);
    //asTimespec(T_skew_n, &T_skew);
    TI->T_skew_n = T_skew_n;
    close(sock);
    // // Ping the server to about next trigger time
    // // send 2
    // char status_buf[8] = {0};
    // status_buf[4] = 2;
    // char buffer[16] = {0}; 

    // int start = 0, valread;
    // long long T_start_n = 0;
    // send(sock, status_buf, 8, 0);
    // valread = read(sock, buffer, 16);
    // if (valread != 16) {
    //     return -1;
    // }
    // T_start_n = bytesToNanosec(buffer);

    close(sock);

    return 0;
}
