#include <stdio.h> 
#include <stdlib.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h>
#include <time.h>
#include <sys/types.h>

int main()
{
    char buffer1[80];
    char buffer2[80];
    char timeData[8];
    // int i = 0;
    // timer_t t_id;

    // struct itimerspec tim_spec = {.it_interval= {.tv_sec=0,.tv_nsec=500000},
    //                 .it_value = {.tv_sec=1,.tv_nsec=0}};

    // if (timer_create(CLOCK_REALTIME , NULL, &t_id))
    //     perror("timer_create");
    struct timespec T1 = {.tv_sec = 0, .tv_nsec = 0};
    struct timespec T2 = {.tv_sec = 0, .tv_nsec = 0};

    clock_gettime(CLOCK_REALTIME , &T1);

    char *T1_sec = (char *) &(T1);
    char *T1_nsec = (char *) &(T1.tv_nsec);
    printf("%lu\t%lu\n", T1_sec,(void *)&T1.tv_nsec);
    for (int i = 0; i < 4; i++) {
        printf("%d\n", sizeof(T1.tv_sec));
    }
    struct tm *gt = gmtime(&T1.tv_sec);
    strftime(buffer1,80,"%x - %I:%M:%S %p", gt);

    // // convert time_t to byte array
    // time_t T1_sec_i = (T1_sec[3] << 24) | (T1_sec[2] << 16) | (T1_sec[1] << 8) | T1_sec[0];  
    // int T1_nsec_i = (T1_nsec[3] << 24) | (T1_nsec[2] << 16) | (T1_nsec[1] << 8) | T1_nsec[0];

    // printf("%d\n", T1.tv_nsec - T1_nsec_i);
    // printf("%d\n", timeData[2]);
//     struct tm *gt2 = gmtime(&T1_sec_i);
//     strftime(buffer2,80,"%x - %I:%M%p", gt2);
//     // for (int i = 0; i < 4; i++) {
//     //     printf("%c ", T1_sec[i]);
//     // }
    sleep(5);
    clock_gettime(CLOCK_REALTIME , &T2);
    struct tm *gt2 = gmtime(&T2.tv_sec);
    strftime(buffer2,80,"%x - %I:%M:%S %p", gt2);
  printf("T1 orig: |%s|\n", buffer1);
  printf("T1 conv: |%s|\n", buffer2);
    return 0;
}