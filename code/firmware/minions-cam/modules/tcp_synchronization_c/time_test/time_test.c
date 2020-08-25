#include <stdio.h> 
#include <stdlib.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h>
#include <time.h>
#include <sys/types.h>

#define BILLION 1000000000

long long asNanosec(struct timespec *T)
{
    return ((long long) T->tv_sec)*BILLION + (long long) T->tv_nsec;
}

void asTimespec(long long t, struct timespec *T)
{
    T->tv_sec = (long) (t/BILLION);
    T->tv_nsec = (long) (t % BILLION);
}

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

    char *T1_sec = (char *) &(T1.tv_sec);
    char *T1_nsec = (char *) &(T1.tv_nsec);
    printf("%lu\t%lu\n", T1_sec,(void *)&T1.tv_nsec);
    for (int i = 0; i < 4; i++) {
        printf("%d\n", sizeof(T1.tv_sec));
    }
    struct tm *gt = gmtime(&T1.tv_sec);
    strftime(buffer1,80,"%x - %I:%M:%S %p", gt);

    // // convert time_t to byte array
    time_t T1_sec_t = (T1_sec[3] << 24) | (T1_sec[2] << 16) | (T1_sec[1] << 8) | T1_sec[0];  
    int T1_nsec_t = (T1_nsec[3] << 24) | (T1_nsec[2] << 16) | (T1_nsec[1] << 8) | T1_nsec[0];
    int T1_sec_i = (int) T1_sec_t;
    long long T1_sec_ll = asNanosec(&T1);
//    T1_sec_ll = T1_sec_ll*BILLION + (long long) T1.tv_nsec;
    printf("%d %d %d %lld\n", T1_sec_t, T1_sec_t, T1_nsec_t, T1_sec_ll*1000000000 + T1_nsec_t);
    // printf("%d\n", T1.tv_nsec - T1_nsec_i);
    // printf("%d\n", timeData[2]);
//     struct tm *gt2 = gmtime(&T1_sec_i);
//     strftime(buffer2,80,"%x - %I:%M%p", gt2);
//     // for (int i = 0; i < 4; i++) {
//     //     printf("%c ", T1_sec[i]);
//     // }
    sleep(5);
    clock_gettime(CLOCK_REALTIME , &T2);
    long long T2_sec_ll = (long long) T2.tv_sec;
    T2_sec_ll = T2_sec_ll * 1000000000 + (long long) T2.tv_nsec;

    printf("%lld\n", T1_sec_ll - T2_sec_ll);
    printf("%lld, nsec= %d\n", T2_sec_ll/1000000000, T2.tv_nsec);
    struct timespec T3;
    asTimespec(T2_sec_ll, &T3);
    printf("T3: %d %d\n", (long) T3.tv_sec, T3.tv_nsec);

    struct tm *gt2 = gmtime(&T2.tv_sec);
    strftime(buffer2,80,"%x - %I:%M:%S %p", gt2);
    printf("T1 orig: |%s|\n", buffer1);
    printf("T1 conv: |%s|\n", buffer2);
    return 0;
}
