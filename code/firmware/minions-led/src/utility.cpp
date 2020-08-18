#include "utility.h"
#include <cctype>
#include <algorithm>

bool isNotAlnum(char c);

// Run a bash script on c++ and deliver the returned string
std::string run_script(std::string command) {
   char buffer[128];
   std::string result = "";

   // Open pipe to file
   FILE* pipe = popen(command.c_str(), "r");
   if (!pipe) {
      return "popen failed!";
   }

   // read till end of process:
   while (!feof(pipe)) {

      // use buffer to read and add to result
      if (fgets(buffer, 128, pipe) != NULL)
         result += buffer;
   }
   result.erase(std::remove_if(result.begin(), result.end(), isNotAlnum), result.end());
   pclose(pipe);
   return result;
}

// Wrapper function for !isalnum
bool isNotAlnum(char c)
{
    return std::isalnum(c) == 0;
}

//  General timer creater
int makeTimer(timer_t *timerID, struct timespec *T_timer_start, int it_sec, int it_nsec, void (*handler)(int, siginfo_t*, void*))
{
    // varaibles
    struct sigaction act;
    struct sigevent te;
    struct itimerspec tim_spec = {.it_interval= {.tv_sec=it_sec,.tv_nsec=it_nsec},
                    .it_value = *T_timer_start};

    // Create the signal and assign it to a provided handler function
    act.sa_flags = SA_SIGINFO | SA_RESTART;
    act.sa_sigaction = handler;
    sigemptyset(&act.sa_mask);

    if (sigaction( SIGALRM, &act, NULL ) == -1)
    {
        perror("timer alert failed");
        return -1;
    }

    // Timer setup
    te.sigev_notify = SIGEV_SIGNAL;
    te.sigev_signo = SIGALRM;
    te.sigev_value.sival_ptr = timerID;
    if (timer_create(CLOCK_MONOTONIC, &te, timerID))
    {
        perror("timer_create");
        return -1;
    }
    if (timer_settime(*timerID, TIMER_ABSTIME, &tim_spec, NULL))
    {
        perror("timer_settime");
        return -1;
    }
    return 0;
}

