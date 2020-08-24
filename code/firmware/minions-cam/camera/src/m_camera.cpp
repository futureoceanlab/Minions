/* --------------------------------------------------------------------------
 *   Minions-cam: run stereo pair on Minions floats
 *   May 12, 2020
 *   Authors: Junsu Jang, FOL/MIT
 *      Description: 
 *   
 *   Minions-cam is intended for Linux based embedded SBC to 
 *   control the stereo camera on Minions floats. 
 *   This firmware has three jobs: 
 *   
 *   1. Trigger images and strobe LEDs accordingly at specified 
 *      framerate
 * 
 *   2. Log relevant sensor data
 * 
 *   3. Save images accordingly once the camera has reached 
 *      below 20m 
 * 
 *   4. Synchronize time with the slave camera
 * 
 *   Jobs 1 and 2 are done by a timer interrupt, which toggles
 *   a flag. Inside the main loop, appropriate GPIO pins are
 *   toggled and sensor data is logged on a CSV file.
 * 
 *   Jobs 2 happens when the images arrive through the USB, images
 *   are saved along with the timestamp.
 * 
 *   Job 4 is processed in the main loop and requires another
 *   timer interrupt with 6 hour long interval. B connects
 *   to the WiFi hosted by A, and A runs a script that ssh
 *   into B and sets the clock on bash
 *   
*/


/* --
 * Mission details require following information
 *   - Deployment start depth (bar)
 *   - Framerate (fps)
 *   - Time synchronization interval (sec)
 *   - sensor measurement rate (regular) (sec (period))
 *   - Post deployment sensor measurement rate (sec (period))
 */

#include <iostream>
#include <stdexcept>
#include <string>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <signal.h>
#include <unistd.h>
#include <atomic>
#include <sstream>

#include "synchronization.h"
#include "peripheral.h"
#include "logger.h"
#include "utility.h"

// Uncomment to operate in DEBUG mode
#define DEBUG

#define PERIOD 1 
#define MIN 60
#define TEN_MIN 600
#define OFFSET 2 
// Number of times to wait until maximum waiting duration
#define MAX_WAIT 10

// Global status for session
#define SES_NEW 0
#define SES_RUNNING 1
#define SES_STOPPED 2

// Flags for each timer status
// When a bit is 1, its corresponding timer is currently running
#define TIMER_T 0b1        // Camera Trigger
#define TIMER_D 0b10       // Drift
#define TIMER_S 0b100      // Stop

/* Function declaration */
static void timerHandler(int sig, siginfo_t *si, void *uc);
void triggerCamera();
void setup();
void runCamera(int drift_period);
void logInternalMsg(std::string msg);

/* Global Variables */

// PID of the imaging firmware for signal
pid_t imaging_pid = 0;

// Class instances for operation
Peripheral *peripheral = new Peripheral(1);
Logger *internal_logger = new Logger();
Logger *time_logger = new Logger();
// Number of times that the unit has fallen a sleep (exponentially) to
// wait for the next instruction from the server
uint8_t wait_count = 0;

// atomic boolean flag for status of the unit
std::atomic_bool fSync(false);
std::atomic_bool fDrift(false);

// RTC time in string (UTC format)
std::string t_rtc;

// Various timers used to timer interrupts
timer_t triggerTimerID, driftTimerID, stopTimerID;

// variable to timstamp current time
struct timespec T_now;
long long T_now_n;
// time (nanoseconds) for monitoring the previous and current amount of
// skew compared to the server
long long T_skew_prev, T_skew_now;

// status variables
uint8_t session_status = SES_STOPPED;
// bit-wise flag variable for timer status
uint8_t timer_status = 0;
// Directory to which the images will be saved
char *data_dir;

/**
 * main: entrance to the firmware. Parse input arguments.
 */
int main(int argc, char* argv[])
{
    int drift_period = 15;
    uint8_t has_pid = 0, has_data_dir = 0;
    for (int i = 0; i < argc; i++)
    {
        if (strcmp(argv[i], "-i") == 0)
        {
            imaging_pid = (pid_t) atoi(argv[i+1]);
            has_pid = 1;
        }
        else if (strcmp(argv[i], "-w") == 0)
        {
            drift_period = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-d") == 0)
        {
            data_dir = argv[i+1];
            has_data_dir = 1;
        }
        else if (strcmp(argv[i], "-h") == 0)
        {
            std::cout << "\t-i\tPID of the imaging firmware\n\
        -w\tTime after synchronization until drift computation (s)\n\
        -d\tPath to directory to store log files\n\
        " << std::endl;
            return 0;
        }
    }
    if (!(has_pid && has_data_dir))
    {
        std::cout << "Please provide the imaging firmware PID\
    and the data directory" << std::endl;
        return -1;
    }
    // Check the existance of the directory
    struct stat sb;
    if (!(stat(data_dir, &sb) == 0 && S_ISDIR(sb.st_mode)))
    {
        // the directory does not exist, so we create it here
        mkdir(data_dir, 0775);
    }
    runCamera(drift_period);
}

void runCamera(int drift_period)
{
    long long T_trig_n, T_drift_n;
    int trig_period = PERIOD; 
    int status;
    struct timespec T_trig, T_drift, T_stop;
    long long server_sec = BILLION;
    setup();
    struct timeinfo TI = {.T_skew_n = 0, .T_start_n = 0, .T_stop_n=0};
    fSync = true;

    // Routine for timer handling
    while (1)
    {
        // Time to synchronize with the server
        if (fSync)
        {
            logInternalMsg("Sync: Attempt to communicate with the server");
            int sync_status = synchronize(&TI, 0);
            
            if (sync_status == -1) 
            {
                logInternalMsg("Synchronize Error");
                sleep(4 << wait_count);
                wait_count ++;
                if (wait_count < MAX_WAIT)
                    continue;
                else
                    break;
            }            
            else if (sync_status == TERMINATE || sync_status == STOP_IMAGING)
            {
                // A session is over, delete any running timers
                if (session_status == SES_RUNNING)
                {
                    session_status = SES_STOPPED;
                    if (timer_status & TIMER_T)
                        timer_delete(triggerTimerID);
                    if (timer_status & TIMER_D)
                        timer_delete(driftTimerID); 
                    if (timer_status & TIMER_S)
                        timer_delete(stopTimerID);
                    timer_status = 0;
                    // Tell the imaging firmware to stop
                    kill(imaging_pid, SIGSTOP);
                }
                // We are terminating, break out of the while loop and also
                // inform the imaging firmware to terminate
                if (sync_status == TERMINATE)
                {
                    // Tell the imaging firmware to terminate
                    kill(imaging_pid, SIGTERM);
                    logInternalMsg("Sync: terminate");
                    break;
                }
                else
                {
                    // In the case of terminate/stop, time to wait is specified in
                    // TI.T_skew_n variable. If it is 0, then the communication with
                    // the server went wrong. 
                    // If we still have sessions left, this will naturally go back to 
                    // sync polling upon waking up from the sleep(wait_duration) 
                    if (TI.T_skew_n == 0)
                    {
                        logInternalMsg("Sync: server not ready, wait exponentially");
                        sleep(4 << wait_count);
                        wait_count ++;
                        if (wait_count < MAX_WAIT)
                            continue;
                        else
                            break;
                    }
                    else
                    {
                    // We have been informed to wait a specific amount of time
                        wait_count = 0;
                        int wait_duration = (int) (TI.T_skew_n / BILLION)+1;
                        std::ostringstream msg_stream;
                        msg_stream << "Sync: server wait " << wait_duration;
                        logInternalMsg(msg_stream.str());
                        sleep(wait_duration);
                    }
                    continue;
                }
            }
            //
            // We are in a proper running status 
            //
            // We are properly synchronized, so we should
            // (1) Set a camera trigger to the right time
            // (2) Set a next drift time
            // if the beginning of a session, we need to run snap_simpleimage 
            logInternalMsg("Sync: Successful communication");
            wait_count = 0;
            as_timespec(TI.T_start_n, &T_trig);
            T_trig_n = TI.T_start_n;
            T_skew_now = TI.T_skew_n;
            as_timespec(TI.T_stop_n, &T_stop);

            T_drift_n = T_trig_n + drift_period*server_sec + server_sec/OFFSET;
            as_timespec(T_drift_n, &T_drift);


            if (session_status == SES_STOPPED)
            {
            // Start a new session
                session_status = SES_RUNNING;
                // Run simple_snapimage
                kill(imaging_pid, SIGCONT);
                // Trigger
                status = makeTimer(&triggerTimerID, &T_trig, PERIOD, 0, &timerHandler);
                // Drift
                status = makeTimer(&driftTimerID, &T_drift, 0, 0, &timerHandler); //MIN, server_sec/4);
                // Timer until the end of this session
                status = makeTimer(&stopTimerID, &T_stop, 0, 0, &timerHandler);
                timer_status |= (TIMER_T | TIMER_D | TIMER_S);
            }
            else
            {
                resetTimer(&triggerTimerID, &T_trig, server_sec*PERIOD);
                resetTimer(&driftTimerID, &T_drift, 0);
            }

            // TODO: Save T_skew_now, T_start_n and T_stop
            std::ostringstream t_data;
            clock_gettime(CLOCK_REALTIME, &T_now);
            T_now_n = as_nsec(&T_now);
            t_data << T_now_n << "," << T_skew_now << "," << T_trig_n << "," << TI.T_stop_n << "\n";
            time_logger->write(t_data.str());

            fSync = 0;
        }

        // Time to compute the amount of drift between the server and the client
        if (fDrift)
        {
            logInternalMsg("Drift: Attempt communication with the server.");
            T_skew_prev = T_skew_now;
            int skew_status = get_skew(&TI);
            if (skew_status == -1) 
            {
                // In the case of communication error, we move to sync as default
                fDrift=0;
                fSync = 1;
                logInternalMsg("Drift: server communication error");
                continue;
            } 
            else if (skew_status == STOP_IMAGING || skew_status == TERMINATE)
            {
                // In stop/terminate, we also adjust the status in sync above
                logInternalMsg("Drift: stop imaging or terminate from server.");
                fSync = 1;
                fDrift = 0;
                wait_count = 0;
                continue;
            }
            logInternalMsg("Drift: Successful communication.");
			T_skew_now = TI.T_skew_n;

            // T_diff is skew difference over a minute that could be locally
            // compensated assuming no change in Temperature over 10 minutes
            // Instead of setting trigger to 1second, we adjust to what is
            // "1 second in server".
            // We also adjust the T_start to the next second that the trigger
            // will start so that the timer is triggered properly.
            double server_period = double((T_skew_now - T_skew_prev)/drift_period + BILLION); 
            server_sec = (long long) (double(BILLION) * (double(BILLION)/server_period));
            T_trig_n += (drift_period+1) * server_sec;
            as_timespec(T_trig_n, &T_trig);
            resetTimer(&triggerTimerID, &T_trig, server_sec*PERIOD);
            // TODO: Save T_skew_now and server_sec
            std::ostringstream t_data;
            clock_gettime(CLOCK_REALTIME, &T_now);
            T_now_n = as_nsec(&T_now);
            t_data << T_now_n << "," << T_skew_now << ",,," << server_sec << "\n";
            time_logger->write(t_data.str());
            fDrift = 0;
        }
        if (!(fSync || fDrift))
        {
            // sleep for 1ms
            usleep(1000);
        }
    }
    // Done with the deployment, terminate and close off everything.
    internal_logger->close();
    time_logger->close();
    return;
}

/**
 * timerHandler: timer interrupt handler. 
 */
static void timerHandler(int sig, siginfo_t *si, void *uc)
{
    timer_t *tidp;
    tidp = (timer_t *) si->si_value.sival_ptr;
    if ( *tidp == triggerTimerID )
    {
        triggerCamera();
    }
    else if ( *tidp == driftTimerID )
    {
        // Alter the drift flag
        fDrift = true;
    }
    else if (*tidp == stopTimerID)
    {
        // Terminate running timers
        fSync = true;
        if (session_status == SES_RUNNING)
        {
            session_status = SES_STOPPED;
            // Turn off Simple Snapimage
            if (timer_status & TIMER_T)
                timer_delete(triggerTimerID);
            if (timer_status & TIMER_D)
                timer_delete(driftTimerID); 
            if (timer_status & TIMER_S)
                timer_delete(stopTimerID);
            timer_status = 0;
            kill(imaging_pid, SIGSTOP);
        }
    }
}

/**
 * logInternalMsg
 */
void logInternalMsg(std::string msg)
{
    #ifdef DEBUG
    std::cout << msg << std::endl;
    #endif
    internal_logger->logMsg(msg);
}



void triggerCamera()
{
    // trigger camera
    peripheral->triggerOn();
    usleep(10);
	peripheral->triggerOff();
}


void setup()
{
    // Timestamp now
    clock_gettime(CLOCK_REALTIME, &T_now);
    long long t_now_sec = T_now.tv_sec;

    // Create log file names
    std::ostringstream internal_log_name, time_log_name;
    internal_log_name << data_dir << "/" << t_now_sec << "_log.txt";
    time_log_name << data_dir << "/" << t_now_sec << "_time.csv";

    // Open respective log file
    time_logger->open(time_log_name.str());
    time_logger->write(std::string("Timestamp (s),T_skew,T_start,T_stop,server sec\n"));
    internal_logger->open(internal_log_name.str());
    if (peripheral->init() == -1)
    {
        logInternalMsg("Error connecting to peripherals");
        return;
    }
    
}
