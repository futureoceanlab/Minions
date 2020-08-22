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
#include <signal.h>
#include <unistd.h>
#include <atomic>
#include <sstream>


#include "synchronization.h"
#include "peripheral.h"
#include "logger.h"
#include "utility.h"


#define DEBUG

#define PERIOD 1 
#define MIN 60
#define TEN_MIN 600
#define OFFSET 2 

#define ARB_WAIT 5                                  // Seconds
#define SES_NEW 0
#define SES_RUNNING 1
#define SES_STOPPED 2

#define TIMER_S 0b1
#define TIMER_T 0b10
#define TIMER_D 0b100
#define TIMER_E 0b1000

Peripheral *peripheral = new Peripheral(1);
Logger *internalLogger = new Logger();

void logInternalMsg(std::string msg);


int count = 0;
uint8_t wait_count = 0;
std::atomic_bool fSync(false);
std::atomic_bool fDrift(false);
std::string t_rtc;
timer_t cameraTimerID, syncTimerID, driftTimerID, stopTimerID;
struct timespec now;
long long T_skew_prev, T_skew_now;
// PID of the imaging firmware for signal
pid_t imaging_pid = 0;
uint8_t session_status = SES_STOPPED;
uint8_t timer_status = 0;

void triggerCamera();

static void timer_handler(int sig, siginfo_t *si, void *uc)
{
    timer_t *tidp;
    tidp = (timer_t *) si->si_value.sival_ptr;
    if ( *tidp == cameraTimerID )
    {
        triggerCamera();
    }
    else if ( *tidp == syncTimerID)
    {
        fSync = true;
    }
    else if ( *tidp == driftTimerID )
    {
        fDrift = true;
    }
    else if (*tidp == stopTimerID)
    {
        fSync = true;
        if (session_status == SES_RUNNING)
        {
            session_status = SES_STOPPED;
            // Turn off Simple Snapimage
            if (timer_status & TIMER_T)
                timer_delete(cameraTimerID);
            if (timer_status & TIMER_D)
                timer_delete(driftTimerID); 
            if (timer_status & TIMER_E)
                timer_delete(stopTimerID);
            if (timer_status & TIMER_S)
                timer_delete(syncTimerID);
            timer_status = 0;
            kill(imaging_pid, SIGSTOP);
        }
    }
}

void triggerCamera()
{
    // trigger camera
    peripheral->triggerOn();
    usleep(10);
	peripheral->triggerOff();
	count++;
}


void setup()
{
    // CSV setup
    std::string logName="changeme.csv";
    internalLogger->open(logName);

    if (peripheral->init() == -1)
    {
        logInternalMsg("Error connecting to peripherals");
        return;
    }
}


int main(int argc, char* argv[])
{
    // TODO: (1) Get configuration parmaeter from argv
    // TODO: (2) Get RTC time +i configuration from the led?
    // TODO: Log the drift + skew time;
    for (int i = 0; i < argc; i++)
    {
        if (strcmp(argv[i], "-i") == 0)
        {
            imaging_pid = (pid_t) atoi(argv[i+1]);
        }
    }
    int drift_period = 15, sync_period = 300;
    runCamera(drift_period, sync_period);
}

void runCamera(int drift_period, int sync_period)
{
    long long T_trig_n, T_sync_n, T_drift_n;
    int trig_period = PERIOD; 
    int status;
    struct timespec T_trig, T_sync, T_drift, T_stop;
    long long server_sec = BILLION;
    setup();
    struct timeinfo TI = {.T_skew_n = 0, .T_start_n = 0, .T_stop_n=0};
    uint8_t hasSyncTimer = 0;
    fSync = true;

    // Routine for timer handling
    while (1)
    {
        // set time for 10 min synchronization
        if (fSync)
        {
            logInternalMsg("Synchronize");
            int sync_status = synchronize(&TI, 0);
            
            if (sync_status == -1) 
            {
                logInternalMsg("Synchronize Error");
                sleep(4 << wait_count);
                wait_count ++;
                if (wait_count < 10)
                    continue;
                else
                    break;
            }            
            else if (sync_status == TERMINATE || sync_status == STOP_IMAGING)
            {
                if (session_status == SES_RUNNING)
                {
                    session_status = SES_STOPPED;
                    if (timer_status & TIMER_T)
                        timer_delete(cameraTimerID);
                    if (timer_status & TIMER_D)
                        timer_delete(driftTimerID); 
                    if (timer_status & TIMER_E)
                        timer_delete(stopTimerID);
                    if (timer_status & TIMER_S)
                        timer_delete(syncTimerID);
                    timer_status = 0;
                    count = 0;
                    // Tell the imaging firmware to stop
                    kill(imaging_pid, SIGSTOP);

                }
                if (sync_status == TERMINATE)
                {
                    // Tell the imaging firmware to terminate
                    kill(imaging_pid, SIGTERM);
                    logInternalMsg("Sync: terminate");
                    break;
                }
                else
                {
                    // If we know the snap_simpeimage is running, we should kill it!
                    // If we still have sessions left, this will naturally go back to 
                    // sync polling upon waking up from the sleep(wait_duration) 
                    if (TI.T_skew_n == 0)
                    {
                        logInternalMsg("Sync: server not ready, wait exponentially");
                        sleep(4 << wait_count);
                        wait_count ++;
                        if (wait_count < 10)
                            continue;
                        else
                            break;
                    }
                    else
                    {
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
            logInternalMsg("Sync: Successful communication");
            wait_count = 0;
            as_timespec(TI.T_start_n, &T_trig);
            T_trig_n = TI.T_start_n;
            T_skew_now = TI.T_skew_n;
            T_drift_n = T_trig_n + drift_period*server_sec + server_sec/OFFSET;
            as_timespec(T_drift_n, &T_drift);
            //
            // We are in a proper running status 
            //
            // We are properly synchronized, so we should
            // (1) Set a camera trigger to the right time
            // (2) Set a next drift time
            // if the beginning of a session, we need to run snap_simpleimage 
            if (session_status == SES_STOPPED)
            {
                session_status = SES_RUNNING;
                T_sync_n = T_trig_n;
                as_timespec(TI.T_stop_n, &T_stop);
                // Run simple_snapimage
                kill(imaging_pid, SIGCONT);
                // Trigger
                status = makeTimer(&cameraTimerID, &T_trig, PERIOD, 0, &timer_handler);
                // Drift
                status = makeTimer(&driftTimerID, &T_drift, 0, 0, &timer_handler); //MIN, server_sec/4);
                status = makeTimer(&stopTimerID, &T_stop, 0, 0, &timer_handler);
                timer_status |= (TIMER_T | TIMER_D | TIMER_E);
                count = 0;
            }
            else
            {
                resetTimer(&cameraTimerID, &T_trig, server_sec*PERIOD);
                resetTimer(&driftTimerID, &T_drift, 0);
            }
            fSync = 0;
        }

        // set time for 1 min drift computation
        if (fDrift)
        {
            logInternalMsg("Compute Drift");
            T_skew_prev = T_skew_now;
			//clock_gettime(CLOCK_REALTIME, &now);
            int skew_status = get_skew(&TI);
            if (skew_status == -1) 
            {
                fDrift=0;
                fSync = 1;
                logInternalMsg("Drift: server communication error");
                continue;
            } 
            else if (skew_status == STOP_IMAGING || skew_status == TERMINATE)
            {
                logInternalMsg("Drift: stop imaging or terminate from server");
                fSync = 1;
                fDrift = 0;
                wait_count = 0;
                continue;
            }
            logInternalMsg("Drift: Successful communication");
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
            resetTimer(&cameraTimerID, &T_trig, server_sec*PERIOD);

            // reset synchronization time with new server second
            T_sync_n = T_sync_n + sync_period * server_sec + server_sec/OFFSET;
            as_timespec(T_sync_n, &T_sync);
            if (!hasSyncTimer)
            {
                makeTimer(&syncTimerID, &T_sync, 0, 0, &timer_handler); //TEN_MIN, server_sec/4);
                timer_status |= TIMER_S;
            }
            else
            {
                resetTimer(&syncTimerID, &T_sync, 0);
            }
            fDrift = 0;
        }
        if (!(fSync || fDrift))
        {
            // sleep for 1ms
            usleep(10);
        }
    }
    internalLogger->close();
    // Done Data Acquisition
    // Programmed data acquisition duration elapsed
    //  - Regularly measure depth and temperature until powered off.
    return;
}


void logInternalMsg(std::string msg)
{
    #ifdef DEBUG
    std::cout << msg << std::endl;
    #endif
    internalLogger->logMsg(msg);
}


