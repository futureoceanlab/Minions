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
 *   is meant to be used by the LED RPI0W.
 * 
 *   There can be multiple sessions of imaging (e.g., 1hr long session 
 *   every 6hrs). In each session, LED will start strobing and serve any
 *   incoming synchronization requests from the clients at any point in time.
 *   This allows for the system to function even if any of the camera nodes are
 *   down.
 * 
 *   Once the session timer is up, the clients and the server talk in the
 *   following procedure:
 *      .stepA
 *          - Clients A + B sends their time T1, and server responsds with its 
 *          T2 and T3. 
 *          - Repeat .stepA until all clients sends 0 in data buffer. 
 *          This menas that the clients have accumulated sufficient number 
 *          of data for averaging (100).
 * 
 *      .stepB  
 *          - all clients are done averaging --> server sends time to start
 *          trigger cameras and tells when this particular session ends
 * 
 *      .stepC
 *          - sockets are closed and LED strobing begins
 *   After the session is over, the client might ask for synchronization. In
 *   that case, the server tells the client that the session is over, and it 
 *   should wait by sleeping for X amount of time until the next session.
 * 
 *   If the deployment is over, the LED server responds with "TERMINATE" to 
 *   the clients. Both server and clients who receive this messeage will close
 *   gracefully. Alternatively, if the message does note get transferred, the 
 *   clients will repeatedly sleep for t seconds, which increases exponentially.
 *  
 *      
 */

#include <sstream>
#include <atomic>

#include "synchronization.h"
#include "peripheral.h"
#include "logger.h"
#include "utility.h"
#include "network.h"

#define DEBUG 

#define BILLION 1000000000LL    // nanoseconds conversion
#define PERIOD 1*BILLION 
#define TERMINATED 1 

// Timer status flag big options
#define TIMER_L 0b1
#define TIMER_S 0b10
#define TIMER_E 0b100

using namespace std;


/**
 * Function Declaration
 */
int runLED();
static void timerHandler(int sig, siginfo_t *si, void *uc);
void ledTimerHandler();
void logTimerHandler();
bool checkDepth();
int startMission();
int handleTriggerRequest(char timeData[], long long trig_period, int cli_id);
int handleSessionChange(long long trig_period);
int handleSyncRequest(char timeData[], int cli_id);
void logInternalMsg(string msg);


/**
 * global variables
 */
Peripheral *peripheral = new Peripheral();
// peripheral data logger (csv)
Logger *dataLogger = new Logger();
// internal message logger (txt)
Logger *internalLogger = new Logger();

timer_t logTimerID, ledTimerID, imageStartTimerID, imageStopTimerID;
struct timespec T_start, T_session_start, T_session_end, T_now;
long long count = 0, c_count = 0;
long long T_start_n, T_session_start_n, T_session_end_n;

string rtc_time = "";
ostringstream msg_stream;

// atmoic boolean flags that get modified during timer interrupts
atomic_bool fStopImaging(true); 
atomic_bool fRunning(false);
atomic_bool fSessionChanged(false);

uint8_t timer_status = 0;

// Deployment configuration and status variables
int cur_session = 1;
float cur_depth = 0;
int n_sessions, session_duration, session_period, target_depth,
    wait_duration, framerate;


/**
 * Entrance to the entire code. 
 */
int main(int argc, char* argv[])   
{
    // Parse configuration input arguments
    // Conditions for running / terminating the missions
    // e.g. time up, velocity profile, target_depth, framerate
    
    // Default values for the variables in case not provided
    n_sessions = 1;             // times
    session_duration = 1;       // minutes
    session_period = 3;         // minutes
    target_depth = 5;           // meters
    wait_duration = 1;          // minutes
    framerate = 1;              // FPS

    for (int i = 0; i < argc; i++) 
    {
        if (strcmp(argv[i], "-n") == 0)
        {
            n_sessions = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-s") == 0)
        {
            session_duration = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-p") == 0)
        {
            session_period = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-d") == 0)
        {
            target_depth = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-w") == 0)
        {
            wait_duration = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-f") == 0)
        {
            framerate = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-h") == 0)
        {
            cout << "-n\tnumber of sessions\n\
-s\tduration of each session (min)\n\
-p\tperiod of each sessions (min)\n\
-d\ttarget depth (m) to which the system waits for \"wait duration\"\n\
-w\twait duration (min)\n\
-f\tframerate (fps)" << endl;
            return -1;
        }
    }
    if (session_period < session_duration)
    {
        cout << "each session needs to be shorter than the period" << endl;
        return -1;
    }
    session_duration *= 60;     // seconds
    session_period *= 60;       // seconds
    wait_duration *= 60;        // seconds
    // Run the main program
    runLED();
}


/**
 * runLED: The high-level logic of the program.
 * 
 *  1. Create timer for logging sensor data regularly
 *  2. Setup master socket that listens to multiple clients

 *  .loop A: main loop
 *      a) Start the deployment once reached a configured target depth for 
 *         the first time. 
 *      b) start and stop imaging/strobing based on status of each session 
 *      c) Check for new connections from clients
 *      .loop B: Check communication with each client
 *          Handle if
 *          i) disconnected / error 
 *          ii)serve synchornization inquiries
 */
int runLED()
{
    long long trig_period = (long long) (1/framerate)*BILLION;
    uint8_t cli_status[2] = {!TERMINATED};    

    // Setup logs
    rtc_time = runScript("/home/pi/rtc.sh");
    ostringstream internalLogName, dataLogName;
    internalLogName << rtc_time << "_log.txt";
    dataLogName << rtc_time << "_data.csv";
    dataLogger->open(dataLogName.str());
    dataLogger->write("Timestamp(ns),RTC,Pressure(mbar),Temperature(C)\n".str());
    internalLogger->open(internalLogName.str());

    logInternalMsg("The LED booted up");
    clock_gettime(CLOCK_MONOTONIC, &T_now);
    makeTimer(&logTimerID, &T_now, 1, 0, &timerHandler);

    // Initialize peripherals    
    if (peripheral->init() == -1)
    {
        logInternalMsg("error connecting to the peripherals");
        return -1;
    }

    int master_socket, addrlen, new_socket, activity, valread, sd;   
    int max_clients = 2, opt=1;
    int client_socket[max_clients] = {0};
    struct sockaddr_in address;   
    configureMasterSocket(&master_socket, &opt, &address);
    //set of socket descriptors  
    fd_set readfds;   
    //accept the incoming connection  
    addrlen = sizeof(address);   


    char buffer[16];  //data buffer of 16 bytes max  
    // Handle network until both cameras have responded saying that they are
    // synchronized and ready to trigger cameras.
    // .loop A
    while(1)   
    {   
        // a) Check and start mission
        if (!fRunning && checkDepth())
        {
            startMission();            
            fRunning = true;
        }
        // b) Control imaging/lighting based on the sessions
        if (fSessionChanged)
        {
            handleSessionChange(trig_period);
            fSessionChanged = false;
        }
        // c) Check Connection
        int conn_status = handleConnection(&readfds, &master_socket, &address, addrlen, client_socket, max_clients);
        if (conn_status == 1)
            continue;

        // .loop B: Handle communicaiton
        for (int i = 0; i < max_clients; i++)   
        {   
            sd = client_socket[i];   
            // No request, continue
            if (!(FD_ISSET( sd , &readfds)))
                continue;
            
            // i) Check Comm status  
            if ((valread = read( sd , buffer, 1024)) == 0)   
            {   
                //Somebody disconnected , get his details and print  
                getpeername(sd , (struct sockaddr*)&address, (socklen_t*)&addrlen);   
                msg_stream.str("");
                msg_stream << inet_ntoa(address.sin_addr) << " disconnected"; 
                logInternalMsg(msg_stream.str());
                //Close the socket and mark as 0 in list for reuse  
                close( sd );   
                client_socket[i] = 0;   
                continue;
            }
            else if (valread < 0)
            {
                getpeername(sd , (struct sockaddr*)&address, (socklen_t*)&addrlen);   
                msg_stream.str("");
                msg_stream << inet_ntoa(address.sin_addr) << " socket error"; 
                logInternalMsg(msg_stream.str());
                perror("Socket: ");
                continue;
            }
            // ii) Handle client synchronization inquiries
            else 
            {
                // clients are responding with its T1 value to receive
                // more time information for synchronization
                long long rec_T = bytes_to_nsec(buffer);
                char timeData[16] = {0};
                if (rec_T > 1) 
                {
                    handleSyncRequest(timeData, i);
                }
                // client sends 0 to let server know that sychronization is
                // complete and it is ready to receive trigger start time.
                else if (rec_T == 0)
                {
                    handleTriggerRequest(timeData, trig_period, i);
                }
                // Send the response
                int send_status = send(sd, timeData, 16, 0);   
                if (send_status == -1) 
                {
                    perror("\nError is ");
                }
            }
        }
    }   
    // The deployment has terminated. Wrap up
    timer_delete(logTimerID);
    dataLogger->close();
    logInternalMsg("Mission Complete. Bye Bye");
    internalLogger->close();
    return 0;   
}


/**
 * timerHandler: When there is a timer interrupt, identify which timer,
 * and serve accordingly. In total, there are four possible timers. LED strobe,
 * sensor data log, stop imaging and start imaging
 */
static void timerHandler(int sig, siginfo_t *si, void *uc)
{
    timer_t *tidp;
    tidp = (timer_t *) si->si_value.sival_ptr;
    // LED
    if ( *tidp == ledTimerID )
    {
        ledTimerHandler();
    }
    // Log
    else if ( *tidp == logTimerID)
    {
        logTimerHandler();
    }
    else if ( *tidp == imageStopTimerID )
    {
        // Indicate that the session is over
        fStopImaging = true;
        fSessionChanged = true;
        cur_session ++;
    }
    else if (*tidp == imageStartTimerID)
    {
        // Indicate that the session is starting
        fStopImaging = false;
        fSessionChanged = true;
    }
}


/**
 * logTimerHandler: write rtc and peripheral data to a log csv file
 */
void logTimerHandler()
{
    // Update with new RTC value to track internal clock drift
    if (c_count % 600 == 0)
    {
        rtc_time = run_script("/home/pi/rtc.sh");
        c_count = 0;
    } 
    else 
    {
        rtc_time = "0";
    }
    peripheral->readData();
    cur_depth = peripheral->getDepth();
    dataLogger->logData(rtc_time, peripheral->getPressure(), peripheral->getTemperature());
    c_count++;
}


/**
 * ledTimerHandler: produce 10ms long periodic strobing
 */
void ledTimerHandler()
{
    peripheral->ledOn();
    usleep(10000);

    peripheral->ledOff();
    count ++;
}

/**
 * checkDepth: check if the target depth has been passed
 */
bool checkDepth()
{
    #ifdef DEBUG
    return true;
    #else
    return cur_depth > target_depth;
    #endif 
}


/**
 * handleSyncRequest:
 *      char timeData[]: char array to be modified with appropriate response
 *                       it is 16-bit long
 *      int cli_id: internal id of the client
 * 
 * Respond appropriately to a synchronization request depending on the status
 * of the deployemt (session running/stopped or terminated)
 * 
 */
int handleSyncRequest(char timeData[], int cli_id)
{
    // T2 for synchronization
    struct timespec T2;
    // handle synchronization 
    clock_gettime(CLOCK_MONOTONIC, &T2);

    // if the deployment has not terminated
    if (cur_session <= n_sessions)
    {
        // if the target depth has been reached
        if (fRunning)
        {
            // if session is running
            if (!fStopImaging)
            {
                // T3 for synchronization
                struct timespec T3;
                clock_gettime(CLOCK_MONOTONIC, &T3);
                char *T2_arr = (char *) &(T2.tv_sec);
                char *T3_arr = (char *) &(T3.tv_sec);
                // Respond with T2 and T3 
                for (int i = 0; i < 8; i++) 
                {
                    timeData[i] = T2_arr[i];
                    timeData[8+i] = T3_arr[i];
                }
            }
            else
            {
                // Find the waiting duration until the next session
                struct timespec T_wait; 
                long long T_wait_n = (T_session_start_n + (cur_session-1)*session_period*BILLION) - as_nsec(&T2);

                msg_stream.str("");
                msg_stream << "Client " << cli_id  << " sleep for " << T_wait_n/BILLION << " seconds";
                logInternalMsg(msg_stream.str());
                
                // Respond wait duration 
                as_timespec(T_wait_n, &T_wait);
                char *T_wait_arr = (char *) &(T_wait.tv_sec);
                for (int j = 0; j < 8; j++) 
                {
                    timeData[8+j] = T_wait_arr[j];
                }
            }
        }
    }                   
    // Terminated     
    else 
    {
        msg_stream.str("");
        msg_stream << "Client " << cli_id  << " terminate";
        logInternalMsg(msg_stream.str());
        // Respond with (1, 0), which means to terminate
        timeData[4] = 1;
        //cli_status[i] = TERMINATED;
        // If both cameras are informed, we shut down the LED unit as well.
        /*if (cli_status[0] == TERMINATED && cli_status[1] == TERMINATED)
            break;*/
    }
}


/**
 * handleTriggerRequest:
 *      timeData[]: char array with modified response
 *      trig_period: LED trigger period
 *      cli_id: internal id of the client
 * 
 * The client asks for the last trigger for synchronization. 
 */
int handleTriggerRequest(char timeData[], long long trig_period, int cli_id)
{
    // Provide the last trigger*/
    struct timespec T_last_trig;
    // Add however many triggers have been made so far
    long long last_trigger = T_start_n + (count * trig_period);
    as_timespec(last_trigger, &T_last_trig);

    // Tell them when this session is expected to end
    long long T_end_n = T_session_end_n + (cur_session-1)*session_period*BILLION;
    struct timespec T_cur_end;
    as_timespec(T_end_n, &T_cur_end);

    // Formulate the response 
    char *data1 = (char *) &(T_last_trig);
    char *data2 = (char *) &(T_cur_end);
    for (int i = 0; i < 8; i++) 
    {
        timeData[i] = data1[i];
        timeData[8+i] = data2[i];
    }

    // Log
    msg_stream.str("");
    msg_stream << "Client " << cli_id  << ": last trigger (" << last_trigger << ", " << T_end_n << ")";
    logInternalMsg(msg_stream.str());
}

/**
 * handleSessionChange: 
 *      trig_period: LED trigger period
 * 
 * Change the timers that corresponds to the recent session change
 */
int handleSessionChange(long long trig_period)
{
    // Session Stop
    if (fStopImaging)
    {
        logInternalMsg("Session stopped");
        // Disarm the LED timer
        if (timer_status & TIMER_L)                
        {
            logInternalMsg("Disarm LED alarm");
            timer_delete(ledTimerID);
            timer_status &= (~TIMER_L);
        }
        // Termination
        if (cur_session > n_sessions)
        {
            logInternalMsg("Mission Over - Terminate");
            // Handle Termination
            // all of the sessions have been made. Stop taking images
            if (timer_status & TIMER_S)
            {
                logInternalMsg("Disarm Image start alarm");
                timer_delete(imageStartTimerID);
            }
            if (timer_status & TIMER_E)
            {
                logInternalMsg("Disarm IMage end alarm");
                timer_delete(imageStopTimerID);
            }
            timer_status &= ~(TIMER_S | TIMER_E);
        }
    }
    // Session start
    else
    {
        count = 0;
        clock_gettime(CLOCK_MONOTONIC, &T_now);

        // Setup start time of this sesssion
        T_start_n = as_nsec(&T_now) + 2*BILLION;
        as_timespec(T_start_n, &T_start);
        struct timespec T_Period = {.tv_sec=0, .tv_nsec=0};
        as_timespec(trig_period, &T_Period);

        // Make a timer for LED strobing
        makeTimer(&ledTimerID, &T_start, T_Period.tv_sec, T_Period.tv_nsec, &timerHandler);
        timer_status |= TIMER_L;

        // Log
        msg_stream.str("");
        msg_stream.clear();
        msg_stream << "Start session " << cur_session;
        logInternalMsg(msg_stream.str());
    }
}


/**
 * startMission: Once the target depth has been reached, set up the 
 * session reltaed timers
 */
int startMission()
{
    // Timestamp current time
    clock_gettime(CLOCK_MONOTONIC, &T_now);
    // Session start time 
    long long T_now_n =  as_nsec(&T_now);
    T_session_start_n = T_now_n + (long long) wait_duration*BILLION;
    T_session_end_n = T_session_start_n + (long long) session_duration*BILLION;
    as_timespec(T_session_start_n, &T_session_start);
    as_timespec(T_session_end_n, &T_session_end); 

    // Create timer to start the sessions
    makeTimer(&imageStartTimerID, &T_session_start, session_period, 0, &timerHandler);
    // Create timer to stop the sessions
    makeTimer(&imageStopTimerID, &T_session_end, session_period, 0, &timerHandler );
    timer_status |= (TIMER_S | TIMER_E);
    logInternalMsg("Imaging start and end timer good to go.");
    return 0;
}

/**
 *  log_intenral_msg: logs a given message on a txt file
 */
void logInternalMsg(string msg)
{
    #ifdef DEBUG
    std::cout << msg << std::endl;
    #endif
    internalLogger->logMsg(msg);
}

