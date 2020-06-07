/* ----------------------------------------------------------------------------
 *   Minions-cam: program to run stereo pair on Minions floats
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


#include <stdio.h>
#include <wiringPi.h>
#include <time.h>
#include <chrono>
#include <iostream>

#include "KellerLD.h"


#define I2C_BUS 1 
#define LED_EN_PIN 17
#define LED_FAULT_PIN 18
#define BUZZ_PIN 22
#define LED_PIN 23
#define TRIG_PIN 24


KellerLD *k_sensor = new KellerLD(I2C_BUS);
/* --
 * Mission details require following information
 *   - Deployment start depth (bar)
 *   - Framerate (fps)
 *   - Time synchronization interval (sec)
 *   - sensor measurement rate (regular) (sec (period))
 *   - Post deployment sensor measurement rate (sec (period))
 */

void timer_handler()
{
    // trigger camera
    digitalWrite(TRIG_PIN, HIGH);
    digitalWrite(TRIG_PIN, LOW);
    
    // Increment frame ID

    // Measure the depth and temperature

    // Save timestamp, depth and temperature, frame id
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    // save RTC timestamp every hour for 250ppm drift (1s per 4000s or 66.6min) 
    // in RPI clock and 1 second resolution of DS3231 RTC (500ms error)
}

void setup()
{
    wiringPiSetup();

    pinMode(LED_FAULT_PIN, INPUT);

    pinMode(BUZZ_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(LED_EN_PIN, OUTPUT);
}

int main()
{
    // Find out if this is a master or a slave


    // Prior Deployment (on deck)
    //  - Regularly measure depth and temperature until below specified depth
    k_sensor->init();
    if (k_sensor->isInitialized())
    {
        printf("Sensor isInitialized\n");
    }
    else
    {
        printf("Sensor NOT connected\n");
    }


    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    k_sensor->readData();
    k_sensor->pressure();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
//    printf("Pressure: %.2f\nAltitude: %.2f\n", k_sensor->pressure(), k_sensor->altitude());

    // During Deployment
    //  - Start taking images for programmed duration

    // Done Data Acquisition
    // Programmed data acquisition duration elapsed
    //  - Regularly measure depth and temperature until powered off.


    return 0;
}
