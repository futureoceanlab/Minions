/**
 *  led_main.c
 *  May 19th, 2020
 *  Authors: Junsu Jang, FOL/MIT
 *  
 *  Description:
 *   One of the three housings for Minions is intended for driving the LEDs 
 *   and synchronizing the cameras. It is going to be run an a RPI Zero W,
 *   whose WiFi is going to be used to synchronize the time of the cameras
 *   based on RDS. 
 * 
 *   This firmware has two jobs:
 *     1. Synchronize the time on the cameras by broadcasting its time
 *        to the cameras with a specified period
 * 
 *     2. Drive the LED at given frequency to strobe light in the water
 * 
 *     Job 1 is achieved in the following order:
 *       i) Waking up the WiFi every X minutes based on timer interrupt
 *       ii) Setup an AP
 *       iii) Wait until the cameras have connected to this AP
 *       iv) Broadcast X times until Y responses are made
 *       v) Turn off WiFi and set the timer to wake up to restart from i)
 * 
 *     Job 2 is achieved with a GPIO to trigger the MOSFET. It also monitors
 *     any fault on the LEDs and controls the enable on the load switch. 
 *     (3 GPIOs)
 */

