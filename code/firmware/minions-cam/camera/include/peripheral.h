/**
 * peripheral: a class to handle communication with the peripheral
 * electronics (e.g., RTC, pressure sensor, GPIO, etc...)
 * 
 * author: Junsu Jang (junsuj@mit.edu)
 * date: Aug 25th, 2020
 * 
 */

#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#include "KellerLD.h"

#define I2C_BUS 1 
#define LED_EN_PIN 22
#define LED_FAULT_PIN 21
#define LED_PIN 23 
#define TRIG_PIN 5

class Peripheral
{
public:
    Peripheral();
    Peripheral(int i2c_bus);
    
    int init();
    void triggerOn();
    void triggerOff();
    void ledOn();
    void ledOff();
    void setup();
    float getPressure();
    float getTemperature();
    float getDepth();
    void readData();

private:
    int i2c_bus = I2C_BUS;
    KellerLD *k_sensor;

    void setupPi();
    void pSensorInit();
};



#endif
