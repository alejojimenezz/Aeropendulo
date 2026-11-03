#ifndef SYSTEMCONFIG_H
#define SYSTEMCONFIG_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "driver/mcpwm.h" // Agregado para MCPWM

class SystemConfig {
private:
    byte* _pinSPI;
    byte* _pinI2C;
    byte _pinSTBY; 
    byte _i2cAddress; 
    
    bool _SDStatus;
    bool _I2CStatus;
    bool _SerialStatus;
    
    unsigned long _bauds;
    unsigned int timeout;

public:
    byte* motorPins; // [0] = PWMA, [1] = AIN1, [2] = AIN2

    Adafruit_MPU6050 mpu;
    Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

    SystemConfig(byte* spiPins, byte* i2cPins, byte* mPins, byte stbyPin, byte i2cAddress);
    bool start();
};

#endif