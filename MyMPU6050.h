#ifndef _MY_MPU6050_h_
#define _MY_MPU6050_h_
#include <Arduino.h>
 #include <Wire.h> // Official Arduino Wire library

static const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
static const uint16_t I2C_TIMEOUT = 100; // Used to check for errors in I2C communication


class MyMPU6050 {
    public:
        MyMPU6050();
        int initialize();
        void readMotionSensor(int &ax, int &ay, int &az, int &gx,
                                     int &gy, int &gz);
    private:
      uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);            
      uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
      uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

      uint8_t i2cBuffer[14]; // Buffer for I2C data      

 };

#endif

