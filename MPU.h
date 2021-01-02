#pragma once
#ifndef _MPU_H
#define _MPU_H

#ifdef TEENSYDUINO
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

enum class AFS { A2G, A4G, A8G, A16G };
enum class GFS { G250DPS, G500DPS, G1000DPS, G2000DPS };
enum class MFS { M14BITS, M16BITS }; // 0.6mG, 0.15mG per LSB

class MPU
{

public:

    MPU():AFSSEL(AFS::A2G), GFSSEL(GFS::G250DPS), MFSSEL(MFS::M14BITS)
    {
        memset(rawdata, 0, 18);
    }

    MPU(AFS _AFSSEL, GFS _GFSSEL)
    {
        AFSSEL = _AFSSEL;
        GFSSEL = _GFSSEL;
        MFSSEL = MFS::M16BITS;
    }

    MPU(AFS _AFSSEL, GFS _GFSSEL, MFS _MFSSEL)
    {
        AFSSEL = _AFSSEL;
        GFSSEL = _GFSSEL;
        MFSSEL = _MFSSEL;

    }

    const float aRes {getAres()};      // scale resolutions per LSB for the sensors
    const float gRes {getGres()};      // scale resolutions per LSB for the sensors
    const float mRes {getMres()};      // scale resolutions per LSB for the sensors
   
    float getAcc(uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
    float getGyro(uint8_t i) const { return (i < 3) ? g[i] : 0.f; }
    float getMag(uint8_t i) const { return (i < 3) ? m[i] : 0.f; }

    float getAccBias(uint8_t i) const { return (i < 3) ? accelBias[i] : 0.f; }
    float getGyroBias(uint8_t i) const { return (i < 3) ? gyroBias[i] : 0.f; }
    float getMagBias(uint8_t i) const { return (i < 3) ? magBias[i] : 0.f; }
    float getMagScale(uint8_t i) const { return (i < 3) ? magScale[i] : 0.f; }

    void setAccBias(uint8_t i, float v) { if (i < 3) accelBias[i] = v; }
    void setGyroBias(uint8_t i, float v) { if (i < 3) gyroBias[i] = v; }
    void setMagBias(uint8_t i, float v) { if (i < 3) magBias[i] = v; }
    void setMagScale(uint8_t i, float v) { if (i < 3) magScale[i] = v; }

    void setMagneticDeclination(const float d) { magnetic_declination = d; }

    int8_t *getRawData()
    {
        return rawdata;
    }


    void print() const
    {
        printRawData();
        printCalibration();
    }

    void printRawData() const
    {
        // Print acceleration values in milligs!
        Serial.print("ax = "); Serial.print((int)1000 * a[0]);
        Serial.print(" ay = "); Serial.print((int)1000 * a[1]);
        Serial.print(" az = "); Serial.print((int)1000 * a[2]); Serial.println(" mg");
        // Print gyro values in degree/sec
        Serial.print("gx = "); Serial.print(g[0], 2);
        Serial.print(" gy = "); Serial.print(g[1], 2);
        Serial.print(" gz = "); Serial.print(g[2], 2); Serial.println(" deg/s");
        // Print mag values in degree/sec
        Serial.print("mx = "); Serial.print((int)m[0]);
        Serial.print(" my = "); Serial.print((int)m[1]);
        Serial.print(" mz = "); Serial.print((int)m[2]); Serial.println(" mG");

    }

    void printCalibration() const
    {
        Serial.println("< calibration parameters >");
        Serial.println("accel bias [g]: ");
        Serial.print(accelBias[0] * 10000.f); Serial.print(", ");
        Serial.print(accelBias[1] * 10000.f); Serial.print(", ");
        Serial.print(accelBias[2] * 10000.f); Serial.println();
        Serial.println("gyro bias [deg/s]: ");
        Serial.print(gyroBias[0]); Serial.print(", ");
        Serial.print(gyroBias[1]); Serial.print(", ");
        Serial.print(gyroBias[2]); Serial.println();
        Serial.println("mag bias [mG]: ");
        Serial.print(magBias[0]); Serial.print(", ");
        Serial.print(magBias[1]); Serial.print(", ");
        Serial.print(magBias[2]); Serial.println();
        Serial.println("mag scale []: ");
        Serial.print(magScale[0]); Serial.print(", ");
        Serial.print(magScale[1]); Serial.print(", ");
        Serial.print(magScale[2]); Serial.println();

        // Serial.println("magCalibration []: ");
        // Serial.print(magCalibration[0]); Serial.print(", ");
        // Serial.print(magCalibration[1]); Serial.print(", ");
        // Serial.print(magCalibration[2]); Serial.println();

    }



    //格式化上传校准信息，便于协议处理
    void reportCalibration() const
    {
        Serial.print("CMAB:");
        Serial.print(accelBias[0] * 10000.f); Serial.print(",");
        Serial.print(accelBias[1] * 10000.f); Serial.print(",");
        Serial.print(accelBias[2] * 10000.f); Serial.println();
        Serial.print("CMGB:");
        Serial.print(gyroBias[0], 4); Serial.print(",");
        Serial.print(gyroBias[1], 4); Serial.print(",");
        Serial.print(gyroBias[2], 4); Serial.println();
        Serial.print("CMMB:");
        Serial.print(magBias[0], 4); Serial.print(",");
        Serial.print(magBias[1], 4); Serial.print(",");
        Serial.print(magBias[2], 4); Serial.println();
        Serial.print("CMMS:");
        Serial.print(magScale[0], 4); Serial.print(",");
        Serial.print(magScale[1], 4); Serial.print(",");
        Serial.print(magScale[2], 4); Serial.println();

        Serial.print("CMMC:");
        Serial.print(magCalibration[0], 4); Serial.print(",");
        Serial.print(magCalibration[1], 4); Serial.print(",");
        Serial.print(magCalibration[2], 4); Serial.println();
    }

protected:

    float accelBias[3] = {0.00745, 0.03503, 0.02692};
    float   gyroBias[3] = {1.39, 1.17, 0.70};
    float   magBias[3] = {-52.48, 619.14, 1015.88};
    float  magScale[3]  = {0.80, 1.15, 1.13}; // Bias corrections for gyro and accelerometer
    float magCalibration[3] = {0, 0, 0}; // factory mag calibration
   
    float  magnetic_declination = -7.51; // Japan, 24th June // -6.59 BeiJing 
    TwoWire* wire;
    uint8_t i2c_err_;


    float a[3], g[3], m[3];

    bool newMagData = false;
    uint8_t rawdata[18];  //the raw data 

    bool b_ahrs {true};

    AFS AFSSEL;
    GFS GFSSEL;
    MFS MFSSEL;


    float getAres() const
    {
        switch (AFSSEL)
        {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case AFS::A2G:  return 2.0 / 32768.0;
            case AFS::A4G:  return 4.0 / 32768.0;
            case AFS::A8G:  return 8.0 / 32768.0;
            case AFS::A16G: return 16.0 / 32768.0;
        }
    }

    float getGres() const
    {
        switch (GFSSEL)
        {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case GFS::G250DPS:  return 250.0 / 32768.0;
            case GFS::G500DPS:  return 500.0 / 32768.0;
            case GFS::G1000DPS: return 1000.0 / 32768.0;
            case GFS::G2000DPS: return 2000.0 / 32768.0;
        }
    }

    float getMres() const
    {
        switch (MFSSEL)
        {
            // Possible magnetometer scales (and their register bit settings) are:
            // 14 bit resolution (0) and 16 bit resolution (1)
            // Proper scale to return milliGauss
            case MFS::M14BITS: return 10. * 4912. / 8190.0;
            case MFS::M16BITS: return 10. * 4912. / 32760.0;
        }
    }




    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
    {
        wire->beginTransmission(address);  // Initialize the Tx buffer
        wire->write(subAddress);           // Put slave register address in Tx buffer
        wire->write(data);                 // Put data in Tx buffer
        i2c_err_ = wire->endTransmission();           // Send the Tx buffer
        if (i2c_err_) printI2CError();
    }

    uint8_t readByte(uint8_t address, uint8_t subAddress)
    {
        uint8_t data = 0; // `data` will store the register data
        wire->beginTransmission(address);         // Initialize the Tx buffer
        wire->write(subAddress);	                 // Put slave register address in Tx buffer
        i2c_err_ = wire->endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_) printI2CError();
        wire->requestFrom(address, (size_t)1);  // Read one byte from slave register address
        if (wire->available()) data = wire->read();                      // Fill Rx buffer with result
        return data;                             // Return data read from slave register
    }

    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
    {
        wire->beginTransmission(address);   // Initialize the Tx buffer
        wire->write(subAddress);            // Put slave register address in Tx buffer
        i2c_err_ = wire->endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_) printI2CError();
        uint8_t i = 0;
        wire->requestFrom(address, count);  // Read bytes from slave register address
        while (wire->available())
        {
            dest[i++] = wire->read();
        } // Put read results in the Rx buffer
    }

    void printI2CError()
    {
        if (i2c_err_ == 7) return; // to avoid stickbreaker-i2c branch's error code
        Serial.print("I2C ERROR CODE : ");
        Serial.println(i2c_err_);
    }




    public:
        virtual bool init(){
          Serial.println("init mpu???");
          };
        virtual void doCalibrate() = 0;
        virtual void update() = 0;
};



#endif // _MPU_H
