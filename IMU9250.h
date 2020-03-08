#pragma once
#ifndef _IMU9250_H_
#define _IMU9250_H_

#define IMU_INT_PIN 19

#include <Arduino.h>
#include "MPU9250.h"



#define GYRO_RATE 100

class IMU9250
{
public:
  IMU9250();
  // void resetKalman();
  void init(int gyroRate );
  // void getIMUInfo(double *buf, double dt);
  //filter paramaters
  double KG_ANG;
  void readIMU(double dt);
  void calculateAttitute(double td);
  double getRoll() { return mpu.getRoll(); };
  double getPitch() { return mpu.getPitch(); };
  double getYaw() { return mpu.getYaw(); };

  //  度/秒 角速度 idx 0，1，2 x y z
  double getGyro(int idx);
  double getQuaternion(int idx);
  //加速度 g/秒？ idx 0, 1, 2 x y z
  double getAcceleration(int idx);
  void debugOut();

  void loadCalibrationFromEEProm();
  void saveCalibrationToEEProm();
  
  //校准 IMU
  void calibrateIMU();

  int16_t *getRawData()
  {
    return mpu.getRawData();
  }

private:
  bool mIMUReady;
  
  // MPU9250 mpu;
class MPU9250_<TwoWire, AFS::A2G, GFS::G250DPS, MFS::M14BITS> mpu;
private:
// These can be measured once and entered here or can be calculated each time the device is powered on

//company
    // float accelBias[3] = {0.0036, 0.00507, 0.01746}; // bias corrections for accelerometer
    // float gyroBias[3] = {-1.63, -0.23, -0.11}; // bias corrections for gyro
    // float magBias[3] = {-106.43, -18.10, 220.92}; // Bias corrections for mag
    // float magScale[3]  = {0.90, 0.85, 1.4}; //




//home
// float accelBias[3] = {0.00745, 0.03503, 0.02692};
// float   gyroBias[3] = {1.39, 1.17, 0.70};
// float   magBias[3] = {-52.48, 619.14, 1015.88};
// float  magScale[3]  = {0.80, 1.15, 1.13}; // Bias corrections for gyro and accelerometer

// float accelBias[3] = {0.00720, 0.03748, 0.02045};
// float   gyroBias[3] = {1.39, 0.96, 0.79};
// float   magBias[3] = {434.35, 384.92, 500.98};
// float  magScale[3]  = {0.81, 0.87, 1.62}; // Bias corrections for gyro and accelerometer


float magnetic_declination =  -6.59; // -6.59 BeiJing // -7.51;  Japan, 24th June

// 2020-03-08 09:31:54 INFO  Simulator:587 - 434.35, 384.92, 500.98
// 2020-03-08 09:31:54 INFO  Simulator:587 - AK8963 mag scale (mG)
// 2020-03-08 09:31:54 INFO  Simulator:587 - 0.81, 0.87, 1.62
// 2020-03-08 09:31:54 INFO  Simulator:587 - < calibration parameters >
// 2020-03-08 09:31:54 INFO  Simulator:587 - accel bias [g]: 
// 2020-03-08 09:31:54 INFO  Simulator:587 - 7.20, 37.48, 20.45
// 2020-03-08 09:31:54 INFO  Simulator:587 - gyro bias [deg/s]: 
// 2020-03-08 09:31:54 INFO  Simulator:587 - 1.39, 0.96, 0.79
// 2020-03-08 09:31:54 INFO  Simulator:587 - mag bias [mG]: 
// 2020-03-08 09:31:54 INFO  Simulator:587 - 434.35, 384.92, 500.98
// 2020-03-08 09:31:54 INFO  Simulator:587 - mag scale []: 
// 2020-03-08 09:31:54 INFO  Simulator:587 - 0.81, 0.87, 1.62


};

#endif /* _BALANCE_SUPERVISOR_H_ */
