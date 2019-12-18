#pragma once
#ifndef _IMU9250_H_
#define _IMU9250_H_

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
  //加速度 g/秒？ idx 0, 1, 2 x y z
  double getAcceleration(int idx);
  void debugOut();

private:
  bool mIMUReady;
  
  // MPU9250 mpu;
class MPU9250_<TwoWire, AFS::A2G, GFS::G250DPS, MFS::M14BITS> mpu;
private:
// These can be measured once and entered here or can be calculated each time the device is powered on

//company
    float gyroBias[3] = {-1.52, -0.27, -0.08}; // bias corrections
    float accelBias[3] = {0.03894, -0.00537, 0.011678}; // bias corrections
    float magBias[3] = {-81.18, -164.69, 330.51};
    float magScale[3]  = {1.02, 0.86, 1.17}; // Bias corrections for gyro and accelerometer

//  accel bias [g]: 
//  39.73, -3.23, 14.89
//  gyro bias [deg/s]: 
//  -1.56, -0.22, -0.14
//  mag bias [mG]: 
//  -81.18, -164.69, 330.51
//  mag scale []: 
//  1.02, 0.86, 1.17



//home
// float   gyroBias[3] = {1.39, 1.17, 0.70};
// float accelBias[3] = {0.00745, 0.03503, 0.02692};
// float   magBias[3] = {-52.48, 619.14, 1015.88};
// float  magScale[3]  = {0.80, 1.15, 1.13}; // Bias corrections for gyro and accelerometer

float magnetic_declination =  -6.59; // -6.59 BeiJing // -7.51;  Japan, 24th June


};

#endif /* _BALANCE_SUPERVISOR_H_ */
