#pragma once
#ifndef _IMU_H_
#define _IMU_H_

#define IMU_INT_PIN 19

#include "ZMCRobot.h"
#include "MPU.h"
#include "MPU9250.h"
#include "MPU6050.h"

#include <Arduino.h>
#include "MadgwickAHRS.h"

enum class FILTERTYPE {MADGWICK, MADG, MAHONY};

#define GYRO_RATE 100

class IMU
{
public:
  IMU();
  // void resetKalman();
  void init(int gyroRate );
  // void getIMUInfo(double *buf, double dt);
  //filter paramaters
  double KG_ANG;
  
  void readIMU(double dt);
  void calculateAttitute(double td);

  double getRoll() { return filter.getRoll(); };
  double getPitch() { return filter.getPitch(); };
  double getYaw()  { return filter.getYaw();} // { return filter.getYaw()-180.0; };

  double getYawRadians()  { return filter.getYawRadians();} // { return filter.getYaw()-180.0; };

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

  //选择滤波器
  void setFilter(FILTERTYPE iFilter );
  void setUseMag( bool val );

  int8_t *getRawData()
  {
    return mpu->getRawData();
  }

private:
  bool mIMUReady;
  FILTERTYPE mFilterType;
  bool useMag;

  MPU *mpu;

// #if MPU == 9250
//   class MPU9250_<TwoWire, AFS::A2G, GFS::G250DPS, MFS::M14BITS> mpu;
// #else
//   class MPU6050_<TwoWire, AFS::A2G, GFS::G250DPS> mpu;
// #endif


  Madgwick filter;
  unsigned long prev_millis;

private:
// These can be measured once and entered here or can be calculated each time the device is powered on

float magnetic_declination =  -6.59; // -6.59 BeiJing // -7.51;  Japan, 24th June


};

#endif /* _BALANCE_SUPERVISOR_H_ */
