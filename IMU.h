
#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include "MyMPU6050.h"
//#include "MPU6050.h"
#include <MadgwickAHRS.h>
//#include <Kalman.h>

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
  double getYaw() { return filter.getYaw(); };

  //  度/秒 角速度 idx 0，1，2 x y z
  double getGyro(int idx);
  //加速度 g/秒？ idx 0, 1, 2 x y z
  double getAcceleration(int idx);
  //    double roll = filter.getRoll();
  //    pitch = filter.getPitch();
  //    heading = filter.getYaw();

  // void  calculateAngle(double dt);
  // double getKalmanAngle() { return m_kalman_angle; };
 void debugOut();

private:
  //传感器角度（atan（ax/ay）, kalman, madgwick filter, Kalman1
  // double m_sensor_angle, m_kalman_angle, m_km_angle; //m_madgwick_angle
  // double m_gyro;
  // double KG, m_x_angle; // m_x_angle 通过融合滤波得到，用于判断小车被提起

  double convertRawAcceleration(int aRaw);
  double convertRawGyro(int gRaw);

  //一阶融合滤波, angle 当前角度，g_angle重力加速度计角度，gyro 陀螺仪角速度
  // angle = KG * g_angle + (1-KG)*(angle + gyro * dt)
  // double estima_cal(double angle, double g_angle, double gyro, double dt, double KG);
  // Kalman kalman;
  // KalmanFilter km;
  bool mIMUReady;
  MyMPU6050 m_mpu6050;
  Madgwick filter;

private:
  //raw IMU 数据
  int m_aix, m_aiy, m_aiz;
  int m_gix, m_giy, m_giz;

  //转换后的 IMU数据
  double ax, ay, az;
  double gx, gy, gz;

  // void sendIMUInfo();

  // uint8_t i2cData[14]; // Buffer for I2C data
};

#endif /* _BALANCE_SUPERVISOR_H_ */
