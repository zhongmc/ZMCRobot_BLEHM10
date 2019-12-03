
#ifndef _IMU9250_H_
#define _IMU9250_H_

#include <Arduino.h>
#include "MPU9250.h"
//#include "MPU6050.h"
#include <MadgwickAHRS.h>
//#include <Kalman.h>

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
  MPU9250 MPU9250;
//   MyMPU6050 m_mpu6050;
  Madgwick filter;

private:
  //raw IMU 数据
  int m_aix, m_aiy, m_aiz;
  int m_gix, m_giy, m_giz;

  //转换后的 IMU数据
  double ax, ay, az;
  double gx, gy, gz;
  double mx, my, mz;

  // void sendIMUInfo();

  // MPU9250 Configuration

// Specify sensor full scale

/* Choices are:

 *  Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale

 *  Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale

 *  Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB

 *  Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate

 *  (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 

 *  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.

 */

uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         

float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors


int16_t MPU9250Data1[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro

int16_t magCount1[3];    // Stores the 16-bit signed magnetometer sensor output

float   magCalibration1[3] = {0, 0, 0};  // Factory mag calibration and mag bias
  // uint8_t i2cData[14]; // Buffer for I2C data


// These can be measured once and entered here or can be calculated each time the device is powered on
float   gyroBias1[3] = {0.96, -0.21, 0.12}, accelBias1[3] = {0.00299, -0.00916, 0.00952};
float   magBias1[3] = {71.04, 122.43, -36.90}, magScale1[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


};

#endif /* _BALANCE_SUPERVISOR_H_ */
