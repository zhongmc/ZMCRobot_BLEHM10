#include "IMU9250.h"
#include "ZMCRobot.h"

#define SPEED_LOOP_COUNT 10


IMU9250::IMU9250()
{
  // KG_ANG = 0.02; //0.2
  // KG = 0.05;
  // m_x_angle = 0;
  mIMUReady = false;
}

void IMU9250::init(int gyroRate)
{

  Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(1000);  

  MPU9250.I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71 
  
  uint8_t c = MPU9250.getMPU9250ID(MPU1);
  Serial.print("MPU9250_1 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 

  {  

    Serial.println("MPU9250 online...");
    MPU9250.resetMPU9250(MPU1); // start by resetting MPU9250_1
    mIMUReady = true;

    aRes = MPU9250.getAres(Ascale);
    gRes = MPU9250.getGres(Gscale);
    mRes = MPU9250.getMres(Mscale);

    MPU9250.initMPU9250(MPU1, Ascale, Gscale, sampleRate); 

    MPU9250.initAK8963Slave(MPU1, Mscale, Mmode, magCalibration1);
  }
  else
  {
    mIMUReady = false;
  }

  filter.begin( gyroRate );

}


void IMU9250::readIMU(double dt)
{

  if( !mIMUReady )
    return;
    

  MPU9250.readMPU9250Data(MPU1, MPU9250Data1); // INT cleared on any read
  // Now we'll calculate the accleration value into actual g's
  ax = (float)MPU9250Data1[0]*aRes - accelBias1[0];  // get actual g value, this depends on scale being set

     ay = (float)MPU9250Data1[1]*aRes - accelBias1[1];   

     az = (float)MPU9250Data1[2]*aRes - accelBias1[2];  



    // Calculate the gyro value into actual degrees per second

     gx = (float)MPU9250Data1[4]*gRes;  // get actual gyro value, this depends on scale being set

     gy = (float)MPU9250Data1[5]*gRes;  

     gz = (float)MPU9250Data1[6]*gRes; 

  

//    if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set

      MPU9250.readMagData(MPU1, magCount1);  // Read the x/y/z adc values

  

    // Calculate the magnetometer values in milliGauss

    // Include factory calibration per data sheet and user environmental corrections

      mx = (float)magCount1[0]*mRes*magCalibration1[0] - magBias1[0];  // get actual magnetometer value, this depends on scale being set

      my = (float)magCount1[1]*mRes*magCalibration1[1] - magBias1[1];  

      mz = (float)magCount1[2]*mRes*magCalibration1[2] - magBias1[2];  

      mx *= magScale1[0];

      my *= magScale1[1];

      mz *= magScale1[2]; 



}

double IMU9250::getGyro(int idx)
{
  switch (idx)
  {
  case 0:
    return gx;
    break;
  case 1:
    return gy;
    break;
  case 2:
    return gz;
    break;
  }

  return 0; ///error
}

double IMU9250::getAcceleration(int idx)
{
  switch (idx)
  {
  case 0:
    return ax;
    break;
  case 1:
    return ay;
    break;
  case 2:
    return az;
    break;
  }
  return 0; ///error
}

//call readIMU() first
void IMU9250::calculateAttitute(double dt)
{
  // update the filter, which computes orientation
  //(-ax1, +ay1, +az1, gx1*pi/180.0f, -gy1*pi/180.0f, -gz1*pi/180.0f,  my1,  -mx1, mz1);
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  // filter.updateIMU(gx, gy, gz, ax, ay, az);
}

//call readIMU() first
// void IMU::calculateAngle(double dt)
// {

//   m_gyro = gx; //
//   // double Angle_accY = atan(ay / sqrt(ax * ax + az * az)) * 180 / 3.14; //offset
//   // double m_sensor_angle = atan2((double)ay, (double)az) * RAD_TO_DEG;
//   double Angle_accY = atan2((double)ay, (double)az) * RAD_TO_DEG;
//   m_sensor_angle = Angle_accY;                          //filter.getRoll();
//   m_kalman_angle = kalman.getAngle(Angle_accY, gx, dt); // Calculate the angle using a Kalman filter

//   // double Angle_accY = atan2((double)ay, (double)az) * RAD_TO_DEG;
//   // m_km_angle = km.getAngle(Angle_accY, gx, dt);

//   double angle_accX = atan2((double)ax, (double)az) * RAD_TO_DEG;
//   m_x_angle = estima_cal(m_x_angle, angle_accX, gy, dt, 0.02);
//   m_km_angle = estima_cal(m_km_angle, m_sensor_angle, gx, dt, KG_ANG);
// }

double IMU9250::convertRawAcceleration(int aRaw)
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  double a = (aRaw * 2.0) / 32768.0;
  return a;
}

//度每秒
double IMU9250::convertRawGyro(int gRaw)
{
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  double g = (gRaw * 250.0) / 32768.0;
  return g;
}

//angle = (0.98)*(angle + gyro * dt) + (0.02)*(x_acc);
//一阶融合滤波, angle 当前角度，g_angle重力加速度计角度，gyro 陀螺仪角速度
// angle = KG * g_angle + (1-KG)*(angle + gyro * dt)
// double IMU::estima_cal(double angle, double g_angle, double gyro, double dt, double KG)
// {
//   double result = KG * g_angle + (1 - KG) * (angle + gyro * dt);
//   return result;
// }
