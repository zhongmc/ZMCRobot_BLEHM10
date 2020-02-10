#include "IMU.h"
#include "ZMCRobot.h"

#define SPEED_LOOP_COUNT 10

int doubleToStr(double val, int scale, char *buf, char append);

IMU::IMU()
{

  // KG_ANG = 0.02; //0.2
  // KG = 0.05;
  // m_x_angle = 0;
  mIMUReady = false;
}

void IMU::init(int gyroRate)
{

  int ret = m_mpu6050.initialize();
  if( ret == 0 )
    mIMUReady = true;
  else
  {
      Serial.println("IMU not pressent!");
  }
  
  filter.begin( gyroRate );

}

// void IMU::getIMUInfo(double *buf, double dt)
// {
//   readIMU(dt);
//   buf[0] = m_sensor_angle; // m_sensor_angle;
//   buf[1] = m_kalman_angle; //m_estima_angle;
//   buf[2] = m_gyro;         //g_fGravityAngle;
//   buf[3] = 0;              //mBalancePWM;
//   buf[4] = m_x_angle;
// }

int doubleToStr(double val, int scale, char *buf, char append)
{
  itoa((int)(val * scale), buf, 10);
  int len = strlen(buf);
  if (append != 0)
  {
    *(buf + len) = append;
    *(buf + len + 1) = 0;
    return len + 1;
  }
  else
    return len;
}

// void IMU::sendIMUInfo()
// {
//   char buf[200];
//   int len = 0, off = 0;
//   buf[0] = 'M';
//   buf[1] = 'U';
//   off = 2;

//   len = doubleToStr(m_sensor_angle, 100, buf + off, ',');
//   off = off + len;
//   len = doubleToStr(m_gyro, 100, buf + off, ',');
//   off = off + len;
//   len = doubleToStr(m_kalman_angle, 100, buf + off, ',');
//   off = off + len;
//   len = doubleToStr(m_km_angle, 100, buf + off, 0);
//   off = off + len;
//   // len = doubleToStr(mSpeedPWM, 100, buf + off, ',');
//   // off = off + len;
//   // len = doubleToStr(robot.velocity * 100, 100, buf + off, ',');
//   // off = off + len;
//   // len = doubleToStr(pwm_l, 100, buf + off, 0);
//   // off = off + len;
//   Serial.write(buf);
//   Serial.write('\r');
//   Serial.write('\n');
// }

// void IMU::resetKalman()
// {
//   readIMU(0);
//   double Angle_accY = atan2((double)ay, (double)az) * RAD_TO_DEG;
//   kalman.setAngle(Angle_accY);
// }

void IMU::readIMU(double dt)
{

  if( !mIMUReady )
    return;
    
  m_mpu6050.readMotionSensor(m_aix, m_aiy, m_aiz, m_gix, m_giy, m_giz);
  // CurieIMU.readMotionSensor(m_aix, m_aiy, m_aiz, m_gix, m_giy, m_giz);
  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(m_aix);
  ay = convertRawAcceleration(m_aiy);
  az = convertRawAcceleration(m_aiz);
  gx = convertRawGyro(m_gix);
  gy = convertRawGyro(m_giy);
  gz = convertRawGyro(m_giz);
}

double IMU::getGyro(int idx)
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


void IMU::calibrateIMU()
{
  
}

double IMU::getQuaternion(int idx)
{
  //tobo done 
    // filter.;
    return 0.0;
}

double IMU::getAcceleration(int idx)
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
void IMU::calculateAttitute(double dt)
{
  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);
}


void IMU::debugOut()
{
    Serial.println("IMU info (yaw,pitch,roll)：");
    Serial.print("madgwick:");
    Serial.print(filter.getYaw(), 2); 
    Serial.print(", "); 
    Serial.print(filter.getPitch(), 2); 
    Serial.print(", "); 
    Serial.println(filter.getRoll(), 2); 
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

double IMU::convertRawAcceleration(int aRaw)
{
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  double a = (aRaw * 2.0) / 32768.0;
  return a;
}

//度每秒
double IMU::convertRawGyro(int gRaw)
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
