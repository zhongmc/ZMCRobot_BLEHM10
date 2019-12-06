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
  delay(100);  

// gyroBias1[3] = {0.96, -0.21, 0.12}

  Serial.print(gyroBias1[0]);
  Serial.write(',');
  Serial.print(gyroBias1[1]);
  Serial.write(',');
  Serial.println(gyroBias1[2]);


  MPU9250.I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71 
  
  uint8_t c = MPU9250.getMPU9250ID(MPU1);
  Serial.print("MPU9250_1 "); Serial.print("I AM "); Serial.println(c, HEX); //Serial.print(" I should be "); Serial.println(0x71, HEX);

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
  filter.update(gx, -gy, -gz, -ax, ay, az, -mx, my, mz);
  // filter.updateIMU(gx, gy, gz, ax, ay, az);

  deltat1 = dt;

  float pi = 3.141592653589793238462643383279502884f;
    // for( int i=0; i<10; i++)
    MadgwickQuaternionUpdate1(-ax, +ay, +az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  -mx, my, mz); //  my1,  -mx1, mz1);



    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch1 = -asinf(a32);
    roll1  = atan2f(a31, a33);
    yaw1   = atan2f(a12, a22);
    pitch1 *= 180.0f / pi;
    yaw1   *= 180.0f / pi; 
    yaw1   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw1 < 0) yaw1   += 360.0f; // Ensure yaw stays between 0 and 360
    roll1  *= 180.0f / pi;
    lin_ax1 = ax + a31;
    lin_ay1 = ay + a32;
    lin_az1 = az - a33;

}


void IMU9250::debugOut()
{
    Serial.println("IMU info (yaw,pitch,roll)：");
    Serial.print("madgwick:");
    Serial.print(filter.getYaw(), 2); 
    Serial.print(", "); 
    Serial.print(filter.getPitch(), 2); 
    Serial.print(", "); 
    Serial.println(filter.getRoll(), 2); 

    Serial.print("kriswiner:");
    Serial.print(yaw1, 2); 
    Serial.print(", "); 
    Serial.print(pitch1, 2); 
    Serial.print(", "); 
    Serial.println(roll1, 2); 

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




__attribute__((optimize("O3"))) void IMU9250::MadgwickQuaternionUpdate1(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{

  float pi = 3.141592653589793238462643383279502884f;
  float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
  float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta


          float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;
            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;
            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;
            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;
            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;
            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;
            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
            // Integrate to yield quaternion
            q1 += qDot1 * deltat1;
            q2 += qDot2 * deltat1;
            q3 += qDot3 * deltat1;
            q4 += qDot4 * deltat1;
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
        }



//angle = (0.98)*(angle + gyro * dt) + (0.02)*(x_acc);
//一阶融合滤波, angle 当前角度，g_angle重力加速度计角度，gyro 陀螺仪角速度
// angle = KG * g_angle + (1-KG)*(angle + gyro * dt)
// double IMU::estima_cal(double angle, double g_angle, double gyro, double dt, double KG)
// {
//   double result = KG * g_angle + (1 - KG) * (angle + gyro * dt);
//   return result;
// }
