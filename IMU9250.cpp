
#include "IMU9250.h"
#include "ZMCRobot.h"

#define SPEED_LOOP_COUNT 10

void imuIntterrupt();

IMU9250::IMU9250()
{
  mIMUReady = false;
}

void IMU9250::init(int gyroRate)
{

  Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(100);  

   if(  mpu.setup() == true )
   {

     for( int i=0; i<3; i++)   //校正偏差
     {
        mpu.setAccBias(i, accelBias[i]);
        mpu.setGyroBias(i, gyroBias[i]);
        mpu.setMagBias(i, magBias[i]);
        mpu.setMagScale(i, magScale[i]);
     }
     mpu.setMagneticDeclination(magnetic_declination);
      mIMUReady = true;

  pinMode(IMU_INT_PIN, INPUT_PULLUP);
  // imu intterupt
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIntterrupt, RISING);


   }
   else
   {
     mIMUReady = false;
   }
   

}


void IMU9250::calibrateIMU()
{
  if( !mIMUReady )
    return;

  mpu.calibrateAccelGyro();
  mpu.calibrateMag();
  mpu.printCalibration();

}

void IMU9250::readIMU(double dt)
{

  if( !mIMUReady )
    return;
  mpu.update();
}

double IMU9250::getGyro(int idx)
{

  return mpu.getGyro(idx);
}

//四元数
double IMU9250::getQuaternion(int idx)
{
  return mpu.getQuaternion(idx);

}

double IMU9250::getAcceleration(int idx)
{
  return mpu.getAcc(idx); ///error
}

//call readIMU() first
void IMU9250::calculateAttitute(double dt)
{
 
}


void IMU9250::debugOut()
{
    Serial.println("IMU info (yaw,pitch,roll)：");
    Serial.print(mpu.getYaw(), 2); 
    Serial.print(", "); 
    Serial.print(mpu.getPitch(), 2); 
    Serial.print(", "); 
    Serial.println(mpu.getRoll(), 2); 

}
