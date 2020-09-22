#include "IMU.h"
#include <EEPROM.h>


enum EEP_ADDR
{
    EEP_CALIB_FLAG = 0x00,
    EEP_ACC_BIAS = 0x01,
    EEP_GYRO_BIAS = 0x0D,
    EEP_MAG_BIAS = 0x19,
    EEP_MAG_SCALE = 0x25
};


#define SPEED_LOOP_COUNT 10

void imuIntterrupt();

IMU::IMU()
{
  mIMUReady = false;
 mpu = new MPU9250();
  // mpu = new MPU6050();
  useMag = false;
}

void IMU::init(int gyroRate)
{

    Serial.println("init mpu...");
    Serial.println(digitalPinToInterrupt(IMU_INT_PIN));
    pinMode(IMU_INT_PIN, INPUT);
    // imu intterupt
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIntterrupt, RISING);

    bool ret = mpu->init();

   if(  ret == true )
   {
      loadCalibrationFromEEProm();
      mpu->printCalibration();

    // for( int i=0; i<3; i++)   //校正偏差
    //  {
    //     mpu.setAccBias(i, accelBias[i]);
    //     mpu.setGyroBias(i, gyroBias[i]);
    //     mpu.setMagBias(i, magBias[i]);
    //     mpu.setMagScale(i, magScale[i]);
    //  }
      mpu->setMagneticDeclination(magnetic_declination);
      mIMUReady = true;
      mpu->update();
   }
   else
   {
     mIMUReady = false;
   }
   
  //  prev_millis = millis();

}

void IMU::setFilter(FILTERTYPE iFilter )
{
  mFilterType = iFilter;
}

void IMU::setUseMag( bool val )
{
    useMag = val ;
}


void IMU::loadCalibrationFromEEProm()
{
    if (EEPROM.read(EEP_CALIB_FLAG) == 0x01)
    {
        Serial.println("calibrated to eeprom? : YES");
        Serial.println("load calibrated values");
        
        float value;

        for( int i=0; i<3; i++)
        {
          EEPROM.get( EEP_ACC_BIAS + 4*i,  value);
          mpu->setAccBias(i, value);
        }

        for( int i=0; i<3; i++)
        {
          EEPROM.get( EEP_GYRO_BIAS + 4*i,  value);
          mpu->setGyroBias(i, value);
        }

        for( int i=0; i<3; i++)
        {
          EEPROM.get( EEP_MAG_BIAS + 4*i,  value);
          mpu->setMagBias(i, value);
        }
        for( int i=0; i<3; i++)
        {
          EEPROM.get( EEP_MAG_SCALE + 4*i,  value);
          mpu->setMagScale(i, value);
        }
    }
    else
    {
        Serial.println("calibrated to eeprom? : NO");
    }
}



void IMU::saveCalibrationToEEProm()
{
    Serial.println("Save calibration to eeprom...");
    EEPROM.write(EEP_CALIB_FLAG, 1);
    EEPROM.put(EEP_ACC_BIAS + 0, mpu->getAccBias(0));
    EEPROM.put(EEP_ACC_BIAS + 4, mpu->getAccBias(1));
    EEPROM.put(EEP_ACC_BIAS + 8, mpu->getAccBias(2));
    EEPROM.put(EEP_GYRO_BIAS + 0, mpu->getGyroBias(0));
    EEPROM.put(EEP_GYRO_BIAS + 4, mpu->getGyroBias(1));
    EEPROM.put(EEP_GYRO_BIAS + 8, mpu->getGyroBias(2));
    EEPROM.put(EEP_MAG_BIAS + 0, mpu->getMagBias(0));
    EEPROM.put(EEP_MAG_BIAS + 4, mpu->getMagBias(1));
    EEPROM.put(EEP_MAG_BIAS + 8, mpu->getMagBias(2));
    EEPROM.put(EEP_MAG_SCALE + 0, mpu->getMagScale(0));
    EEPROM.put(EEP_MAG_SCALE + 4, mpu->getMagScale(1));
    EEPROM.put(EEP_MAG_SCALE + 8, mpu->getMagScale(2));

    mpu->reportCalibration();
    
    Serial.println("Save OK.");
}



void IMU::calibrateIMU()
{
  if( !mIMUReady )
    return;

  mpu->doCalibrate();
  mpu->reportCalibration(); //  printCalibration();
  // saveCalibrationToEEProm();

}

void IMU::readIMU(double dt)
{

  if( !mIMUReady )
    return;
  // mpu.readIMU();

  mpu->update();
}

void IMU::calculateAttitute(double dt)
{
  long cur_millis = millis();
  long process_time = cur_millis - prev_millis;
  filter.invSampleFreq = (float)process_time/1000.0f;

  float gx,gy,gz,ax,ay,az,mx,my,mz;
  gx = mpu->getGyro(0);
  gy = mpu->getGyro(1);
  gz = mpu->getGyro(2);

  ax = mpu->getAcc(0);
  ay = mpu->getAcc(1);
  az = mpu->getAcc(2);
  
  if( useMag )
  {
    mx = mpu->getMag(0);
    my = mpu->getMag( 1 );
    mz = mpu->getMag( 2 );
    filter.update(gx, gy, gz, ax, ay, az, my, mx, -mz);
  }
  else
  {
    filter.updateIMU(gx, gy, gz, ax, ay, az);
  }
  prev_millis = cur_millis;
  // filter.updateIMU(gx, gy, gz, ax, ay, az);
 
}


double IMU::getGyro(int idx)
{

  return mpu->getGyro(idx);
}

//四元数
double IMU::getQuaternion(int idx)
{
  switch( idx )
  {
    case 0:
      return filter.q0;
      break;
    case 1:
      return filter.q1;
      break;
    case 2:
      return filter.q2;
      break;
    case 3:
      return filter.q3;
      break;
  }
  return 0;
}

double IMU::getAcceleration(int idx)
{
  return mpu->getAcc(idx); ///error
}



void IMU::debugOut()
{
    Serial.println("IMU info (yaw,pitch,roll)：");
    Serial.print(filter.getYaw(), 2); 
    Serial.print(", "); 
    Serial.print(filter.getPitch(), 2); 
    Serial.print(", "); 
    Serial.println(filter.getRoll(), 2); 

}
