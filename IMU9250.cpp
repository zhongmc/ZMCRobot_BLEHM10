
#include "IMU9250.h"
#include "ZMCRobot.h"

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

IMU9250::IMU9250()
{
  mIMUReady = false;
}

void IMU9250::init(int gyroRate)
{

  Serial.println("init mpu9250...");

    Serial.print("dtoi(19):");
    Serial.println(digitalPinToInterrupt(IMU_INT_PIN));
    pinMode(IMU_INT_PIN, INPUT);
    // imu intterupt
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIntterrupt, RISING);


  Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(100);  

   if(  mpu.setup() == true )
   {
      loadCalibrationFromEEProm();
      
       mpu.printCalibration();

    // for( int i=0; i<3; i++)   //校正偏差
    //  {
    //     mpu.setAccBias(i, accelBias[i]);
    //     mpu.setGyroBias(i, gyroBias[i]);
    //     mpu.setMagBias(i, magBias[i]);
    //     mpu.setMagScale(i, magScale[i]);
    //  }

    mpu.setMagneticDeclination(magnetic_declination);
    mIMUReady = true;
    mpu.update();
   }
   else
   {
     mIMUReady = false;
   }
   

}


void IMU9250::loadCalibrationFromEEProm()
{
    if (EEPROM.read(EEP_CALIB_FLAG) == 0x01)
    {
        Serial.println("calibrated to eeprom? : YES");
        Serial.println("load calibrated values");
        
        float value;

        for( int i=0; i<3; i++)
        {
          EEPROM.get( EEP_ACC_BIAS + 4*i,  value);
          mpu.setAccBias(i, value);
        }

        for( int i=0; i<3; i++)
        {
          EEPROM.get( EEP_GYRO_BIAS + 4*i,  value);
          mpu.setGyroBias(i, value);
        }

        for( int i=0; i<3; i++)
        {
          EEPROM.get( EEP_MAG_BIAS + 4*i,  value);
          mpu.setMagBias(i, value);
        }
        for( int i=0; i<3; i++)
        {
          EEPROM.get( EEP_MAG_SCALE + 4*i,  value);
          mpu.setMagScale(i, value);
        }
    }
    else
    {
        Serial.println("calibrated to eeprom? : NO");
    }
}



void IMU9250::saveCalibrationToEEProm()
{
    Serial.println("Save calibration to eeprom...");
    EEPROM.write(EEP_CALIB_FLAG, 1);
    EEPROM.put(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
    EEPROM.put(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
    EEPROM.put(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
    EEPROM.put(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
    EEPROM.put(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
    EEPROM.put(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
    EEPROM.put(EEP_MAG_BIAS + 0, mpu.getMagBias(0));
    EEPROM.put(EEP_MAG_BIAS + 4, mpu.getMagBias(1));
    EEPROM.put(EEP_MAG_BIAS + 8, mpu.getMagBias(2));
    EEPROM.put(EEP_MAG_SCALE + 0, mpu.getMagScale(0));
    EEPROM.put(EEP_MAG_SCALE + 4, mpu.getMagScale(1));
    EEPROM.put(EEP_MAG_SCALE + 8, mpu.getMagScale(2));

    // eeprom.writeByte(EEP_CALIB_FLAG, 1);
    // eeprom.writeFloat(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
    // eeprom.writeFloat(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
    // eeprom.writeFloat(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
    // eeprom.writeFloat(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
    // eeprom.writeFloat(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
    // eeprom.writeFloat(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
    // eeprom.writeFloat(EEP_MAG_BIAS + 0, mpu.getMagBias(0));
    // eeprom.writeFloat(EEP_MAG_BIAS + 4, mpu.getMagBias(1));
    // eeprom.writeFloat(EEP_MAG_BIAS + 8, mpu.getMagBias(2));
    // eeprom.writeFloat(EEP_MAG_SCALE + 0, mpu.getMagScale(0));
    // eeprom.writeFloat(EEP_MAG_SCALE + 4, mpu.getMagScale(1));
    // eeprom.writeFloat(EEP_MAG_SCALE + 8, mpu.getMagScale(2));
    // if (b_save) eeprom.commit();
    Serial.println("Save OK.");
}



void IMU9250::calibrateIMU()
{
  if( !mIMUReady )
    return;

  mpu.calibrateAccelGyro();
  mpu.calibrateMag();

  mpu.printCalibration();
  saveCalibrationToEEProm();

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
