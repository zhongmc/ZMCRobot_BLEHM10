#include "MyMPU6050.h"


MyMPU6050::MyMPU6050()
{
  
}

int MyMPU6050::initialize()
{
  Serial.println("init imu...");
    Wire.begin();
    // #if ARDUINO >= 157
      Wire.setClock(400000UL); // Set I2C frequency to 400kHz
    // #else
    //   TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
    // #endif

    delay(200);
    
    i2cRead(0x75, i2cBuffer, 1);
    if (i2cBuffer[0] != 0x68) { // Read "WHO_AM_I" register
        Serial.print("Error reading sensor ");
        Serial.println( i2cBuffer[0] );
        return -1;
    }

  i2cWrite(0x6B, 0x80, true); // Reset device, this resets all internal registers to their default values
  int c = 0;
  do {
    i2cRead(0x6B, i2cBuffer, 1);
    c++;
    if( c>10)
    {
      Serial.println("failed1!");
      return -1;
      // break;
    }
  } while (i2cBuffer[0] & 0x80); // Wait for the bit to clear
  
  delay(5);
  i2cWrite(0x6B, 0x09, true); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode

  i2cBuffer[0] = 1; // Set the sample rate to 500Hz - 1kHz/(1+1) = 500Hz
  i2cBuffer[1] = 0x03; // Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling

   i2cWrite(0x19, i2cBuffer, 4, true); // Write to all four registers at once
    delay(100); // Wait for the sensor to get ready  
    Serial.println("IMU ready.");
    return 0;
}


       
void MyMPU6050::readMotionSensor(int &ax, int &ay, int &az, int &gx, int &gy, int &gz)
{
  
   while (i2cRead(0x3B, i2cBuffer, 14));
   ax = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
   ay = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
   az = ((i2cBuffer[4] << 8) | i2cBuffer[5]);

   gx = ((i2cBuffer[8] << 8) | i2cBuffer[9]);
   gy = ((i2cBuffer[10] << 8) | i2cBuffer[11]);
   gz = ((i2cBuffer[12] << 8) | i2cBuffer[13]);
   
}

uint8_t MyMPU6050::i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t MyMPU6050::i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t MyMPU6050::i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

