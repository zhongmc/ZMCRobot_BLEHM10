#include <Arduino.h>
#include "Wire.h"


void sendBleMessages(byte *tmp, uint8_t len );


#define SLAVE_ADDRESS 0x38

//Control mode 2 or 3 pin (HALF/ FULL); with 3 pin mode, with brake function,call StopMotor() will brake the motor
#define FULL_CTRL_MODE
// #define HALF_CTRL_MODE 

#define LEFT_WHEEL_DIR 4
#define LEFT_WHEEL_PWM 5
#define LEFT_WHEEL_DIR_B 10

#define RIGHT_WHEEL_DIR 7
#define RIGHT_WHEEL_PWM 6
#define RIGHT_WHEEL_DIR_B 11

//read in whell dir
#define LEFT_WHEEL_A 3
#define LEFT_WHEEL_B 9 //12
#define RIGHT_WHEEL_A 2
#define RIGHT_WHEEL_B 8 //13  2 8

unsigned U0L = 0, U0R = 0;

volatile long count1 = 0;
volatile long count2 = 0;

unsigned long micros1, micros2;

uint16_t counter = 0;
uint16_t r_counter = 0;

byte motorType = 0;

byte I2CCmd;



void longToBytes(long value, byte *buffer)
{
  boolean sign = false;
  if( value < 0 )
  {
    sign = true;
    value = -value;
  }

  for(uint8_t i=0; i<4; i++ )
  {
    *(buffer+i) = value;
    value=value/256;
  }

  if( sign )
    *(buffer+3) = *(buffer+3) | 0x80;
}


int bytesToInt(byte *buf)
{
  int value = *(buf+1) & 0x7f;
  value = value*256 + *buf;
  if( (*(buf+1) & 0x80 ))
    value = -value;
  return value;
}

long bytesToLong(byte *buf)
{
    long value = 0;
    boolean sign = false;
    if(( *(buf + 3) & 0x80 ) != 0 )
    {
      sign = true;
      *(buf + 3) = *(buf + 3) & 0x7f;
    }

    for(uint8_t i=0; i<4; i++ )
    {
      value= value*256;
      value = value + *(buf+3-i);
    }

    if( sign )
      value = - value;

  return value;
  
}

void doSendCounter()
{

  if( motorType != 2 )
    return;


  counter++;
  byte buf[10];
  longToBytes(count1, buf);
  longToBytes(count2, buf+4);
  buf[8] = counter & 0xff;
  buf[9] = (counter/256)&0xff;
  Wire.write(buf, 10);

  // Serial.print("Send Cnt:");
  // for( uint8_t k=0; k<i; k++ )
  // {
  //     Serial.print(" ");
  //     Serial.print(buf[k]);
  // }
  // Serial.println();
}

void sendCounterInfo(int idt)
{

    if( motorType == 0)
    {
      r_counter++;
    }

    byte buffer[18];
    buffer[0] = 0xA4;
    buffer[1] = 12;

    longToBytes(count1, buffer + 2);
    longToBytes(count2, buffer + 6);

    buffer[10] = (byte)idt;
    buffer[11] = (byte)(idt/256);

    buffer[12] = (byte)r_counter;
    buffer[13] = (byte)(r_counter/256);

    sendBleMessages(buffer, 14);

}

 Position readPosition()
 {
    Position pos;
    if( motorType != 1 )
    {
      pos.x = 0;
      pos.y = 0;
      pos.theta = 0;
      return pos;
    }

  byte buffer[8];
  memset(buffer, 0, 8);
  Wire.beginTransmission(SLAVE_ADDRESS);   // Initialize the Tx buffer
  Wire.write('P');            // Put slave register address in Tx buffer
  uint8_t i2c_err_ = Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
  // if (i2c_err_) pirntI2CError();
  uint8_t i = 0;
  Wire.requestFrom(SLAVE_ADDRESS, 8);  // Read bytes from slave register address
  while (Wire.available() > 0 )
  {
    buffer[i++] = Wire.read();
  } // Put read results in the Rx buffer

  int x,y,q;
  x = bytesToInt(buffer);
  y = bytesToInt(buffer+2);
  q = bytesToInt(buffer+4);

  pos.x = (double)x/1000.0;
  pos.y = (double)y/1000.0;
  pos.theta = (double)q/1000.0;
  return pos;

 }



// extern double x, y, theta;
void doSendPosition()
{
  // byte buf[8];
  // double scale = 1000;
  // floatToByte(buf, x, scale);
  // floatToByte(buf + 2, y, scale);
  // floatToByte(buf + 4, theta, scale);
  // Wire.write(buf, 8);
}




//master 端请求数据，必须在这里响应，发送数据
void requestEvent()
{
  if( I2CCmd == 'P' )
    doSendPosition();
  else
    doSendCounter();
}

//master 端请求数据前发送的数据
void receiveEvent(int count) {
  I2CCmd = Wire.read();
  if( I2CCmd == 'R') //reset counter
  {
    count1 = 0;
    count2 = 0;
  }
}


void initMotorPins()
{

  // pinMode(LEFT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_WHEEL_PWM, OUTPUT);

  //  pinMode(RIGHT_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_WHEEL_PWM, OUTPUT);

  //moto dir
  pinMode(LEFT_WHEEL_B, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_B, INPUT_PULLUP);

  analogWrite(LEFT_WHEEL_PWM, 0);
  analogWrite(RIGHT_WHEEL_PWM, 0);

  digitalWrite(LEFT_WHEEL_DIR, HIGH);
  digitalWrite(RIGHT_WHEEL_DIR, HIGH);

#ifdef  FULL_CTRL_MODE
  pinMode(LEFT_WHEEL_DIR_B, OUTPUT);
  pinMode(RIGHT_WHEEL_DIR_B, OUTPUT);
  digitalWrite(LEFT_WHEEL_DIR_B, LOW);
  digitalWrite(RIGHT_WHEEL_DIR_B, LOW);
  Serial.println("Full ctrl mode!");
#else
  Serial.println("Half ctrl mode!");

#endif

  count1 = 0;
  count2 = 0;
}

void initAsSlave()
{
  Serial.println("init motor as slave ...");
  pinMode(LEFT_WHEEL_A, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_A, INPUT_PULLUP);

  //改为change 让精度增加一倍
  attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_A), Code1, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_A), Code2, FALLING);
  initMotorPins();

  Wire.begin(SLAVE_ADDRESS);    // join I2C bus as a slave with address 1
  Wire.setClock(400000); // I2C frequency at 400 kHz
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event

}


void initAsMaster()
{
  Serial.println("init motor as master ...");
  initMotorPins();
  // testLong2Bytes();
}



void initMotor( byte mode ) // 0 normal 1 master 2 slave 
{

    motorType = mode;

    if( mode == 0 )
    {
      Serial.println("init motor as normal ...");
      pinMode(LEFT_WHEEL_A, INPUT_PULLUP);
      pinMode(RIGHT_WHEEL_A, INPUT_PULLUP);

      //改为change 让精度增加一倍
      attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_A), Code1, FALLING);
      attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_A), Code2, FALLING);
      initMotorPins();

    }
    else if( mode == 1 )
    {
      initAsMaster();
    }
    else
    {
      initAsSlave();
    }
    

}


void resetCounter()
{

  if( motorType == 0)
  {
    count1 = 0;
    count2 = 0;
    r_counter = 0;
  }
  else if( motorType == 1)
  {
    byte buffer[10];
    memset(buffer, 0, 10);
    Wire.beginTransmission(SLAVE_ADDRESS);   // Initialize the Tx buffer
    Wire.write('R');            // Put slave register address in Tx buffer
    uint8_t i2c_err_ = Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
    // if (i2c_err_) pirntI2CError();
    uint8_t i = 0;
    Wire.requestFrom(SLAVE_ADDRESS, 10);  // Read bytes from slave register address
    while (Wire.available() > 0 )
    {
      buffer[i++] = Wire.read();
    } // Put read results in the Rx buffer

    count1 = bytesToLong(buffer);
    count2 = bytesToLong(buffer+4);
    r_counter = buffer[8] + 256*buffer[9];
  }
}

void readCounter()
{
  if( motorType == 0 || motorType == 2)
    return;

  byte buffer[10];
  memset(buffer, 0, 10);

  Wire.beginTransmission(SLAVE_ADDRESS);   // Initialize the Tx buffer
  Wire.write('C');            // Put slave register address in Tx buffer
  uint8_t i2c_err_ = Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
  // if (i2c_err_) pirntI2CError();
  uint8_t i = 0;
  Wire.requestFrom(SLAVE_ADDRESS, 10);  // Read bytes from slave register address
  while (Wire.available() > 0 )
  {
    buffer[i++] = Wire.read();
  } // Put read results in the Rx buffer

  count1 = bytesToLong(buffer);
  count2 = bytesToLong(buffer+4);
  r_counter = buffer[8] + 256*buffer[9];
 
 }


void StopMotor()
{
#ifdef  FULL_CTRL_MODE
  analogWrite(LEFT_WHEEL_PWM, 0);
  analogWrite(RIGHT_WHEEL_PWM, 0);
  digitalWrite(LEFT_WHEEL_DIR, LOW);
  digitalWrite(RIGHT_WHEEL_DIR, LOW);
  digitalWrite(LEFT_WHEEL_DIR_B, LOW);
  digitalWrite(RIGHT_WHEEL_DIR_B, LOW);
  digitalWrite(LEFT_WHEEL_PWM, HIGH);
  digitalWrite(RIGHT_WHEEL_PWM, HIGH);
#else
  analogWrite(LEFT_WHEEL_PWM, 0);
  analogWrite(RIGHT_WHEEL_PWM, 0);
#endif
}

void MoveMotor(int pwm)
{

  MoveLeftMotor(pwm);
  MoveRightMotor(pwm);
}

void MoveLeftMotor(int PWM)
{
  int pwm_out;
  if (PWM >= 0)
  {
    pwm_out = PWM + U0L;
    if (pwm_out > 255)
      pwm_out = 255;
    digitalWrite(LEFT_WHEEL_DIR, HIGH);
    #ifdef  FULL_CTRL_MODE
    digitalWrite(LEFT_WHEEL_DIR_B, LOW);
    #endif
  }
  else
  {
    pwm_out = -1 * PWM + U0L;
    if (pwm_out > 255)
      pwm_out = 255;
    digitalWrite(LEFT_WHEEL_DIR, LOW);
    #ifdef  FULL_CTRL_MODE
    digitalWrite(LEFT_WHEEL_DIR_B, HIGH);
    #endif
  }
  analogWrite(LEFT_WHEEL_PWM, pwm_out);
}

void MoveRightMotor(int PWM)
{
  int pwm_out;
  if (PWM > 0)
  {
    pwm_out = PWM + U0R;
    if (pwm_out > 255)
      pwm_out = 255;
    digitalWrite(RIGHT_WHEEL_DIR, LOW);
    #ifdef  FULL_CTRL_MODE
    digitalWrite(RIGHT_WHEEL_DIR_B, HIGH);
    #endif
  }
  else
  {
    pwm_out = -1 * PWM + U0R;
    if (pwm_out > 255)
      pwm_out = 255;
    digitalWrite(RIGHT_WHEEL_DIR, HIGH);
    #ifdef  FULL_CTRL_MODE
    digitalWrite(RIGHT_WHEEL_DIR_B, LOW);
    #endif
  }
  analogWrite(RIGHT_WHEEL_PWM, pwm_out);
}

//speed counter for left
void Code1()
{
  micros1 = micros();
  int wheelDir = digitalRead(LEFT_WHEEL_B);
  if (wheelDir == HIGH)
    count1++;
  else
    count1--;
}

//speed counter for right
void Code2()
{
  micros2 = micros();
  int wheelDir = digitalRead(RIGHT_WHEEL_B);
  if (wheelDir == LOW) //HIGH)
    count2++;
  else
    count2--;
}

long readLeftEncoder()
{
  return count1;
}

long readRightEncoder()
{

  return count2;
}

long getWheelsPosition()
{
  return count1 + count2;
}

