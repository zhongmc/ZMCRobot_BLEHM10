#include "ZMCRobot.h"
#include <Arduino.h>

#include "Robot.h"

#include "Supervisor.h"
#include "DriveSupervisor.h"
#include "BlinkLed.h"

#include "IMU.h"


#include "pitches.h"

#include "commFunctions.h"
//#include "MyKey.h"
//#include "MyMenu.h"



#define VOLT_IN_PIN A0
#define ULTRASONIC_ECHO 18  //2,3,18,19(serial 1), 20, 21(I2C) external interrupt
#define ULTRASONIC_TRIG 12

#define MELODY_PIN 48

#define BUZZ_TONE 1
#define BUZZ_NOTONE 0

#define BUZZER 1


void sendIMURawData();
void readCounter();
void sendRobotStateWithCounter(Robot *robot, double ultraDist, double voltage, int idt);


extern byte  info_required;   //0 none 1 ctrlInfo 2 count info

byte currentState = STATE_IDLE;

IMU mIMU;
bool mIMUDataOk = false;

RearDriveRobot robot;
Supervisor supervisor;
DriveSupervisor driveSupervisor;

BlinkLed blinkLed;

//BlinkMatrixLed blinkLed;
bool bExecDrive, bExecGTG;

bool mUseIMU = false;

long trigTime, echoTime;
double ultrasonicDistance;
bool waitForEcho = false;
long lastTrigTimer = 0;


bool mROSConnected = false;

bool doCheckBattleVoltage = true;
bool openDebug = false;

extern volatile long count1, count2;
extern int comDataCount;


double m_right_ticks = 0, m_left_ticks = 0;
bool mSimulateMode = false;

unsigned int loopExecuteTime = 0;
int imuCycle;
int sampleTime = 30; // (sample time 30 ms);
unsigned long prevSampleMillis, prevBattMillis, prevImuMillis, prevStateMillis;

double irDistance[5];
Position pos;

double batteryVoltage;  // Measured battery level
uint8_t batteryLowCount; // Counter used to check if it should check the battery level

void setup()
{


  Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(100);  
  Serial.begin(115200);
  delay(100);

  // initAsMaster();
  initMotor(1);   // 0 normal 1 as master 2 as slave
  // initBlueTooth( 115200 );
  initBluetooth();

  mIMU.init( 10 );  //gyroRate (sample frenquence)
  prevImuMillis = millis();

  pinMode(13, OUTPUT);

  comDataCount = 0;
  count1 = 0;
  count2 = 0;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(ULTRASONIC_ECHO, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  waitForEcho = false;
  lastTrigTimer = 0;
 
  // ultr sound echo intterupt
  attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO), UltrasonicEcho, CHANGE);
  TrigUltrasonic();

  SETTINGS sett = robot.getSettings();
  sampleTime = sett.sampleTime;
  supervisor.updateSettings(sett);
  driveSupervisor.updateSettings(sett);

  Serial.print("Sample time:");
  Serial.println( sampleTime );
  

  bExecDrive = false;
  bExecGTG = false;

  blinkLed.init();
  blinkLed.normalBlink();

  interrupts();
 
  // bCount = 0;
  int melody[] = {
        NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4
    };
    int noteDurations[] = {
    4, 8, 8, 4,4,4,4,4
    };

  playMelody(MELODY_PIN, melody, noteDurations, 8);

  #if OP_VOLTAGE == VOLT33
    Serial.println("Work at 3.3V...");
  #else
    Serial.println("Work at 5V...");
  #endif

  Serial.println("READY!");

  prevSampleMillis = millis();
  prevBattMillis = prevSampleMillis; //millis();
  prevStateMillis = prevSampleMillis;
  batteryLowCount = 0;
}

void loop()
{
  checkSerialData();
  BLEHM10Loop();
  blinkLed.beSureToBlink();
  //ultrasonic process
  processUltrasonic();
  unsigned long millisNow;
  if( mIMUDataOk )
  {
    imuCycle = millisNow - prevImuMillis;
    prevImuMillis = millisNow;
    mIMUDataOk = false;
    mIMU.readIMU( 0 );           //1/GYRO_RATE
    mIMU.calculateAttitute( 0 ); //1/GYRO_RATE
    if( mROSConnected )
    {
      sendIMURawData();
    }

  }

  millisNow = millis();
  if ( (millisNow - prevSampleMillis) >= sampleTime ) 
  {
    int idt = millisNow -  prevSampleMillis;
    double dt = (double)idt/1000.0;
    prevSampleMillis = millisNow;
    double yaw = mIMU.getYawRadians(); //(PI * mIMU.getYaw()) / 180.0;
    readCounter();
    
    if (mSimulateMode)
      robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
    else
    {
        robot.updateState(readLeftEncoder(), readRightEncoder(), yaw, dt);
    }
    Output m_output;
    m_output.vel_l = 0;
    m_output.vel_r = 0;
    m_output.v = 0;
    m_output.w = 0;

    if (currentState == STATE_GOTOGOAL)
    {
      m_output = supervisor.execute(&robot, yaw, dt);
      if( mSimulateMode )
      {
        delay(3);
        checkSerialData(); //wait for 
      }
    }
    else if (currentState == STATE_DRIVE)
    {
      m_output = driveSupervisor.execute(&robot, yaw, dt); //1/20
    }

    int pwm_l = robot.vel_l_to_pwm(m_output.vel_l );
    int pwm_r = robot.vel_r_to_pwm(m_output.vel_r );


    if (mSimulateMode)
    {
      m_left_ticks = m_left_ticks + robot.pwm_to_ticks_l( pwm_l, dt);
      m_right_ticks = m_right_ticks + robot.pwm_to_ticks_r(pwm_r, dt);

    }
    else
    {
      MoveLeftMotor(pwm_l);
      MoveRightMotor(pwm_r);
    }


    if( info_required == 1 && (currentState == STATE_GOTOGOAL || currentState == STATE_DRIVE))
    {
      byte ctrl_info[20];
      ctrl_info[0] = 0xA3;
      ctrl_info[1] = 16;
      floatToByte(ctrl_info + 2, robot.velocity, 1000);
      floatToByte(ctrl_info + 4, robot.w, 1000);
      floatToByte(ctrl_info + 6, m_output.m_v, 1000);
      floatToByte(ctrl_info + 8, m_output.m_w, 1000);
      floatToByte(ctrl_info + 10, m_output.v, 1000);
      floatToByte(ctrl_info + 12, m_output.w, 1000);
      floatToByte(ctrl_info + 14, m_output.vel_l, 1000);
      floatToByte(ctrl_info + 16, m_output.vel_r, 1000);
      sendBleMessages(ctrl_info, 18);
    }

  //report Robot States
  // sendRobotStateValue( pos, irDistance, batteryVoltage);
   sendRobotStateWithCounter(&robot, ultrasonicDistance, batteryVoltage, idt);

    unsigned int execTime =  millis() - millisNow;
    if( execTime > loopExecuteTime)
      loopExecuteTime = execTime;
    TrigUltrasonic();
  }


  if (millisNow - prevBattMillis >= 1000 ) 
  {
    prevBattMillis =  millisNow;
    if (isBatteryLow())
    {
       batteryLowCount++;
       if( doCheckBattleVoltage && batteryLowCount == 3)
       {
          Serial.print(batteryVoltage);
          Serial.println(" Bat L...");
          if (currentState != STATE_IDLE)
          {
              StopMotor();
               currentState = STATE_IDLE;
              int melody[] = {
               NOTE_C4, NOTE_G3,NOTE_C4, NOTE_G3
              };
            int noteDurations[] = {
             4, 8, 8, 4
             };

            playMelody(MELODY_PIN, melody, noteDurations, 4);
          }
          blinkLed.slowBlink();
        }
     
     }
     else
     {
        if( batteryLowCount >= 3 )
          Serial.println("Bat ok.");
        batteryLowCount = 0;
     }
    
  }

}


void sendIMURawData()
{
  int8_t *rawData = mIMU.getRawData();
  byte buf[3];

  buf[0] = 0xA2;  //A为二进制报文标志，0 为报文类型
  buf[1] = 18;   //长度
  
  Serial.write(buf[0]);
  Serial.write(buf[1]);

  for( int i=0; i<18; i++)
    Serial.write( rawData[i] );

  Serial.flush();          
}

double getYaw()
{
  return mIMU.getYaw();
}

void setGoal(double x, double y, double theta, double v)
{
  // count1 = 0;
  // count2 = 0;

  supervisor.setGoal(x, y, theta, v);
  Serial.print("SG:");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(theta);
  Serial.print(",");
  Serial.println(v);

}

void startGoToGoal()
{
  if (currentState == STATE_GOTOGOAL)
    return;
  // supervisor.updateSettings(mSettings);
  Serial.print("Start GTG:");
  Serial.print(supervisor.m_Goal.x);
  Serial.print(",");
  Serial.println(supervisor.m_Goal.y);
  supervisor.reset();
  currentState = STATE_GOTOGOAL;
}

void goToGoalIsr()
{
  bExecGTG = true;
}

void ResetRobot()
{
  resetCounter();
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;

  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
}

void startDrive()
{
  Serial.println("Start DRV!");
  currentState = STATE_DRIVE;
  driveSupervisor.reset(robot.theta);
}

void driveIsr()
{
  bExecDrive = true;
}


void SetSimulateMode(int val)
{
  mSimulateMode = val;
  m_left_ticks = 0;
  m_right_ticks = 0;
  if (val != 0 )
  {
    doCheckBattleVoltage = false;
    Serial.print("simulate: ");
    Serial.println( val );
  }
  else
  {
    doCheckBattleVoltage = true;
    Serial.println("Close simulate!");
  }
}

void SetIgnoreObstacle(bool igm)
{
  Serial.print("ignore obs:");
  Serial.println(igm);
  supervisor.mIgnoreObstacle = igm;
  driveSupervisor.mIgnoreObstacle = igm;
}

void stopRobot()
{
  blinkLed.normalBlink();
  StopMotor();
  currentState = STATE_IDLE;
}


void setDriveGoal(double v, double w)
{
  if (currentState == STATE_DRIVE)
  {
    if (abs(v) < 0.001 && abs(w) < 0.001) //stop
    {
      v = 0;
      w = 0;
      //todo  stop motor
      StopMotor();
      currentState = STATE_IDLE;
    }
    // else
    driveSupervisor.setGoal(v, w);
  }
  else
  {
    startDrive();
    driveSupervisor.setGoal(v, w);
  }
}

void turnAround(int dir, int angle, bool useIMU )
{
  if( currentState != STATE_DRIVE )
  {
    stopRobot(); //stop currentState
    startDrive();
  }
  driveSupervisor.turnAround(dir, angle, useIMU, robot.theta, mIMU.getYawRadians() );
}

void turnAround(int dir, int angle, double w )
{
  if( currentState != STATE_DRIVE )
  {
    stopRobot(); //stop currentState
    startDrive();
  }
  driveSupervisor.turnAround(dir, angle, w,  robot.theta, mIMU.getYawRadians() );
}



bool isBatteryLow()
{
#if OP_VOLTAGE == VOLT33
    batteryVoltage = (double)analogRead(VOLT_IN_PIN) * 0.0352771 + 0.2;
#else //5V
    batteryVoltage = (double)analogRead(VOLT_IN_PIN) * 0.0537109375 + 0.9;
#endif

  if ((batteryVoltage < 9 && batteryVoltage < 7.2) || (batteryVoltage > 9 && batteryVoltage < 11.2)) // && batteryVoltage > 5) // Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
    return true;
  else
    return false;
}


void processUltrasonic()
{
  if (waitForEcho)
  {
    if (echoTime > 0)
    {
      ultrasonicDistance = 0.00017 * echoTime;
      if (ultrasonicDistance > MAX_ULTRASONIC_DIS)
        ultrasonicDistance = MAX_ULTRASONIC_DIS - 0.01;
      waitForEcho = false;
      echoTime = 0;
    }
    else if (millis() - lastTrigTimer > 50)
    {
      waitForEcho = false;
      //      Serial.println("u ...");
      ultrasonicDistance = 1.2; //MAX_ULTRASONIC_DIS;
    }
  }
}

//触发超声波，启动测距
void TrigUltrasonic()
{
    long curTime = millis();

    if( waitForEcho == true )
      return;

    // if (curTime - lastTrigTimer < sampleTime )
    //   return;

    lastTrigTimer = curTime;
    waitForEcho = true;
    digitalWrite(ULTRASONIC_TRIG, HIGH); //trig the ultrosonic

    long curMicros = micros();
    while (true) //10us pules
    {
      if (micros() - curMicros > 10)
        break;
    }
    digitalWrite(ULTRASONIC_TRIG, LOW); //trig the ultrosonic
}


void imuIntterrupt()
{
  mIMUDataOk = true;
}

//the ultrasonic isr service
void UltrasonicEcho()
{

  int echoSig = digitalRead(ULTRASONIC_ECHO);
  if (echoSig == HIGH)
  {
    trigTime = micros();
    echoTime = 0;
  }
  else
    echoTime = micros() - trigTime;
}


#if BUZZER == BUZZ_TONE 

void playMelody(uint8_t pin, int melody[], int noteDurations[], int len )
{
  
    for (int thisNote = 0; thisNote < len; thisNote++)
    {
      int noteDuration = 1000/noteDurations[thisNote];
      tone(pin, melody[thisNote],noteDuration);
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      noTone(pin);
    }

}

#else

void playMelody(uint8_t pin, int melody[], int noteDurations[], int len )
{
    pinMode(pin, OUTPUT);
    for (int thisNote = 0; thisNote < len; thisNote++)
    {
      int noteDuration = 1000/noteDurations[thisNote];
      digitalWrite(pin, HIGH);
      delay( noteDuration );
      // tone(pin, melody[thisNote],noteDuration);

      digitalWrite(pin, LOW);

      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // noTone(pin);
    }

}

#endif


void setIMUFilter(int iFilter, bool withMag )
{
  switch( iFilter )
  {
    case 0:
      mIMU.setFilter( FILTERTYPE::MADGWICK);
      Serial.println("set IMU Filter to: MADGWICK!");
      if( withMag )
        Serial.println(" with mag TRUE!");
      else
        Serial.println(" with mag FALSE!");
          
      break;
    case 1:
      mIMU.setFilter(FILTERTYPE::MADG );
      Serial.println("set IMU Filter to: MADG!");
      break;
    case 2:
      mIMU.setFilter(FILTERTYPE::MAHONY );
      Serial.println("set IMU Filter to: MAHONY!");
      break;
  }
 
  mIMU.setUseMag( withMag );
}

void setUseIMU(bool useImu,  float alpha )
{
    mUseIMU = useImu;
    robot.setUseIMU(useImu, alpha);
    log("use IMU:%d,%s\n", useImu, floatToStr(0, alpha));

}
