#include "ZMCRobot.h"
#include <Arduino.h>

#include "Robot.h"

#include "Supervisor.h"
#include "DriveSupervisor.h"
#include "BlinkLed.h"

#include "IMU9250.h"
#include "IMU.h"

#include "pitches.h"

#include "commFunctions.h"
//#include "MyKey.h"
//#include "MyMenu.h"

#define VOLT_IN_PIN A0
#define ULTRASONIC_ECHO 18  //2,3,18,19(serial 1), 20, 21(I2C) external interrupt
#define ULTRASONIC_TRIG 12

#define MELODY_PIN 48

byte currentState = STATE_IDLE;

//MyKey myKey;
// IMU9250 mIMU;

#if MPU == 9250
  IMU9250 mIMU;
#else
  IMU mIMU;
#endif

bool mIMUDataOk = false;

// RearDriveRobot robot();
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

//char *titles[] = {"Self balance", "Cruise", "Speed ", "Start", "Remote by BLE", "To Target", "Target X:", "Target Y:", "Start",
//                         "PID of Balance", "KP: ", "KI: ", "KD: ", "Config", "Calibrate Motor", "balance angle"
//                        };

//menu_item menuItems[17];
//MyMenu menu(&menuItems[0]);

unsigned long loopExecuteTime = 0;
int imuCycle;
int sampleTime = 30; // (sample time 30 ms);
unsigned long prevSampleMillis, prevBattMillis, prevImuMillis, prevStateMillis;
//bool backLightOn = false;
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
  initMotor();
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

  SETTINGS sett = supervisor.getSettings();
  sampleTime = sett.sampleTime;

  Serial.print("Sample time:");
  Serial.println( sampleTime );
  
  supervisor.init();
  driveSupervisor.init();

  bExecDrive = false;
  bExecGTG = false;

  blinkLed.init();
  blinkLed.normalBlink();

  Serial.println("READY!");

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
  millisNow = millis();

  if( mIMUDataOk )
  {
    imuCycle = millisNow - prevImuMillis;
    prevImuMillis = millisNow;
    
    mIMUDataOk = false;

     mIMU.readIMU( 0 );           //1/GYRO_RATE
     mIMU.calculateAttitute( 0 ); //1/GYRO_RATE

      if( mROSConnected )
      {
          int16_t *rawData = mIMU.getRawData();
          Serial.print("RD");
          for( int i=0; i<8; i++)
          {
            Serial.print(rawData[i]);
            Serial.print(',');
          }
          Serial.println(rawData[8]);

          log("IM%d,%d,%d,%d\n",
              (int)(1000* mIMU.getQuaternion(0)),
              (int)(1000* mIMU.getQuaternion(1)),
              (int)(1000* mIMU.getQuaternion(2)),
              (int)(1000* mIMU.getQuaternion(3))
          );

      }

  }

  if ( (millisNow - prevSampleMillis) >= sampleTime ) 
  {
      double dt = (double)(millisNow -  prevSampleMillis )/1000.0;
       prevSampleMillis = millisNow;
      double yaw = (PI * mIMU.getYaw()) / 180.0;
      if (currentState == STATE_GOTOGOAL)
      {
        //report states
        supervisor.execute(readLeftEncoder(), readRightEncoder(), yaw, dt);
        if( supervisor.mSimulateMode )
        {
          delay(3);
          checkSerialData(); //wait for 
        }
        supervisor.getIRDistances(irDistance);
        pos = supervisor.getRobotPosition();
        // sendRobotStateValue(8, pos, irDistance, batteryVoltage);
      }
      else if (currentState == STATE_DRIVE)
      {
        driveSupervisor.execute(readLeftEncoder(), readRightEncoder(), yaw, dt); //1/20
        driveSupervisor.getIRDistances(irDistance);
        pos = driveSupervisor.getRobotPosition();
        // sendRobotStateValue(8, pos, irDistance, batteryVoltage);
      }
      else
      {
        supervisor.readIRDistances(irDistance);
        if( mUseIMU )
        {
          pos.theta = yaw;  //mIMU.getYawRadians(); 
          supervisor.setRobotPosition(pos.x, pos.y, pos.theta);
          driveSupervisor.setRobotPosition(pos.x, pos.y, pos.theta);
        }
        // sendRobotStateValue(8, pos, irDistance, batteryVoltage);
//report Robot States
    // SendRobotStates( pos, irDistance, batteryVoltage);

      }

    
    long execTime =  millis() - millisNow;
    if( execTime > loopExecuteTime)
      loopExecuteTime = execTime;
  }

  if( millisNow - prevStateMillis >= 60 )
  {
//report Robot States
    prevStateMillis =  millisNow;
    // SendRobotStates( pos, irDistance, batteryVoltage);
    sendRobotStateValue( pos, irDistance, batteryVoltage);
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

Position getRobotPosition()
{
    if (currentState == STATE_DRIVE)
    {
      return driveSupervisor.getRobotPosition();
    }
    else
    {
      return supervisor.getRobotPosition();
    }
    
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

  stopRobot(); //stop currentState

  // supervisor.updateSettings(mSettings);
  Serial.print("Start GTG:");
  Serial.print(supervisor.m_Goal.x);
  Serial.print(",");
  Serial.println(supervisor.m_Goal.y);

  // to test set goal y to 0
  supervisor.reset(readLeftEncoder(), readRightEncoder());
  currentState = STATE_GOTOGOAL;

  const int oneSecInUsec = 1000000;                     // A second in mirco second unit.
  // CurieTimerOne.start(oneSecInUsec / 20, &goToGoalIsr); // set timer and callback //the controller loop need 30ms to exec
}

void goToGoalIsr()
{
  bExecGTG = true;
  // supervisor.execute(readLeftEncoder(), readRightEncoder(), 0.05);
}

void ResetRobot()
{
  supervisor.resetRobot();
  driveSupervisor.resetRobot();
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
}

void startDrive()
{
  // if ( currentState >= 2 )
  //   return;

  Serial.println("Start DRV!");
  currentState = STATE_DRIVE;

  // to test set goal y to 0
  driveSupervisor.reset(readLeftEncoder(), readRightEncoder());
  //   currentState = STATE_DRIVE;

  // const int oneSecInUsec = 1000000; // A second in mirco second unit.
  // // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
  // CurieTimerOne.start(oneSecInUsec / 20, &driveIsr); // set timer and callback
}

void driveIsr()
{
  bExecDrive = true;
  // driveSupervisor.execute(readLeftEncoder(), readRightEncoder(), 0.05); //1/20
}

void SetSimulateMode(int val)
{
  // supervisor.mSimulateMode = val;
  supervisor.setSimulateMode(val);
  driveSupervisor.mSimulateMode = (val != 0);
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
  // CurieTimerOne.kill();
  StopMotor();
  delay(100);

  if (currentState == STATE_DRIVE)
  {
    driveSupervisor.update(readLeftEncoder(), readRightEncoder(), 0.03); //处理当前运动的惯性
    Position pos = driveSupervisor.getRobotPosition();
    supervisor.setRobotPosition(pos.x, pos.y, pos.theta);
  }
  else if (currentState == STATE_GOTOGOAL)
  {
    supervisor.update(readLeftEncoder(), readRightEncoder(), 0.03);  //处理当前运动的惯性
    Position pos = supervisor.getRobotPosition();
    driveSupervisor.setRobotPosition(pos.x, pos.y, pos.theta);
  }
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
      stopRobot();
    }
    // else
    driveSupervisor.setGoal(v, w);
  }
  else
  {
    stopRobot(); //stop currentState
    startDrive();
    driveSupervisor.setGoal(v, w);
  }
}

void turnAround(int dir, int angle )
{
  if( currentState != STATE_DRIVE )
  {
    stopRobot(); //stop currentState
    startDrive();
  }
  driveSupervisor.turnAround(dir, angle );
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
  else
  {
    long curTime = millis();
    if (curTime - lastTrigTimer < 40)
      return;
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

void setIMUFilter(int iFilter, bool withMag )
{
  switch( iFilter )
  {
    case 0:
      mIMU.setFilter( FILTER::MADGWICK);
      Serial.println("set IMU Filter to: MADGWICK!");
      if( withMag )
        Serial.println(" with mag TRUE!");
      else
        Serial.println(" with mag FALSE!");
          
      break;
    case 1:
      mIMU.setFilter(FILTER::MADG );
      Serial.println("set IMU Filter to: MADG!");
      break;
    case 2:
      mIMU.setFilter(FILTER::MAHONY );
      Serial.println("set IMU Filter to: MAHONY!");
      break;
  }
 
  mIMU.setUseMag( withMag );
}

void setUseIMU(bool useImu,  float alpha )
{
    mUseIMU = useImu;
    driveSupervisor.setUseIMU(useImu, alpha);
    supervisor.setUseIMU(useImu, alpha);
    log("use IMU:%d,%s\n", useImu, floatToStr(0, alpha));

}

