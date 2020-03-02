#include "ZMCRobot.h"
#include <Arduino.h>

// #include "CurieTimerOne.h"
// #include <CurieBLE.h>
#include "IRSensor.h"
#include "Robot.h"


#include "Supervisor.h"
#include "DriveSupervisor.h"
#include "BlinkLed.h"
//#include "BlinkMatrixLed.h"
#include "IMU9250.h"
#include "IMU.h"

#include "pitches.h"





//#include "MyKey.h"
//#include "MyMenu.h"

#define VOLT_IN_PIN A0
#define ULTRASONIC_ECHO 18  //2,3,18,19(serial 1), 20, 21(I2C) external interrupt
#define ULTRASONIC_TRIG 10


#define START_KEY 0
#define LEFT_KEY 1
#define RIGHT_KEY 2
#define RET_KEY 3

#define START_KEY_PIN 11
#define LEFT_KEY_PIN 10
#define RIGHT_KEY_PIN 9
#define RET_KEY_PIN 8

byte currentState = STATE_IDLE;

//MyKey myKey;
// IMU9250 mIMU;

#if MPU == 9250
  IMU9250 mIMU;
#else
  IMU mIMU;
#endif

bool mIMUDataOk = false;

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
byte settingsReqQueue[8];
short queueLen = 0;

extern volatile long count1, count2;
extern int comDataCount;

//char *titles[] = {"Self balance", "Cruise", "Speed ", "Start", "Remote by BLE", "To Target", "Target X:", "Target Y:", "Start",
//                         "PID of Balance", "KP: ", "KI: ", "KD: ", "Config", "Calibrate Motor", "balance angle"
//                        };

//menu_item menuItems[17];
//MyMenu menu(&menuItems[0]);

unsigned long millisPrevKey, millisPrev, loopExecuteTime, imuMillisPrev;
int imuCycle;
int sampleRate = 10;  // sampleTime = 10 * sampleRate (ms);

//bool backLightOn = false;
// LiquidCrystal_I2C lcd(0x27, 16, 2);

//  GP2Y0A41 = 0,     //4-30cm
//  GP2Y0A21 = 1     //10-80cm
IRSensor irSensor(GP2Y0A41);
double irDistance[5];
Position pos;

double batteryVoltage;  // Measured battery level
uint8_t batteryCounter, batteryLowCount; // Counter used to check if it should check the battery level

uint8_t driverCounter; 


void setup()
{

  Serial.begin(115200);
  delay(100);
  //lcd.init(); // initialize the lcd
  //lcd.backlight(); //Open the backlight
  //  lcd.noBacklight();
  //  lcd.print(" "); // Print a message to the LCD.
  // showMainTips();

  /*
  Serial.println("Initialze key...");
  myKey.addKey(RET_KEY, RET_KEY_PIN);
  myKey.addKey(RIGHT_KEY, RIGHT_KEY_PIN);
  myKey.addKey(LEFT_KEY, LEFT_KEY_PIN);
  myKey.addKey(START_KEY, START_KEY_PIN);


  myKey.initKey();
*/
  // // start the IMU and filter
  // CurieIMU.begin();
  // CurieIMU.setGyroRate(GYRO_RATE);
  // CurieIMU.setAccelerometerRate(GYRO_RATE);
  // // Set the accelerometer range to 2G
  // CurieIMU.setAccelerometerRange(2);
  // // Set the gyroscope range to 250 degrees/second
  // CurieIMU.setGyroRange(250);

  // filter.begin(GYRO_RATE);

  initMotor();

  // initialize variables to pace updates to correct rate
  //  microsPerReading = 1000000 / GYRO_RATE;  //25
  //  microsPrevious = micros();

  initBluetooth();
  //  Serial.println(F("Initialze MENU..."));
  //
  //void MyMenu::setKeyId(byte stKey, byte rtKey, byte lKey, byte rKey)
  //  menu.setKeyId(START_KEY, RET_KEY,  LEFT_KEY, RIGHT_KEY);
  //  initMenu();

  //  const int oneSecInUsec = 1000000;   // A second in mirco second unit.
  // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
  //  CurieTimerOne.start(oneSecInUsec / GYRO_RATE, &timedBlinkIsr);  // set timer and callback

  //  Serial.println(sizeof(long));

  mIMU.init( 10 );  //gyroRate (sample frenquence)

  imuMillisPrev = millis();

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

  // SETTINGS mSettings;
  // mSettings.sType = 0;

  // mSettings.sType = 0;
  // mSettings.kp = 5;
  // mSettings.ki = 0.01;
  // mSettings.kd = 0.05;

  // mSettings.max_rpm = 200;
  // mSettings.min_rpm = 40; //45

  // mSettings.atObstacle = 0.25; //0.15
  // mSettings.unsafe = 0.1;
  // mSettings.dfw = 0.2;      //0.25
  // mSettings.velocity = 0.2; //0.3

  // supervisor.updateSettings(mSettings);
  // driveSupervisor.updateSettings(mSettings);

  SETTINGS sett = supervisor.getSettings();

  sampleRate = sett.sampleTime/10;

  Serial.print("SampleRate:");
  Serial.println( sampleRate );
  
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

  playMelody(melody, noteDurations, 8);

  #if OP_VOLTAGE == VOLT33
    Serial.println("Work at 3.3V...");
  #else
    Serial.println("Work at 5V...");
  #endif


  millisPrevKey = millis();
  millisPrev = millisPrevKey; //millis();
  driverCounter = 0;
  batteryLowCount = 0;
}

void loop()
{
  checkSerialData();
  doBleHM10Loop();
  blinkLed.beSureToBlink();
  //ble cmd process
  processSetingsRequire();
  //ultrasonic process
  processUltrasonic();

  unsigned long millisNow;
  if( mIMUDataOk )
  {
    millisNow = millis();
    imuCycle = millisNow - imuMillisPrev;
    imuMillisPrev = millisNow;
    
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
      }

  }


  millisNow = millis();
  if (millisNow - millisPrev >= 10)
  {
     millisPrev = millisNow;
    //  mIMU.readIMU((millisNow - millisPrev)/1000.0);           //1/GYRO_RATE
    //  mIMU.calculateAttitute(  (millisNow - millisPrev)/1000.0); //1/GYRO_RATE
    //   if( mROSConnected )
    //   {
    //       int16_t *rawData = mIMU.getRawData();
    //       Serial.print("RD");
    //       for( int i=0; i<8; i++)
    //       {
    //         Serial.print(rawData[i]);
    //         Serial.print(',');
    //       }
    //       Serial.println(rawData[8]);
    //   }
    driverCounter++;
    if( driverCounter >= sampleRate )
    {
      driverCounter = 0;
      if (currentState == STATE_GOTOGOAL)
      {
        //report states
        supervisor.execute(readLeftEncoder(), readRightEncoder(), mIMU.getGyro(2),  0.01*sampleRate);
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
        driveSupervisor.execute(readLeftEncoder(), readRightEncoder(), mIMU.getGyro(2), 0.01*sampleRate); //1/20
        driveSupervisor.getIRDistances(irDistance);
        pos = driveSupervisor.getRobotPosition();
        // sendRobotStateValue(8, pos, irDistance, batteryVoltage);
      }
      else
      {
        supervisor.readIRDistances(irDistance);
        if( mUseIMU )
        {
          pos.theta = (PI * mIMU.getYaw()) / 180.0;
          // supervisor.setRobotPosition(pos.x, pos.y, pos.theta);
          // driveSupervisor.setRobotPosition(pos.x, pos.y, pos.theta);
        }
        // sendRobotStateValue(8, pos, irDistance, batteryVoltage);
      }

//report Robot States
        sendRobotStateValue(8, pos, irDistance, batteryVoltage);

        if( mROSConnected )
        {

          log("RP%d,%d,%d,%d,%d\n",
                (int)(10000 * pos.x),
                (int)(10000 * pos.y),
                (int)(10000 * pos.theta),
                (int)(10000 * pos.w),
                (int)(10000 * pos.v));

          log("IR%d,%d,%d,%d,%d\n",
                (int)(100 * irDistance[0]),
                (int)(100 * irDistance[1]),
                (int)(100 * irDistance[2]),
                (int)(100 * irDistance[3]),
                (int)(100 * irDistance[4]));

          log("IM%d,%d,%d,%d\n",
              (int)(1000* mIMU.getQuaternion(0)),
              (int)(1000* mIMU.getQuaternion(1)),
              (int)(1000* mIMU.getQuaternion(2)),
              (int)(1000* mIMU.getQuaternion(3))
          );
        }
                    
      batteryCounter++;
      if (batteryCounter >= 100/sampleRate )  // Measure battery every 1s
      { 
        batteryCounter = 0;
        if (isBatteryLow())
        {
          Serial.println("Bat L...");
          batterLowCount++;
          if( doCheckBattleVoltage && batterLowCount == 3)
          {

            if (currentState != STATE_IDLE)
            {
              stopAndReset();
              currentState = STATE_IDLE;

                int melody[] = {
                  NOTE_C4, NOTE_G3,NOTE_C4, NOTE_G3
              };
              int noteDurations[] = {
              4, 8, 8, 4
              };

              playMelody(melody, noteDurations, 4);

            }
            blinkLed.slowBlink();
          }
        }
        else
        {
          if( batteryLowCount != 0 )
            Serial.println("Bat ok.");
          batteryLowCount = 0;
        }
        
      }
    }
    loopExecuteTime = millis() - millisNow;
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

void setGoal(double x, double y, int theta, double v)
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

  if (currentState == STATE_DRIVE)
  {
    Position pos = driveSupervisor.getRobotPosition();
    supervisor.setRobotPosition(pos.x, pos.y, pos.theta);
  }
  else if (currentState == STATE_GOTOGOAL)
  {
    Position pos = supervisor.getRobotPosition();
    driveSupervisor.setRobotPosition(pos.x, pos.y, pos.theta);
  }
  currentState = STATE_IDLE;
  stopAndReset();
}

void setDriveGoal(double v, double w)
{
  if (currentState == STATE_DRIVE)
  {
    if (abs(v) < 0.001 && abs(w) < 0.001) //stop
      stopRobot();
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

void playMelody(int melody[], int noteDurations[], int len )
{
  
    for (int thisNote = 0; thisNote < len; thisNote++)
    {
      int noteDuration = 1000/noteDurations[thisNote];
      tone(11, melody[thisNote],noteDuration);
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      noTone(11);
    }

}