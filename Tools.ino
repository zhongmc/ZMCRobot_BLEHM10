#include "ZMCRobot.h"

#include "IMU.h"

#include "Supervisor.h"
#include "DriveSupervisor.h"
extern Supervisor supervisor;
extern DriveSupervisor driveSupervisor;
extern RearDriveRobot robot;

extern long trigTime, echoTime;
extern double ultrasonicDistance;
extern int imuCycle;
extern unsigned int loopExecuteTime;

static char comData[82];
int comDataCount = 0;

//extern Supervisor supervisor;
extern double gp2y0a41[3][4];
extern volatile long count1, count2;

extern bool openDebug;
extern bool mROSConnected;

extern double batteryVoltage;
extern int sampleTime;

byte  info_required = 0;  //0 none 1 ctrlInfo 2 count info
 

#define SERIAL 0
#define BLE 1

// char *scanfDouble(char *buf, double&value, char split);
// char *scanfInt(char *buf, int&value, char split);
//字符串按分割符分割
int split(char *src, char delim, char **res, int resLen );


void readCounter();

extern IMU mIMU;


void checkSerialData()
{

  //read speed setting from serial
  if (Serial.available())
  {
    while (Serial.available() > 0)
    {
      char ch = Serial.read();
      if (ch == ';' || ch == '\r' || ch == '\n') //new command
      {
        processCommand(comData, comDataCount, SERIAL);
        comDataCount = 0;
        continue;  /////之前没有这个，大bug
      }

      comData[comDataCount++] = ch;
      if (comDataCount > 80) //some error
      {
        Serial.print("Err, CMD too long (>80)...");
        // comData[comDataCount] = 0;
        // Serial.println(comData);
        comDataCount = 0;
      }
    }
  }
}

//  Serial.println("\r\n\r\n***** BLE Commands *******************************");
//  Serial.println(" ST stop!");
//  Serial.println(" RS reset robot");
//  Serial.println(" RP[type] require for settings, 1,2,3,4,5");
//  Serial.println(" RT[type] Set robot type 0 normal 1 balance");
//  Serial.println(" SM[val] Set simulate mode 1 true");
//  Serial.println(" GO Start goto goal");
//  Serial.println(" GG[][][] Set goto goal goan x,y,Q");
//  Serial.println(" GD Start drive mode");
//  Serial.println(" GB Start balance mode");
//  Serial.println(" SD[v][w] set drive goal");
//  Serial.println(" BS stop balance drive or goto goal\r\n\r\n");
//
// Serial.println(" ?; print this info");
// Serial.println(" gr[]; controller infos");
// Serial.println(" sb; start balance");
// Serial.println(" st; stop robot");
// Serial.println(" gd; distance of ultraSonic");
// Serial.println(" od; open debug");
// Serial.println(" cd; close debug");
// Serial.println(" ci; read count");

// Serial.println(" mm[pwm]; move motor with pwm");
// Serial.println(" ml[pwm]; move left motor with pwm");
// Serial.println(" mr[pwm]; move right motor with pwm");
// Serial.println(" gg[xd]; start go to goal");

// Serial.println(" sm[0/1]; simulate mode");
// Serial.println(" io[0/1]; ignore obstacle");
// Serial.println(" rs; reset");
// Serial.println(" tl; turn around test left");
// Serial.println(" tr; turn around test right");

//  Serial.println(" bp[]; balance kp");
//  Serial.println(" bi[]; balance ki");
//  Serial.println(" bd[]; balance kd");
//
//  Serial.println(" vp[]; velocity kp");
//  Serial.println(" vi[]; velocity ki");
//  Serial.println(" vd[]; velocity kd");

void processCommand(char *buffer, int bufferLen, int src)
{
  *(buffer + bufferLen) = 0;
  if (bufferLen < 2)
  {
    if (buffer[0] == 'b') //get baundrate
      Serial.println(115200);
    else if (buffer[0] == 'p')
      Serial.println(100);
    // else
    //   Serial.println("Na");
    return;
  }
  char ch0, ch1;
  ch0 = tolower(buffer[0]);
  ch1 = tolower(buffer[1]);

  //运行过程中频繁使用，最先处理
  if (ch0 == 's' && ch1 == 'd') //set drive Goal
  {
    double v, w = 0;

    v = atof(buffer + 2);
    char *buf = strchr(buffer, ',');
    if (buf != NULL)
      w = atof(buf + 1);

    setDriveGoal(v, w);
  }

  else if (ch0 == 'g' && ch1 == 'r')
  {
    Serial.println("\r\n====");
    Serial.print("-BAT:");
    Serial.print(batteryVoltage);
    Serial.print(", uc dis:");
    Serial.println(ultrasonicDistance);

    Serial.print("-exec time:");
    Serial.println(loopExecuteTime );
    Serial.print("-IMU Cycle:");
    Serial.println(imuCycle);

    Serial.println("\r\n===");
    supervisor.getRobotInfo();
    Serial.println("\r\n===");
    driveSupervisor.getRobotInfo();

    SendMessages("_OK_:%s,%s,%s :%d;\n", 
          floatToStr(0, robot.x),
          floatToStr(1, robot.y),
          floatToStr(2, robot.theta),
          loopExecuteTime
          );

  }
  else if (ch0 == 's' && ch1 == 't') //stop
  {
    // Serial.println("Stop!");
    stopRobot();
  }

  else if (ch0 == 'c' && ch1 == 'i') //count info
  {
    printCountInfo();
  }
  else if( ch0 == 'i' && ch1 =='c') //read i2c counter;
  {
    long beg = micros();
    readCounter();
    int dt = micros() - beg;
    printCountInfo();
    delay(20);
    sendCounterInfo(dt);
  }

  else if (ch0 == 'r' && ch1 == 'i') //require for info 0 none 1 ctrlInfo 2 counter
  {
    int val = atoi(buffer + 2);
    info_required = val;
    SendMessages("_info %d;", val);
  }
  else if( ch0 =='i' && ch1 == 'p')
  {
    Position pos = readPosition();

    SendMessages("_XYQ:%s,%s,%s;\n", 
          floatToStr(0, pos.x),
          floatToStr(1, pos.y),
          floatToStr(2, pos.theta)
          );


  }

  // else if (ch0 == 'm' && ch1 == 'l') // move left motor
  // {
  //   int pwm = atoi(buffer + 2);
  //   printCountInfo();
  //   MoveLeftMotor(pwm);
  // }

  // else if (ch0 == 'm' && ch1 == 'r') // move right motor
  // {
  //   int pwm = atoi(buffer + 2);
  //   printCountInfo();
  //   MoveRightMotor(pwm);
  // }

  else if (ch0 == 'm' && ch1 == 'm') // move motor mmpwml,pwmr
  {
    int pwml = atoi(buffer + 2);
    char *buf = strchr((buffer + 2), ',');

    int pwmr = pwml;
    if (buf != NULL)
      pwmr = atoi(buf + 1);

    motorSpeed(pwml, pwmr);
    MoveMotor(0);
  }
  else if (ch0 == 's' && ch1 == 'p') //speed test
  {
    int pwm0, pwm1, step = 0;
    pwm0 = atoi(buffer + 2);
    char *buf = strchr((buffer + 2), ',');
    pwm1 = atoi(buf + 1);
    buf = strchr((buf + 1), ',');
    step = atoi(buf + 1);

    // Serial.print("SP:");
    // Serial.print(pwm0);
    // Serial.print(",");
    // Serial.print(pwm1);
    // Serial.print(",");
    // Serial.println(step);
    log("SP:%d,%d,%d\n", pwm0, pwm1, step);
    if (step == 0)
      step = 10;

    speedTest(pwm0, pwm1, step);
  }

  else if (ch0 == 'r' && ch1 == 's') //RESET
  {
    ResetRobot();
    SendMessages("_OK_:%s,%s,%s :%d;\n", 
          floatToStr(0, robot.x),
          floatToStr(1, robot.y),
          floatToStr(2, robot.theta),
          loopExecuteTime
          );
  }
  else if( ch0 == 'c' && ch1 == 'r') //ros connected
  {
    Serial.println("\nROS Connected OK!");
    mROSConnected = true;
    mIMU.calibrateIMU();
  }

  else if( ch0 == 's' && ch1 == 'i') //get settings info
  {

    SendSettings();

  }
  else if (ch0 == 'p' && ch1 == 'i') //set pid cmd: pi type kp,ki,kd;
  {
    setPID(buffer + 2);
  }
  else if (ch0 == 's' && ch1 == 'r') // set robot params sr min_rpm,max_rpm,R,L,atObs,dfw,usafe,max_w;
  {
    // int min_rpm, max_rpm;
    // float r, l, atobs, dfw, usafe, max_w;

    SETTINGS sett;
    Serial.println("Set Robot:");

    char *ptrs[9];
    int count = split(buffer+2, ',', ptrs, 9);
    if( count < 9 )
    {
      Serial.println("Data format error!");
      return;
    }

    sett.sampleTime = atoi( ptrs[0]);
    Serial.println(sett.sampleTime);
  
    sett.min_rpm = atoi( ptrs[1]);
    Serial.println(sett.min_rpm);

    sett.max_rpm  = atoi(ptrs[2]);
    sett.radius = atof( ptrs[3] );
    sett.length = atof( ptrs[4] );
    sett.atObstacle = atof( ptrs[5] );
    sett.dfw = atof( ptrs[6] );
    sett.unsafe = atof( ptrs[7] );
    // sett.max_w = atof( ptrs[8] ); 
   
    // sscanf(buffer+2, "%i,%i,%f,%f,%f,%f,%f,%f", &sett.min_rpm, &sett.max_rpm, &sett.radius, 
    //           &sett.length, &sett.atObstacle, &sett.dfw, 
    //           &sett.unsafe, &sett.max_w );
    Serial.print(sett.max_rpm);
    Serial.print(',');
    Serial.println(sett.radius, 4);

    Serial.print(sett.length, 4);
    Serial.print(',');
    Serial.print(sett.atObstacle, 4);
    Serial.print(',');
    Serial.print(sett.dfw, 4);
    Serial.print(',');
    Serial.print(sett.unsafe, 4);
    Serial.print(',');
    Serial.println(0, 4);
    
    sett.sType = 0;
    robot.updateSettings(sett);

    sett = robot.getSettings();
    supervisor.updateSettings(sett); 
    driveSupervisor.updateSettings(sett);
  
    sampleTime = sett.sampleTime;
    Serial.print("Sample：");
    Serial.println(sampleTime );

  }

  else if (ch0 == 's' && ch1 == 'm') //simulate mode //simulate mode sm0 sm1 sm2; 0: cancel simulate mode 1:simulate with the obstacle; 2: simulate with obstacle plus motor
  {
    int val = atoi(buffer + 2);
    SetSimulateMode(val);

    //     if (val == 1)
    //         SetSimulateMode(true);
    // else
    //   SetSimulateMode(false);
  }

  else if (ch0 == 't' && ch1 == 'l') //turn around left/ right(-pwm) test tl0/1/2,360; tl dir, angle;
  {
    int dir = atoi(buffer + 2);
    int angle = 360;
    bool useIMU = false;
    char *buf = strchr(buffer, ',');
    double w = 0.8;
    if (buf != NULL)
    {
      angle = atof(buf + 1);
      buf = strchr(buf+1, ',');
      if( buf != NULL )
      {
        w = atof( buf+1);
      }
    }
    turnAround(dir, angle, w);
  }
  // else if (ch0 == 'm' && ch1 == 'g') //go to goal
  // {
  //   manuaGoal();
  // }

  else if( ch0 == 'i' && ch1 == 'm')  //imu 0/1 0/1,0-1; useIMU, withMag, alpha;
  {
    bool val = *(buffer + 2) - '0';
    float alpha = 0.5;
    if (val == true)
      alpha = atof((char *)(buffer + 4));
    setUseIMU(val, alpha);
  }
  else if( ch0 == 'm' && ch1 == 'f') //imu filter select: 0 madgwick 1 madg 2 mentho; mf0/1/2,0/1;
  {
    int iFilter = *(buffer+2)-'0';
    bool withMag = *(buffer+4) - '0';
    // log("sel IM filter:%d.\n", iFilter);
    setIMUFilter( iFilter, withMag );

  }
  else if( ch0 == 'c' && ch1 == 'm')   //do imu calibration
  {
    Serial.println("- Calibrate the IMU....");
    mIMU.calibrateIMU();
  }

  else if (ch0 == 'i' && ch1 == 'f') // set ir filter IF0/1,0.6;
  {
    bool val = *(buffer + 2) - '0';
    float filter = atof((char *)(buffer + 3));
    log("S IR flt:%d,%s\n", val, floatToStr(0, filter));
    robot.setIRFilter(val, filter);
  }
  else if ( ch0 == 'i' && ch1 == 'r')
  {
    short idx = *(buffer + 2) - '0';
    byte val = *(buffer + 3) - '0';
    log("S IR:%d,%d\n", idx, val);

    robot.setHaveIrSensor(idx, val );
    // robot.setHaveIRSensor(idx, val);
  }


  else if( ch0 == 's' && ch1 == 'c')  //save calibration to eeprom
  {
    mIMU.saveCalibrationToEEProm();
  }

  else if (ch0 == 'g' && ch1 == 'o') //start go to goal
  {
    startGoToGoal();
  }

  else if (ch0 == 'g' && ch1 == 'g') //set goto goal goal
  {


    char *ptrs[5];
    int count = split(buffer+2, ',', ptrs, 4);
    if( count < 4 )
    {
      Serial.println("Data format error!");
      return;
    }

    Serial.println( buffer );
    
    double x = atof( ptrs[0] )/10.0;
    double y = atof( ptrs[1] )/10.0;
    double angle = atof( ptrs[2] ); ///1000.0;
    double v = atof( ptrs[3] )/100.0;

    double theta;

    if (angle <= 180)
      theta = (angle * PI) / 180.0;
    else
    {
      angle = angle - 360;
      theta = (angle * PI) / 180.0;
    }
    setGoal(x, y, theta, v);
  }

  else if (ch0 == 'o' && ch1 == 'd') //set obstacle distance
  {
    double ods[5];
   
    char *ptrs[5];
    int count = split(buffer+2, ',', ptrs, 5);
    if( count < 5 )
    {
      Serial.println("Data format error!");
      return;
    }

    for(int i=0; i<5; i++)
      ods[i] = atof( ptrs[i] )/1000.0;
    robot.setObstacleDistance(ods);
  }
  else if (ch0 == 'r' && ch1 == 'p') //set robot position
  {

        char *ptrs[5];
    int count = split(buffer+2, ',', ptrs, 3);
    if( count < 3 )
    {
      Serial.println("Data format error!");
      return;
    }

    double x = atof( ptrs[0] )/1000.0;
    double y = atof( ptrs[1] )/1000.0;
    double theta = atof( ptrs[2] )/1000.0;

    robot.x = x;
    robot.y = y;
    robot.theta = theta;
   
    // supervisor.setRobotPosition(x, y, theta);
  }

  else if (ch0 == 'i' && ch1 == 'o') //ignore atObstacle
  {
    int val = atoi(buffer + 2);
    SetIgnoreObstacle(val);

    // if (val == 1)
    //   SetIgnoreObstacle(true);
    // else
    //   SetIgnoreObstacle(false);
  }
         else if( ch0 == 'p' && ch1 == 'm') //pin pinMode
        {
          char *p = strchr(buffer, ',');
          *p = 0;
          int pin = atoi( buffer+2);
          int pm = atoi( p+1);
          Serial.print("PinMode:");
          Serial.print(pin);
          Serial.print(", ");
          Serial.println( pm );

          pinMode(pin, pm);

        }
        else if( ch0=='d' && ch1 == 'w') //digital write pin
        {
         char *p = strchr(buffer, ',');
          *p = 0;
          int pin = atoi( buffer+2);
          int pm = atoi( p+1);
          Serial.print("dw:");
          Serial.print(pin);
          Serial.print(", ");
          Serial.println( pm );

          digitalWrite(pin, pm);

        }
        else if( ch0=='a' && ch1 == 'w') //anol write pin
        {
         char *p = strchr(buffer, ',');
          *p = 0;
          int pin = atoi( buffer+2);
          int pm = atoi( p+1);
          Serial.print("aw:");
          Serial.print(pin);
          Serial.print(", ");
          Serial.println( pm );

          analogWrite(pin, pm);

        }

        else if( ch0 == 'd' && ch1 == 'r') //digital read
        {
            int pin = atoi(buffer+2);
            Serial.print("dr:");
            Serial.print(pin);
            Serial.print(",:");
            Serial.println(digitalRead(pin));
        }
        else if(ch0 == 'a' && ch1 == 'r') //digital read
        {
            int pin = atoi(buffer+2);
            Serial.print("ar:");
            Serial.print(pin);
            Serial.print(",:");
            Serial.println(analogRead(pin));
        }

        // else if(  buf[0] == 's' && buf[1] == 'b') //set baud rate 
        // {
        //   Serial.print( "Set baud rate to: ");
        //   long baudrate = atol( buf+2);
        //   Serial.println( baudrate );
        //   bluetooth.begin( baudrate );
            
        // } 
  else
  {
    Serial.print("Na,");
    Serial.println(buffer );
  }
}

void printCountInfo()
{
  unsigned long c1, c2;

  c1 = count1;
  c2 = count2;

  Serial.print("-CI:");
  Serial.print(count1);
  Serial.write(',');
  Serial.println(count2);

  bluetooth.print("-CI:");
  bluetooth.print(count1);
  bluetooth.write(',');
  bluetooth.println(count2);

}

void speedTest(int pwm0, int pwm1, int step)
{
  // long c1, c2, lt;
  for (int i = pwm0; i < pwm1; i += step)
  {
    motorSpeed(i, i);
  }
  MoveMotor(0);
}

void motorSpeed(int pwml, int pwmr)
{
  long c1, c2, lt, ct, bt;
  lt = millis();
  MoveLeftMotor(pwml);
  MoveRightMotor(pwmr);

  int cnt = 0;
  SendMessages("-mm %d,%d;\n", pwml, pwmr);

  c1 = 0;
  while( true )
  {
    BLEHM10Loop();
    ct = millis();
    if( ct - lt < 40)
      continue;
    readCounter();
    cnt++;
    if( c1==0 && cnt >= 25 )
    {
      bt = ct;
      c1 = count1;
      c2 = count2;
    }
    sendCounterInfo( ct-lt );
    lt = ct;
    if( cnt >= 50 )
      break;
  }

  int cc1 = count1 - c1;
  int cc2 = count2 - c2;
  SendMessages("%d,%d,%d;\n", (int)(lt - bt), cc1, cc2);
  // log("%d,%d,%d\n", lt, c1, c2);
}


void getDoubleValues(char *buffer, int c, double *fvs)
{
  char *buf = buffer;
  for (int i = 0; i < c; i++)
  {
    fvs[i] = (float)atoi(buf) / 1000.0;
    buf = strchr(buf, ',');
    if (buf == NULL)
      break;
    buf++;
    // Serial.print(fvs[i]);
    // Serial.print(',');
  }
  // Serial.println(';');
}

int formatStr(char *buf, char *format, ...)
{
  int c = 0;
  va_list vArgList;
  va_start(vArgList, format);
  c = vsprintf(buf, format, vArgList); //_vsnprintf(buf, 256, format, vArgList);
  va_end(vArgList);
  // *(buf + c) = 0;
  return c;
}



void log(const char *format, ...)
{
  char tmp[200];
  va_list vArgList;
  va_start(vArgList, format);
  // sniprintf(tmp, 490, format, vArgList);
  vsprintf(tmp, format, vArgList);
  va_end(vArgList);
  Serial.print(tmp);
}



char tmp[20][15];

const char *floatToStr(int idx, double val)
{

  return floatToStr(idx, (unsigned char)6, (unsigned char)3, val);
}

const char *floatToStr(int idx, unsigned char width, unsigned char prec, double val)
{
  if (idx >= 19)
    return NULL;
  return dtostrf(val, width, prec, tmp[idx]);
}


//cmd: pi type kp,ki,kd; type 2,3,4
void setPID(char *buffer)
{
  double p, i, d;
  int type = *buffer - '0';

  p = atof((buffer+1));
  char *buf = strchr(buffer, ',');
  i = atof((buf + 1));
  buf = strchr((buf + 1), ',');
  d = atof(buf + 1);

  log("set pid:%d, %s, %s, %s;\n",
      type,
      floatToStr(0, p),
      floatToStr(1, i),
      floatToStr(2, d));

    robot.setPIDParams(type, p, i, d);
    SETTINGS sett = robot.getSettings();
    supervisor.updateSettings(sett); 
    driveSupervisor.updateSettings(sett);
}


//字符串按分割符分割
int split(char *src, char delim, char **res, int resLen )
{
    int idx = 0;
    char *p = src;
    res[idx++] = src;

    while(true )
    {
        p = strchr(p, delim);
        if( p == NULL )
          break;
        p++;
        res[idx++] = p;
        if( idx >= resLen )
          break;
    }

    return idx;
}
