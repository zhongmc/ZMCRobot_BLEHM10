#include "ZMCRobot.h"
#include "robot.h"

#include "BleFuncs.h"


#include "Supervisor.h"
#include "DriveSupervisor.h"


extern Supervisor supervisor;
extern DriveSupervisor driveSupervisor;


extern bool doCheckBattleVoltage; // = true;
extern bool openDebug;            // = false;
extern byte settingsReqQueue[8];
extern short queueLen; // = 0;

void processBleCommandPackage(byte *data) 
{
  // central wrote new value to characteristic, update LED
  Serial.print("Ble cmd:");
  //the first two chas as CMD
  char cmd[3];
  cmd[2] = '\0';
  cmd[0] = toupper(data[0]);
  cmd[1] = toupper(data[1]);
  Serial.println(cmd);

  if (cmd[0] == 'S' && cmd[1] == 'T') //stop
  {
    stopRobot();
  }
  else if (cmd[0] == 'R' && cmd[1] == 'P') //Required for settings
  {
    Serial.println("Req sof:");
    Serial.println(data[2]);

    if (queueLen < 7)
      settingsReqQueue[queueLen++] = data[2];
    else
      Serial.println("Q Ovf");

    //    requireForSettings = true;
    //    requiredSettingsType = data[2];
  }
  // else if (cmd[0] == 'R' && cmd[1] == 'T') //robot type 0 3wheel car ; 1 balance robot
  // {
  //   Serial.print("Set robot type;");
  //   Serial.println(data[2]);
  // }
  // else if (cmd[0] == 'C' && cmd[1] == 'B') // check battle voltage
  // {
  //   if (data[2] == 1)
  //     doCheckBattleVoltage = true;
  //   else
  //     doCheckBattleVoltage = false;
  // }

  else if (cmd[0] == 'S' && cmd[1] == 'M') //simulate mode
  {
    if (data[2] == 1)
      SetSimulateMode(true);
    else
      SetSimulateMode(false);
  }
  else if (cmd[0] == 'I' && cmd[1] == 'O') //ignore atObstacle
  {
    if (data[2] == 1)
      SetIgnoreObstacle(true);
    else
      SetIgnoreObstacle(false);
  }

  else if (cmd[0] == 'R' && cmd[1] == 'S') //RESET
  {
    ResetRobot();
  }

  else if (cmd[0] == 'G' && cmd[1] == 'O') // action...
  {
    startGoToGoal();
  }

  // else if (cmd[0] == 'G' && cmd[1] == 'D') //start drive mode
  // {
  //   startDrive();
  // }

  else if (cmd[0] == 'G' && cmd[1] == 'G') // Go To Goal: x, y, theta
  {
    double x, y, v;
    int theta;
    x = byteToFloat((byte *)(data + 2), 100);
    y = byteToFloat((byte *)(data + 4), 100);
    theta = byteToInt((byte *)(data + 6));
    v = byteToFloat((byte *)(data + 8), 100);

    setGoal(x, y, theta, v);

    log("GTG:%s,%s,%d,%s\n",
        floatToStr(0, x),
        floatToStr(1, y),
        theta,
        floatToStr(2, v));

    // Serial.print("GTG:");
    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.print(",");
    // Serial.println(theta);
  }

  // else if (cmd[0] == 'M' && cmd[1] == 'G') //go to goal
  // {
  //   float d = atof((char *)(data + 2));
  //   Serial.print("m gtg:");
  //   Serial.println(d);

  //   count1 = 0;
  //   count2 = 0;
  //   supervisor.reset(0, 0);

  //   setGoal(d, 0, 0, 0.12);
  //   startGoToGoal();
  // }

  // else if (cmd[0] == 'S' && cmd[1] == 'R') //step response
  // {
  //   startStepResponse(90);
  // }

  
  else if (cmd[0] == 'I' && cmd[1] == 'M') //use IMU or not IM0/1,0.5;
  {
    bool val = *(data + 2) - '0';
    float alpha = 0.5;
    if (val == true)
      alpha = atof((char *)(data + 4));
    log("use IMU:%d,%s\n", val, floatToStr(0, alpha));
    driveSupervisor.mUseIMU = val;
    driveSupervisor.alpha = alpha;
    //driveSupervisor.setUseIMU(val, alpha);
  }

  else if (cmd[0] == 'I' && cmd[1] == 'F') // set ir filter IF0/1,0.6;
  {
    bool val = *(data + 2) - '0';
    float filter = atof((char *)(data + 3));
    log("IR flt:%d,%s\n", val, floatToStr(0, filter));
    supervisor.setIRFilter(val, filter);
    driveSupervisor.setIRFilter(val, filter);
  }
  else if (cmd[0] == 'I' && cmd[1] == 'R')
  {
    short idx = *(data + 2) - '0';
    byte val = *(data + 3) - '0';
    log("S IR:%d,%d\n", idx, val);

    supervisor.setHaveIRSensor(idx, val);
    driveSupervisor.setHaveIRSensor(idx, val);
  }

  else if (cmd[0] == 'P' && cmd[1] == 'I') //pid
  {
    setPID((char *)(data + 2));
  }


  else if (cmd[0] == 'S' && cmd[1] == 'D') //set drive Goal
  {
    double v, w;
    v = byteToFloat((byte *)(data + 2), 100);
    w = byteToFloat((byte *)(data + 4), 100);
    log("v=%s,w=%s\n",
        floatToStr(0, v),
        floatToStr(1, w));
    setDriveGoal(v, w);
  }
}

void setConfigValue(byte *cfgArray)
{
  int settingsType = (int)cfgArray[0];
  Serial.print("cfg v BT:");
  Serial.println(settingsType);

  SETTINGS settings;
  settings.sType = settingsType;

  if (settingsType == 1 || settingsType == 2 || settingsType == 3 || settingsType == 4)
  {
    settings.kp = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.ki = byteToFloat((byte *)(cfgArray + 3), 1000);
    settings.kd = byteToFloat((byte *)(cfgArray + 5), 1000);

    log("KP:%s, KI:%s, KD:%s\n",
        floatToStr(0, settings.kp),
        floatToStr(1, settings.ki), floatToStr(2, settings.kd));
    // Serial.print("KP:");
    // Serial.print(settings.kp);
    // Serial.print(" KI:");
    // Serial.print(settings.ki);
    // Serial.print(" KD:");
    // Serial.println(settings.kd);
  }

  else if (settingsType == 5)
  {

    settings.atObstacle = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.unsafe = byteToFloat((byte *)(cfgArray + 3), 100);
    settings.dfw = byteToFloat((byte *)(cfgArray + 5), 100);
    settings.max_w = byteToFloat((byte *)(cfgArray + 7), 100);

    settings.max_rpm = byteToInt((byte *)(cfgArray + 9));
    settings.min_rpm = byteToInt((byte *)(cfgArray + 11));

    settings.radius = byteToFloat((byte *)(cfgArray + 13), 1000);
    settings.length = byteToFloat((byte *)(cfgArray + 15), 1000);

    // settings.pwm_diff = (int)cfgArray[13]; //byteToInt((byte *)(cfgArray + 13) );
    // settings.pwm_zero = (int)cfgArray[14];
    // settings.angleOff = byteToFloat((byte *)(cfgArray + 15), 100);

    log("atObs:%s, unsafe:%s, dfw:%s, max_w:%s, max_rmp:%d, min_rpm:%d, R:%s, L:%s\n",
        floatToStr(0, settings.atObstacle),
        floatToStr(1, settings.unsafe),
        floatToStr(2, settings.dfw),
        floatToStr(3, settings.max_w),
        settings.max_rpm,
        settings.min_rpm,
        floatToStr(4, settings.radius),
        floatToStr(5, settings.length)

    );
    // Serial.print(" atObstacle:");
    // Serial.print(settings.atObstacle);

    // Serial.print(" unsafe:");
    // Serial.print(settings.unsafe);
    // Serial.print(" dfw:");
    // Serial.print(settings.dfw);

    // Serial.print(" v:");
    // Serial.print(settings.velocity);
    // Serial.print(" max_rpm:");
    // Serial.print(settings.max_rpm);
    // Serial.print(" min_rpm:");
    // Serial.print(settings.min_rpm);
    // Serial.print(" radius:");
    // Serial.print(settings.radius);
    // Serial.print(" length:");
    // Serial.println(settings.length);
  }
 else if (settingsType == 6)
  {

    settings.atObstacle = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.unsafe = byteToFloat((byte *)(cfgArray + 3), 100);
    settings.max_pwm = byteToInt((byte *)(cfgArray + 5));
    settings.pwm_zero = (int)cfgArray[7];
    settings.pwm_diff = (int)cfgArray[8];
    // settings.angleOff = byteToFloat((byte *)(cfgArray + 9), 100);

    settings.radius = byteToFloat((byte *)(cfgArray + 11), 1000);
    settings.length = byteToFloat((byte *)(cfgArray + 13), 1000);

    settings.max_w = byteToFloat((byte *)(cfgArray + 15), 100);

    Serial.print(" atObstacle:");
    Serial.print(settings.atObstacle);

    Serial.print(" unsafe:");
    Serial.print(settings.unsafe);
    Serial.print(" max_pwm:");
    Serial.print(settings.max_pwm);
    Serial.print(" pwm_zero:");
    Serial.print(settings.pwm_zero);
    Serial.print(" pwm_diff:");
    Serial.print(settings.pwm_diff);
    Serial.print(" angle_off:");
    // Serial.print(settings.angleOff);
    Serial.print(" radius:");
    Serial.println(settings.radius);
    Serial.print(" wheel distance:");
    Serial.println(settings.length);
  }


  if (settingsType == 1 || settingsType == 5 || settingsType == 2 || settingsType == 3 || settingsType == 4)
  {
    supervisor.updateSettings(settings);
    driveSupervisor.updateSettings(settings);
  }

  // if (settingsType == 2 || settingsType == 3 || settingsType == 4 || settingsType == 6)
  // {
  //   balanceSupervisor.updateSettings(settings);
  // }
  // setSettings(settings);
  //  updateConfigToMenu();
}

void intToByte(byte *arrayBuf, int val)
{
  if (val > 0)
  {
    *arrayBuf = val & 0xff;
    val = (val & 0xff00) / 256;
    *(arrayBuf + 1) = val;
  }
  else
  {
    val = -val;
    *arrayBuf = val & 0xff;
    val = (val & 0xff00) / 256;
    *(arrayBuf + 1) = val | 0x80;
  }
}

void floatToByte(byte *arrayBuf, double val, double scale)
{
  int tmp = (int)(val * scale);
  if (tmp > 0)
  {
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp;
  }
  else
  {
    tmp = -tmp;
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp | 0x80;
  }
}

double byteToFloat(byte *arrayBuf, double scale)
{
  int val = *(arrayBuf + 1) & 0x7f;
  val = val * 256 + *arrayBuf;
  if ((*(arrayBuf + 1) & 0x80) != 0)
    val = -val;
  return (double)val / scale;
}

int byteToInt(byte *arrayBuf)
{
  int val = *(arrayBuf + 1) & 0x7f;
  val = val * 256 + *arrayBuf;
  if ((*(arrayBuf + 1) & 0x80) != 0)
    val = -val;

  return val;
}

//type, x, y, theta, d0,d1,d2,d3,d4,voltage
void sendRobotStateValue(byte stateType, Position pos, double irDistance[5], double voltage)
{
 
  byte buf[19];
  memset(buf, 0, 19);

  buf[0] = stateType;

  double scale = 1000;

  floatToByte(buf + 1, pos.x, scale);
  floatToByte(buf + 3, pos.y, scale);
  floatToByte(buf + 5, pos.theta, scale);

  //    Serial.print( voltage);
  //    Serial.print( ", " );

  scale = 100;
  for (int i = 0; i < 5; i++)
  {
    floatToByte(buf + 7 + 2 * i, irDistance[i], scale);
    //  Serial.print( irDistance[i] );
    //    Serial.print( "," );
  }
  //    Serial.println( ";" );

  floatToByte(buf + 17, voltage, scale);
  sendStatePkgToBle( buf, 19 );

//   bool ret = zmcRobotStateChar.setValue(buf, 19);
//   if (!ret)
//   {
//     Serial.println("wt bt F!");
//   }
}

void processSetingsRequire()
{
  if (queueLen == 0)
    return;
  byte sType = settingsReqQueue[0];
  for (int i = 0; i < queueLen - 1; i++)
    settingsReqQueue[i] = settingsReqQueue[i + 1];
  queueLen--;

  SETTINGS settings;
  settings.sType = sType;
  // // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed; 4: PID theta  5: settings for robot; 6: settings for balance robot;
  settings = supervisor.getSettings(sType);
  SendSettings(settings);
}

void SendSettings(SETTINGS settings)
{
  int settingsType = settings.sType;

  Serial.print("Send settings:");
  Serial.println(settingsType);

  byte settingsArray[18];
  settingsArray[0] = (byte)settingsType;
  int len = 7;
  if (settingsType == 1 || settingsType == 2 || settingsType == 3 || settingsType == 4)
  {

    floatToByte(settingsArray + 1, settings.kp, 100);
    floatToByte(settingsArray + 3, settings.ki, 1000);
    floatToByte(settingsArray + 5, settings.kd, 1000);
  }

  else if (settingsType == 5)
  {
    floatToByte(settingsArray + 1, settings.atObstacle, 100);
    floatToByte(settingsArray + 3, settings.unsafe, 100);
    floatToByte(settingsArray + 5, settings.dfw, 100);
    floatToByte(settingsArray + 7, settings.max_w, 100);
    intToByte(settingsArray + 9, settings.max_rpm);
    intToByte(settingsArray + 11, settings.min_rpm);
    floatToByte(settingsArray + 13, settings.radius, 1000);
    // settingsArray[13] = (byte)settings.pwm_diff;
    // settingsArray[14] = (byte)settings.pwm_zero;
    floatToByte(settingsArray + 15, settings.length, 1000);
    len = 17;
  }
/*  else if (settingsType == 6)
  {
    floatToByte(settingsArray + 1, settings.atObstacle, 100);
    floatToByte(settingsArray + 3, settings.unsafe, 100);
    intToByte(settingsArray + 5, settings.max_pwm);
    settingsArray[7] = (byte)settings.pwm_zero;
    settingsArray[8] = (byte)settings.pwm_diff;

    // speedLoop, thetaLoop
    settingsArray[9] = (byte)balanceSupervisor.mSpeedLoop;
    settingsArray[10] = (byte)balanceSupervisor.mThetaLoop;

    //    floatToByte(settingsArray + 9, 0, 1000); //settings.angleOff, 1000);

    floatToByte(settingsArray + 11, settings.radius, 1000);
    floatToByte(settingsArray + 13, settings.length, 1000);

    floatToByte(settingsArray + 15, settings.velocity, 100);

    settingsArray[17] = (byte)balanceSupervisor.mSimulateMode;
    len = 18;
  }
  */
 
    sendSettingsPkgToBle(settingsArray, len);
}
