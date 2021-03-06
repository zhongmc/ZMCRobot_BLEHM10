#include "ZMCRobot.h"
#include "robot.h"

#include "commFunctions.h"


#include "Supervisor.h"
#include "DriveSupervisor.h"


extern Supervisor supervisor;
extern DriveSupervisor driveSupervisor;

extern RearDriveRobot robot;

extern byte currentState;

extern volatile long count1, count2;

extern bool doCheckBattleVoltage; // = true;
extern bool openDebug;            // = false;
extern bool mUseIMU;
extern double batteryVoltage;

extern byte currentState;


void SendRobotStates(Position pos, double irDistance[5], double voltage)
{
    SendMessages("RP%d,%d,%d,%d,%d,%d;",
       (int)(1000 * pos.x),
       (int)(1000 * pos.y),
       (int)(1000 * pos.theta),
        (int)(1000 * pos.w),
        (int)(1000 * pos.v),
        (int)(10*voltage));

    // delay(5);

    SendMessages("IR%d,%d,%d,%d,%d;",
        (int)(100 * irDistance[0]),
        (int)(100 * irDistance[1]),
        (int)(100 * irDistance[2]),
        (int)(100 * irDistance[3]),
        (int)(100 * irDistance[4]));


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

byte bleStCnt = 0;
byte pkgIdx = 0;
unsigned long lastBLEStateSend; 

void sendRobotStateWithCounter(Robot *robot, double ultraDist, double voltage, int idt)
{
 
  byte buf[24];
  memset(buf, 0, 24);

  buf[0] = 0xA1;  //A为二进制报文标志，1 为报文类型
  buf[1] = 18;   //长度
  
  buf[18] = pkgIdx;
  buf[19] = (byte)idt;
  
  double scale = 1000;
  floatToByte(buf + 2, robot->x, scale);
  floatToByte(buf + 4, robot->y, scale);
  floatToByte(buf + 6, robot->theta, scale);
  buf[8] = (int)(100.0 * ultraDist);
  buf[9] = (byte)(int)(10.0 * voltage);

  longToBytes(count1, buf + 10);
  longToBytes(count2, buf + 14);

  int bleInterval = millis() - lastBLEStateSend;
  bleStCnt++;

  if( bleInterval >= 50 || bleStCnt >= 3)
  {
    pkgIdx++;
    bleStCnt = 0;
    lastBLEStateSend = millis();
    sendBleMessages( buf, 20);
  }

  //2021-01-25 add v, w to serial msg
  buf[1] = 22;   //长度
  floatToByte(buf + 20, robot->velocity, scale);
  floatToByte(buf + 22, robot->w, scale);

  for (int i = 0; i < 24; i++)
  {
     Serial.write(*(buf + i));
  }
  Serial.flush();
}

// x, y, theta, v, d0,d1,d2,d3,d4,voltage
void sendRobotStateValue( Position pos, double irDistance[5], double voltage)
{
 
  byte buf[20];
  memset(buf, 0, 20);

  buf[0] = 0xA0;  //A为二进制报文标志，0 为报文类型
  buf[1] = 13;   //长度
 
  double scale = 1000;
  floatToByte(buf + 2, pos.x, scale);
  floatToByte(buf + 4, pos.y, scale);
  floatToByte(buf + 6, pos.theta, scale);
  //floatToByte(buf + 8, pos.w, scale);
  buf[8] = (int)(100.0 * pos.v);
  buf[9] = (byte)(int)(10.0 * voltage);

  for (int i = 0; i < 5; i++)
  {
    buf[10+i] = (byte)(int)(100.0*irDistance[i]);
  }

  for (int i = 0; i < 15; i++)
  {
     Serial.write(*(buf + i));
  }
  Serial.flush();
  bleStCnt++;
  int bleInterval = millis() - lastBLEStateSend;

  if( bleInterval >= 50 || bleStCnt >= 3)
  {
    bleStCnt = 0;
    lastBLEStateSend = millis();
    sendBleMessages( buf, 15);
  }


}


void SendSettingsValue()
{

}


void SendSettings()
{
  Serial.println("Req for settings!");


    SETTINGS sett = robot.getSettings( );

    SendMessages("ROP%d,%d,%d,%s,%s,%s,%s,%s,%s\n", 
          sett.sampleTime, sett.min_rpm, sett.max_rpm, 
          floatToStr(0, (unsigned char)2, (unsigned char)5, sett.radius),
          floatToStr(1, (unsigned char)2, (unsigned char)5, sett.length),
          floatToStr(2, sett.atObstacle),
          floatToStr(3, sett.dfw ),
          floatToStr(4, sett.unsafe ),
          floatToStr(5, 0.0 )  //max_w
          );

    delay(10);
    SendMessages("PID1,%s,%s,%s\n", 
      floatToStr(0, sett.kp),
      floatToStr(1, sett.ki),
      floatToStr(2, (unsigned char)2, (unsigned char)5, sett.kd));

    // SendMessages("PID2,%s,%s,%s\n", 
    //   floatToStr(0, sett.pkp),
    //   floatToStr(1, sett.pki),
    //   floatToStr(2, (signed char)2, (unsigned char)5, sett.pkd));

    delay(10);

    // SendMessages("PID3,%s,%s,%s\n", floatToStr(0, sett.tkp),
    //   floatToStr(1, sett.tki),
    //   floatToStr(2, sett.tkd));

    SendMessages("PID4,%s,%s,%s\n", 
    floatToStr(0, sett.dkp),
    floatToStr(1, sett.dki),
    floatToStr(2, (unsigned char)2, (unsigned char)5, sett.dkd));

}


void SendMessages(const char *format, ...)
{
  char tmp[200];
  memset(tmp, 0, 200);
  
  va_list vArgList;
  va_start(vArgList, format);
  // vsniprintf(tmp, 20, format, vArgList);
  vsnprintf(tmp, 199, format, vArgList);
  va_end(vArgList);
  int len = strlen(tmp);
  sendBleMessages( tmp, len);

  for (int i = 0; i < len; i++)
  {
    Serial.write((byte) *(tmp + i));
  }
  Serial.flush();

}




//   else if( cmd[0] == 'R' && cmd[1] == 'I') //robot info
//   {
//     Position pos = getRobotPosition();
//     logToBle("x:%d,y:%d,Q:%d;\n", (int)(100*pos.x), (int)(100 *pos.y), (int)(100*pos.theta) );
//     double yaw = getYaw();
//     int cs = currentState;
//     logToBle("s:%d,Y:%d,B:%d;\n", cs, (int)(10*yaw), (int)(10*batteryVoltage));
//     SETTINGS settings = supervisor.getSettings();

//     int i1,i2;
//     i1 = settings.useIMU;
//     i2 = settings.irFilter;
//     logToBle("im:%d,%d,IF:%d%,%d;\n", i1, (int)(10*settings.imuAlpha), i2, (int)(10*settings.irAlpha));


//   }
