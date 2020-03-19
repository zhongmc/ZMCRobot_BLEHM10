#ifndef _ZMC_ROBOT_H_
#define _ZMC_ROBOT_H_

#include <Arduino.h>

#define BALANCE_CAR 1
#define DRIVE_CAR 2


#define CAR_TYPE DRIVE_CAR
 // BALANCE_CAR DRIVE_CAR



//states
#define STATE_IDLE 0
#define STATE_MENU 1
#define STATE_DRIVE 2
#define STATE_GOTOGOAL 3
#define STATE_BALANCE 4
#define STATE_CALIBRATE 5

// TT 单相测速， dual A/B相测速， CHANGE A/B相，以改变中断计数，上升沿、下降沿都计数，精度加倍
#define TT_MOTOR 0
#define DUAL_MOTOR 1
#define CHANGE_MOTOR 2
#define MOTOR 1
 //TT_MOTOR DUAL_MOTOR CHANGE_MOTOR

#define MPU 9250
//3.3V or 5_V VOLT33 VOLT5
#define VOLT5 5
#define VOLT33 3
#define OP_VOLTAGE 5
// 电池电压测量 V = D *(R1+R2)*ref / (1024*R2)  R1= 47 R2=4.7  D*ref*0.0107421875
// R1 = 46.7 R2=4.67  Vd 0.9v  0.0107421875
//3.3v 0.0352771  0.03544921875
// 5V  0.0537109375

  // int sType; // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed;
  // 4：PID for balance theta； 5: settings for robot; 6: settings for balance robot;

#define PKG_PID_DRV 1
#define PKG_PID_BANLANCE 2
#define PKG_PID_SPEED 3
#define PKG_PID_THETA 4
#define PKG_SET_ROBOT 5
#define PKG_SET_BALANCE 6

#define PKG_STATE_BALANCE 9
#define PKG_STATE_DRV 8

#define PKG_MSG 10



#define MAX_ULTRASONIC_DIS 1
#define MAX_IRSENSOR_DIS 0.3
#define SPEED_DOWN_DIS 0.2 //distance to goal to speed down
#define DIS_SPEED_DOWN_SCALE 10
#define W_SPEED_DOWN_SCALE 1

typedef struct
{
  double x, y;
} Vector;

typedef struct
{
  double x, y, theta;
  double v, w;
} Position;

typedef struct
{
  double kp, ki, kd;
} PIDParam;

long readLeftEncoder();
long readRightEncoder();

void MoveLeftMotor(int PWM);
void MoveRightMotor(int PWM);
void StopMotor();

void setGoal(double x, double y, int theta, double v);
void startGoToGoal();
void stopRobot();
void ResetRobot();
void SetSimulateMode(int sm);

void SetIgnoreObstacle(bool igm);

void setDriveGoal(double v, double w);
void startDrive();
void stopAndReset();

void initMotor();
void initBluetooth();
void checkSerialData();
//ble cmd process
void processSetingsRequire();
void sendRobotStateValue(byte stateType, Position pos, double irDistance[5], double voltage);

void sendBalanceRobotStateValue(Position pos, double irDistance[5], double voltage);

void MoveMotor(int pwm);

void checkBLTL();
void turnAround(int pwm, int stopCount);
void setPID(char *buffer);

void startBalance();
void stopBalance();
void balanceUnnormal();
void balanceRecovered();
void irRemoteProcess(int code);

int formatStr(char *buf, char *fmt, ...);

void log(const char *fmt, ...);

const char *floatToStr(int idx, double val);
const char *floatToStr(int idx, char width, unsigned char prec, double val);
Position getRobotPosition();
double getYaw();

void showBatVoltage(float v);
void showBleState(int state ); //0 无设备, 1 有设备，未连接, 2  有设备，已连接;



#endif
