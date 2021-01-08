
#ifndef _DRIVE_SUPERVISOR_H_
#define _DRIVE_SUPERVISOR_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"
#include "RearDriveRobot.h"
#include "BalanceRobot.h"

void readCounter();

#define S_STOP 0
#define s_DRIVE 1
#define S_TURN 2
#define S_TURN_END 3
#define S_FW 4

class DriveSupervisor
{
public:
  DriveSupervisor();
  Output execute(Robot *robot, double yaw, double dt);

  void reset();
  void reset(double theta);

  void setGoal(double v, double w);
  //turn around dir: 0 原地转圈，1：左轮转，2 右轮转； angle 度数；0-360;
  void turnAround(int dir, int angle, bool useIMU, double theta, double yaw );
  void turnAround(int dir, int turnAngle, double w, double theta, double yaw);

  void getRobotInfo()
  {
    Serial.println("Drive robot info:");

    Serial.print("v:");
    Serial.print(m_output.v);
    Serial.print(",w:");
    Serial.print(m_output.w);
    // m_Controller.PrintInfo();
  }

  void updateSettings(SETTINGS settings);

  bool mIgnoreObstacle;

private:
  void check_states(Robot *robot);
  void doVControll(Robot *robot, double dt);

  double ctrl_v, ctrl_w;
  double target_theta;  //
  boolean keepTheta;
  
 // Vel mVel;
  bool unsafe;
  bool danger;

  int m_state;

  bool inTurnState = false;

  double turnedTheta, targetTurnTheta;
  double lastTheta;
  double targetYaw;
  bool ctrlTurnByIMU;
private:
  double d_unsafe;
  Input m_input;
  Output m_output;

//ctroller 

  double lastError;
  double lastErrorIntegration;
  double kp, ki, kd;

  double lastVError;
  double lastVErrorIntegration;
  double vkp, vki, vkd;



};

#endif /* _DRIVE_SUPERVISOR_H_ */
