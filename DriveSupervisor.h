
#ifndef _DRIVE_SUPERVISOR_H_
#define _DRIVE_SUPERVISOR_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"
#include "RearDriveRobot.h"

// #include "VelocityController.h"

#include "VWDriveController.h"
#include "DifferencialController.h"

#define S_STOP 0
#define s_DRIVE 1
#define S_TURN 2
#define S_TURN_END 3
#define S_FW 4

class DriveSupervisor
{
public:
  DriveSupervisor();
  void execute(long left_ticks, long right_ticks, double yaw, double dt);

  void update(long left_ticks, long right_ticks, double dt);
  void reset(long leftTicks, long rightTicks);
  void resetRobot();
  void setGoal(double v, double w);
  //turn around dir: 0 原地转圈，1：左轮转，2 右轮转； angle 度数；0-360;
  void turnAround(int dir, int angle );

  void getRobotInfo()
  {
    Serial.println("Drive robot info:");

    Serial.print("v:");
    Serial.print(m_output.v);
    Serial.print(",w:");
    Serial.print(m_output.w);

    Serial.print(",vel-l:");
    Serial.print(robot.vel_l);
    Serial.print(",vel-r:");
    Serial.println(robot.vel_r);
    // long c1, c2;
    // c1 = (long)m_left_ticks;
    // c2 = (long)m_right_ticks;
    Serial.print("c1:");
    Serial.print(m_left_ticks);
    Serial.print(",c2:");
    Serial.println(m_right_ticks);
    // Serial.print("use imu:");
    // Serial.print(mUseIMU );
    // Serial.print(", alp=");
    // Serial.println( alpha );

    robot.getRobotInfo();
    m_Controller.PrintInfo();
  }

  //val: 0 none, 1,yes, 2 use ultro sonic
  void setHaveIRSensor(int idx, byte val);

  void setIRFilter(bool open, float val);

  void getIRDistances(double dis[5]);
  void getRobotVel(double dis[5]);

  Position getRobotPosition();
  void setRobotPosition(double x, double y, double theta);

  SETTINGS getSettings();
  void updateSettings(SETTINGS settings);
  void setPIDParams(int type, double kp, double ki, double kd );

  unsigned int getLeftTicks(){return m_left_ticks;};
  unsigned int getRightTicks(){ return m_right_ticks;};

  void init();

  void setUseIMU(bool beUseIMU, double _alpha)
  {
    // mUseIMU = beUseIMU;
    // alpha = _alpha;
    robot.setUseIMU(beUseIMU, _alpha);    
  };

  bool mSimulateMode;
  bool mIgnoreObstacle;
  // bool mUseIMU;
  // double alpha;

private:
  void check_states();

  double v, w;
 // Vel mVel;
  bool unsafe;
  bool danger;

  int m_state;
  double m_right_ticks, m_left_ticks;

  int turnDir, turnAngle;
  double turnedTheta, startTheta;
  bool inTurnState = false;

private:
  
  VWDriveController m_Controller;
  DifferencialController m_DifController;
  //   Robot robot;
  RearDriveRobot robot;

  double d_unsafe;
  Input m_input;
  Output m_output;
};

#endif /* _DRIVE_SUPERVISOR_H_ */
