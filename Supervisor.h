
#ifndef _SUPERVISOR_H_
#define _SUPERVISOR_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"
// #include "RearDriveRobot.h"
// #include "BalanceRobot.h"

#include "GoToGoal.h"
#include "AvoidObstacle.h"
#include "FollowWall.h"
#include "SlidingMode.h"

#include "VWDriveController.h"

#define S_STOP 0
#define S_GTG 1
#define S_AVO 2
#define S_FLW 3

class Supervisor
{
public:
  Supervisor();
  
  Output execute(Robot *robot, double yaw, double dt);

  // void executeFollowWall(double dt);
  void executeAvoidAndGotoGoal(Robot *robot, double dt);

  void reset();
  void setGoal(double x, double y, double targetTheta, double v);

  void getRobotInfo()
  {
    Serial.print("GTG Robot:");
    if (m_state == S_STOP)
      Serial.println("Stoped!");
    else if (m_state == S_GTG)
      Serial.println("GTG!");
    else if (m_state == S_AVO)
      Serial.println("AVO!");
    else if (m_state == S_FLW)
    {
      if (m_FollowWall.dir == 0)
        Serial.println("FLW L");
      else
        Serial.println("FLW R");
    }

    Serial.print("i-v:");
    Serial.print(m_input.v);

    Serial.print("; v:");
    Serial.print(m_output.v);
    Serial.print(", w:");
    Serial.print(m_output.w);

    // Serial.print(", vel-l:");
    // Serial.print(robot.vel_l);
    // Serial.print(", vel-r:");
    // Serial.println(robot.vel_r);
    Serial.print("at ob:");
    Serial.print(d_at_obs);
    Serial.print(", ");
    Serial.print("dfw:");
    Serial.println(d_fw);

    Serial.print("exec time:");
    Serial.println(execTime);

    Serial.print("GTG CTRL ");
    m_GoToGoal.PrintInfo();
    Serial.print("FLW CTRL ");
    m_FollowWall.PrintInfo();
    Serial.print("Diff Ctrl:");
    m_DriveCtrl.PrintInfo();
  }

  //the target to go!
  Vector m_Goal;
  Vector mFollowWallPoint;
  void updateSettings(SETTINGS settings);

  bool mIgnoreObstacle;

private:
  void set_progress_point(double x, double y);
  void check_states(Robot *robot);
  bool changeToFollowWall(Robot *robot);

  bool progress_made;
  bool at_goal;
  bool at_obstacle;
  bool noObstacle;
  bool unsafe;
  bool danger;

  // bool mUseIMU;
  // double alpha;


  int m_state;
  double m_right_ticks, m_left_ticks;
  double m_distanceToGoal;

private:
  GoToGoal m_GoToGoal;
  AvoidObstacle m_AvoidObstacle;
  FollowWall m_FollowWall;
  SlidingMode m_SlidingMode;

  VWDriveController m_DriveCtrl;
  // RearDriveRobot robot;
  // BalanceRobot robot;
  
  Controller *m_currentController;

  double d_fw; //distance to follow wall
  double d_at_obs;
  double d_unsafe;
  double d_stop;
  
  double d_prog;
  long execTime;

  Input m_input;
  Output m_output;
//  Vel mVel;
};

#endif /* _SUPERVISOR_H_ */
