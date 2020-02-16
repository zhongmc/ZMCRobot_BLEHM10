
#ifndef _SUPERVISOR_H_
#define _SUPERVISOR_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"
#include "RearDriveRobot.h"

#include "GoToGoal.h"
#include "AvoidObstacle.h"
#include "FollowWall.h"
#include "SlidingMode.h"

#include "DifferencialController.h"

#define S_STOP 0
#define S_GTG 1
#define S_AVO 2
#define S_FLW 3

class Supervisor
{
public:
  Supervisor();
  void execute(long left_ticks, long right_ticks, double gyro, double dt);

  // void executeFollowWall(double dt);
  void executeAvoidAndGotoGoal(double dt);

  void reset(long leftTicks, long rightTicks);
  void resetRobot();
  void setGoal(double x, double y, int theta, double v);

  void setHaveIRSensor(int idx, byte val);

  void setIRFilter(bool open, float filter);

  void setSimulateMode(int val);

  void setUseIMU(bool beUseIMU, double _alpha)
  {
    mUseIMU = beUseIMU;
    alpha = _alpha;
    robot.setUseIMU(beUseIMU, _alpha);
  };

  bool isUseIMU()
  {
    return mUseIMU;
  }



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

    Serial.print(", vel-l:");
    Serial.print(robot.vel_l);
    Serial.print(", vel-r:");
    Serial.println(robot.vel_r);
    // long c1, c2;
    // c1 = (long)m_left_ticks;
    // c2 = (long)m_right_ticks;
    Serial.print("c1:");
    Serial.print(m_left_ticks);
    Serial.print(", c2:");
    Serial.println(m_right_ticks);

    Serial.print("at ob:");
    Serial.print(d_at_obs);
    Serial.print(", ");
    Serial.print("dfw:");
    Serial.println(d_fw);

    Serial.print("exec time:");
    Serial.println(execTime);
    robot.getRobotInfo();
    Serial.print("GTG CTRL ");
    m_GoToGoal.PrintInfo();
    Serial.print("FLW CTRL ");
    m_FollowWall.PrintInfo();
    Serial.print("Diff Ctrl:");
    m_DiffCtrl.PrintInfo();
  }

  //the target to go!
  Vector m_Goal;
  Vector mFollowWallPoint;

  void getIRDistances(double dis[5]);
  void readIRDistances(double dis[5]);

  void setObstacleDistance(double dis[5]);

  Position getRobotPosition();
  void setRobotPosition(double x, double y, double theta);
  void init();

  void updateSettings(SETTINGS settings);
  void setPIDParams(int type, double kp, double ki, double kd );
  SETTINGS getSettings();

  unsigned int getLeftTicks(){return m_left_ticks;};
  unsigned int getRightTicks(){ return m_right_ticks;};

  bool mSimulateMode;
  bool mIgnoreObstacle;

private:
  void
  set_progress_point();
  void check_states();
  bool changeToFollowWall();

  bool progress_made;
  bool at_goal;
  bool at_obstacle;
  bool noObstacle;
  bool unsafe;
  bool danger;

  bool mUseIMU;
  double alpha;


  int m_state;
  double m_right_ticks, m_left_ticks;
  double m_distanceToGoal;

private:
  GoToGoal m_GoToGoal;
  AvoidObstacle m_AvoidObstacle;
  FollowWall m_FollowWall;
  SlidingMode m_SlidingMode;

  DifferencialController m_DiffCtrl;

  // Robot robot;

  RearDriveRobot robot;

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
