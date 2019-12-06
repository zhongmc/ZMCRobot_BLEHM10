#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <Arduino.h>
#include "IRSensor.h"

typedef struct
{
  double v, theta; //velocity and target direction
  double x_g, y_g; //target x,y
  double targetAngle;
  double turning;
} Input;

typedef struct
{
  double v;
  double w;
} Output;

typedef struct
{
  double vel_r;
  double vel_l;
} Vel;

typedef struct
{
  int pwm_l, pwm_r;
} PWM_OUT;

typedef struct
{
  int sType; // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed; 4：PID for balance theta； 5: settings for robot; 6: settings for balance robot;
  double kp, ki, kd;
  double atObstacle, unsafe;
  double dfw;
  double velocity;
  int max_rpm, min_rpm;
  //轮子半径，轮距
  double radius, length;
  double max_w;
  
//平衡车参数
  int pwm_diff;
  int max_pwm, pwm_zero;
  // double angleOff, wheelSyncKp;
} SETTINGS;

class Robot
{
public:
  Robot();
  // Robot(double R, double L, double ticksr_l, double ticksr_r, double maxRpm, double minRpm);

  double x, y, theta, w;
  double velocity;

  virtual PWM_OUT getPWMOut(double v, double w) = 0;

  virtual Vel ensure_w(double v, double w) = 0;
  virtual double vel_l_to_pwm(double vel) = 0;
  virtual double vel_r_to_pwm(double vel) = 0;

  virtual double pwm_to_ticks_r(double pwm, double dt) = 0;
  virtual double pwm_to_ticks_l(double pwm, double dt) = 0;

  void setObstacleDistance(double dis[5]);

  void setHaveIrSensor(int idx, byte value)
  {
    if (idx >= 0 && idx < 5)
    {
      haveIrSensor[idx] = value;
      irSensors[idx]->setDistance(irSensors[idx]->getMaxDistance());
    }
  }

  void setIRFilter(bool open, float filter)
  {
    for (int i = 0; i < 5; i++)
      irSensors[i]->setIRFilter(open, filter);
  }

  byte haveIrSensor[5];

  SETTINGS getPIDParams()
  {
    return mPIDSettings;
  }

  //     double vel_l, vel_r;

  double getObstacleDistance();
  void updateState(long left_ticks, long right_ticks, double dt);
  void updateState(long left_ticks, long right_ticks, double gyro, double alpha, double dt);

  void reset(long left_ticks, long right_ticks);
  Vel uni_to_diff(double v, double w);
  Output diff_to_uni(double vel_l, double vel_r);
  IRSensor **getIRSensors();

  void setIRSensorType(SENSOR_TYPE sensorType);

  void updateSettings(SETTINGS settings);
  void updatePID(SETTINGS pids);

  void getRobotInfo();

  void readIRSensors(double dt);

  double wheel_radius;
  double wheel_base_length;


  int pwm_diff;

  // double angleOff;

  double max_vel, min_vel;

  double max_rpm;
  double min_rpm;
  
  double max_w;  //the rotate /turn arround limit 

  // double max_v, min_v;
  // double max_w, min_w;

//balance 
  double angle;
  double gyro;

  double vel_l, vel_r;


  int ticks_per_rev_l, ticks_per_rev_r;

protected:
  void init(double R, double L, double ticksr_l, double ticksr_r, double minRpm, double maxRpm, SENSOR_TYPE sensorType);

  SETTINGS mPIDSettings;

  IRSensor *irSensors[5];

private:
  double m_per_tick_l, m_per_tick_r;
  long prev_left_ticks, prev_right_ticks;
};

#endif /* _ROBOT_H_ */
