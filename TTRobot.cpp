#include "TTRobot.h"

TTRobot::TTRobot()
{
  //R, L, ticksr, minRpm, maxRpm, GP2Y0A41);
  init(0.065 / 2, 0.125, 20, 20, 40, 200, GP2Y0A41);

  mPIDSettings.kp = 5;
  mPIDSettings.ki = 0.01;
  mPIDSettings.kd = 0.05;
  //  mSettings.kp = 5;
  //   mSettings.ki = 0.01;
  //   mSettings.kd = 0.05;
}

Vel TTRobot::ensure_w(double v, double w)
{

  Vel vel = uni_to_diff(v, w);
  double vel_l, vel_r;

  double vel_min, vel_max;
  vel_min = vel.vel_l;
  vel_max = vel.vel_r;

  if (vel_min > vel.vel_r)
  {
    vel_min = vel.vel_r;
    vel_max = vel.vel_l;
  }

  // stop one motor to support large angle turning
  double minVel = 0;
  if (abs(w) < 0.2)
    minVel = min_vel;

  if (vel_max > max_vel)
  {
    vel_r = vel.vel_r - (vel_max - max_vel);
    vel_l = vel.vel_l - (vel_max - max_vel);
  }
  else if (vel_min < minVel)
  {
    vel_r = vel.vel_r + (minVel - vel_min);
    vel_l = vel.vel_l + (minVel - vel_min);
  }
  else
  {
    vel_r = vel.vel_r;
    vel_l = vel.vel_l;
  }

  if (vel_l < minVel)
    vel_l = minVel;
  else if (vel_l > max_vel)
    vel_l = max_vel;

  if (vel_r < minVel)
    vel_r = minVel;
  else if (vel_r > max_vel)
    vel_r = max_vel;

  vel.vel_l = vel_l;
  vel.vel_r = vel_r;
  return vel;
}

double TTRobot::vel_l_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);
  if (nvel < min_vel)
    nvel = min_vel;
  else if (nvel > max_vel)
    nvel = max_vel;

  double retVal = 9.6893 * nvel + 10.179; //6.393 * nvel + 13.952;

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double TTRobot::vel_r_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);

  if (nvel < min_vel)
    nvel = min_vel;
  else if (nvel > max_vel)
    nvel = max_vel;

  double retVal = 9.5946 * nvel + 18.738; // 6.2798 * nvel + 18.787;

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double TTRobot::pwm_to_ticks_r(double pwm, double dt)
{

  double npwm = abs(pwm);
  if (npwm < 20)
    return 0;

  double ticks = dt * (0.5084 * npwm - 9.7666);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}

double TTRobot::pwm_to_ticks_l(double pwm, double dt)
{
  double npwm = abs(pwm);
  if (npwm < 14) //14
    return 0;

  double ticks = dt * (0.4975 * npwm - 6.9066);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}
