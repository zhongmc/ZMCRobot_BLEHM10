#include "TTRobot.h"

TTRobot::TTRobot()
{
  //R, L, ticksr, minRpm, maxRpm, GP2Y0A41);
  init( 0.0325, 0.125, 20, 40, 200, GP2Y0A41);

  mSettings.kp = 5;
  mSettings.ki = 0.01;
  mSettings.kd = 0.05;
  //  mSettings.kp = 5;
  //   mSettings.ki = 0.01;
  //   mSettings.kd = 0.05;
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
