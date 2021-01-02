
#include "BalanceRobot.h"

BalanceRobot::BalanceRobot()
{
  init(0.032,  0.1595, 390, 40, 180, GP2Y0A21); //19 85
  mSettings.atObstacle = 0.3;
  mSettings.unsafe = 0.1;
  mSettings.dfw = 0.25;
  mSettings.kp = 0.5;     //5,  2; //25;  //20 0.5 2; 2019-01-26:   5, 0.02, 0.9; 5, 0.05, 1.2; 5,0.08,1.2 2019-02-09 5, 0.01, 0.2
  mSettings.ki = 0.6;   //0, .4; // 0.01;
  mSettings.kd = 0.0005;     //0, 0.02; //0.2

  mSettings.dkp = 1; // 0.3; //0.38; 
  mSettings.dki = 0.2; // 0.8; //0.26;
  mSettings.dkd = 0.005; //0; //0.003;

  mSettings.useIMU = false;
  mSettings.irFilter = false;
  mSettings.imuAlpha = 0.7;
  mSettings.irAlpha = 0.5;

  mSettings.sampleTime = 30; //50;


  irSensors[0] = new IRSensor(-0.045, 0.05, PI / 2, A1, GP2Y0A21);
  irSensors[1] = new IRSensor(0.08, 0.04, PI / 4, A2, GP2Y0A21); //0.16,0.045, PI/6 0.075, 0.035
  irSensors[2] = new IRSensor(0.162, 0.0, 0, A3, GP2Y0A41);
  irSensors[3] = new IRSensor(0.08, -0.04, -PI / 4, A4, GP2Y0A21);
  irSensors[4] = new IRSensor(-0.045, -0.05, -PI / 2, A5, GP2Y0A21);

  haveIrSensor[0] = false;
  haveIrSensor[1] = false;
  haveIrSensor[2] = false;
  haveIrSensor[3] = false;
  haveIrSensor[4] = false;
}

PWM_OUT BalanceRobot::getPWMOut(double v, double w)
{
  PWM_OUT pwm;
  pwm.pwm_l = 0;
  pwm.pwm_r = 0;

  return pwm;
}


double BalanceRobot::vel_l_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);
  // if (nvel < min_vel)
  //   return 0;

  if (nvel > max_vel)
    nvel = max_vel;

  // double retVal = 0.5729 * nvel * nvel - 5.1735 * nvel + 86.516;
  // double retVal = 9.1631 * nvel + 27.898; //6.393 * nvel + 13.952;
  //y = 0.1086x2 + 3.4864x + 60.919
// y = 0.1485x2 - 0.316x + 55.542

  double retVal = 0.1485 * nvel * nvel - 0.316 * nvel + 55.541;
  // if( retVal < 0 )
  //   retVal =0;

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double BalanceRobot::vel_r_to_pwm(double vel)
{
  //ax^2+bx+c
  double nvel = abs(vel);

  if (nvel < min_vel)
    return 0; //nvel = min_vel;

  if (nvel > max_vel)
    nvel = max_vel;

//y = 0.2013x2 - 2.9149x + 72.184

  double retVal = 0.2013 * nvel * nvel - 2.9149 * nvel + 72.184;
  if( retVal < 0 )
    retVal = 0;

  if (vel >= 0)
    return retVal;
  else
    return -retVal;
}

double BalanceRobot::pwm_to_ticks_l(double pwm, double dt)
{
  double npwm = abs(pwm);
  if (npwm < 60) //14
    return 0;
  double ticks = dt * (-0.0264 * npwm * npwm + 16.836 * npwm - 882.53);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}

double BalanceRobot::pwm_to_ticks_r(double pwm, double dt)
{

  double npwm = abs(pwm);
  if (npwm < 60)
    return 0;
  double ticks = dt * (-0.0312 * npwm * npwm + 18.344 * npwm - 974.3);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}
