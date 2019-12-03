#include "RearDriveRobot.h"

RearDriveRobot::RearDriveRobot()
{
  //R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
  // init(0.0325, 0.1785, 330, 360, 60, 150, GP2Y0A41); //0.0325 0.1785； 0.0325， 0.156

  // init(0.0330, 0.16, 390, 390, 50, 180, GP2Y0A41); 2019-02-09 蓝色轮子，有一边转弯差点
  //0.0325 0.1785； 0.0325， 0.156
  // init(0.0313, 0.1631, 390, 390, 50, 180, GP2Y0A21);

  //黑色轮子，长板
  // init(0.0312, 0.1666, 390, 390, 50, 180, GP2Y0A21);

  //黑色轮子，自制板
  // init(0.0312, 0.162, 390, 390, 50, 180, GP2Y0A21);

  //黑色轮子，自制板 新1：90电机
  init(0.0317, 0.162, 990, 990, 20, 80, GP2Y0A21);
  
  max_w = 0.8; 

  vel_pwm_a = 28;
  vel_pwm_b = -20;

//pwm to tickes paramaters
  //  y = -0.0341x2 + 16.786x - 633.3
  pta_l = -0.0341;
  ptb_l = 16.786;
  ptc_l = -633.3;

  //y = -0.0367x2 + 17.236x - 630.72
  pta_r = -0.0367;
  ptb_r = 17.236;
  ptc_r = -630.72;

  // int maxPWM, minPWM;
  // // pwm = vel_pwm_a * vel + vel_pwm_b
  // double vel_pwm_a, vel_pwm_b;
  // //pwm to ticks ticks = dt*(pta * pwm^2 + ptb *pwm + ptc)
  // double pta_l, pta_r, ptb_l, ptb_r, ptc_l, ptc_r;

  //balance car
  //  init(0.0332, 0.159, 390, 390, 50, 180, GP2Y0A21);

  //GP2Y0A21 (10-80) GP2Y0A41 (4-30) 长版
  // irSensors[0] = new IRSensor(-0.045, 0.05, PI / 2, A1, GP2Y0A21);
  // irSensors[1] = new IRSensor(0.08, 0.04, PI / 4, A2, GP2Y0A21); //0.16,0.045, PI/6 0.075, 0.035
  // irSensors[2] = new IRSensor(0.162, 0.0, 0, A3, GP2Y0A21);
  // irSensors[3] = new IRSensor(0.08, -0.04, -PI / 4, A4, GP2Y0A21);
  // irSensors[4] = new IRSensor(-0.045, -0.05, -PI / 2, A5, GP2Y0A21);

  //GP2Y0A21 (10-80) GP2Y0A41 (4-30) 自制版
  irSensors[0] = new IRSensor(-0.073, 0.066, PI / 2, A1, GP2Y0A21);
  irSensors[1] = new IRSensor(0.061, 0.05, PI / 4, A2, GP2Y0A21); // 0.16,0.045, PI/6 0.075, 0.035
  irSensors[2] = new IRSensor(0.072, 0.0, 0, A3, GP2Y0A21);
  irSensors[3] = new IRSensor(0.061, -0.05, -PI / 4, A4, GP2Y0A21);
  irSensors[4] = new IRSensor(-0.073, -0.066, -PI / 2, A5, GP2Y0A21);

  haveIrSensor[0] = true;
  haveIrSensor[1] = true;
  haveIrSensor[2] = true;
  haveIrSensor[3] = true;
  haveIrSensor[4] = true;

  //平衡之家
  // mPIDSettings.kp = 5;   // 5; //25;  //20 0.5 2; 2019-01-26:   5, 0.02, 0.9; 5, 0.05, 1.2; 5,0.08,1.2 2019-02-09 5, 0.01, 0.2
  // mPIDSettings.ki = 0.2; //.4; // 0.01;
  // mPIDSettings.kd = 0.0; //0.02; //0.2

  //1:90
  mPIDSettings.kp = 1.2;    // 5; //25;  //20 0.5 2; 2019-01-26:   5, 0.02, 0.9; 5, 0.05, 1.2; 5,0.08,1.2 2019-02-09 5, 0.01, 0.2
  mPIDSettings.ki = 0.01;  //.4; // 0.01;
  mPIDSettings.kd = 0.0; //0.02; //0.2
}

PWM_OUT RearDriveRobot::getPWMOut(double v, double w)
{

  Vel vel = ensure_w(v, w);

  int pwm_l = (int)vel_l_to_pwm(vel.vel_l);
  int pwm_r = (int)vel_r_to_pwm(vel.vel_r);

  // if (v == 0)
  // {
  //   if (abs(pwm_l) > minPWM + 30)
  //   {
  //     if (pwm_l > 0)
  //     {
  //       pwm_l = minPWM + 30;
  //       pwm_r = -(minPWM + 30);
  //     }
  //     else
  //     {
  //       pwm_l = -(minPWM + 30);
  //       pwm_r = minPWM + 30;
  //     }
  //   }
  // }

  PWM_OUT pwm;
  pwm.pwm_l = pwm_l;
  pwm.pwm_r = pwm_r;
  return pwm;
}

Vel RearDriveRobot::ensure_w(double v, double w)
{

  if (v == 0)
  {
    Vel vel = uni_to_diff(v, w);
    if( abs(vel.vel_l) < min_vel )
    {
        if( vel.vel_l < 0 )
        {
            vel.vel_l = -min_vel;
            vel.vel_r = min_vel;
        }
        else
        {
            vel.vel_l = min_vel;
            vel.vel_r = -min_vel;
        }
   }
    return vel;
  }

  double sw = w;
  if (sw > max_w)
    sw = max_w;
  else if (sw < -max_w)
    sw = -max_w;
  Vel vel = uni_to_diff(v, sw);

  if (vel.vel_l * vel.vel_r >= 0)
    return vel;

  if (abs(vel.vel_l) > abs(vel.vel_r))
  {
    vel.vel_r = 0;
  }
  else
    vel.vel_l = 0;

  return vel;

  // if (abs(v) > 0)
  // {
  //   if (abs(w) > 1.2)
  //   {
  //     vel = uni_to_diff(v, w);
  //     vel = zeroMinVel(vel);
  //     return vel;
  //   }

  //   Vel vel_d = uni_to_diff(abs(v), w); // w_lim);

  //   vel.vel_r = vel_d.vel_r;
  //   vel.vel_l = vel_d.vel_l;

  //   if (vel.vel_l > max_vel)
  //     vel.vel_l = max_vel;
  //   if (vel.vel_r > max_vel)
  //     vel.vel_r = max_vel;

  //   if (vel.vel_l < 0)
  //     vel.vel_l = 0;
  //   if (vel.vel_r < 0)
  //     vel.vel_r = 0;

  //   if (v < 0)
  //   {
  //     vel.vel_l = -vel.vel_l;
  //     vel.vel_r = -vel.vel_r;
  //   }
  // }
  // else
  // {
  //   vel = uni_to_diff(0, w);
  //   vel = zeroMinVel(vel);
  //   // if (vel.vel_l < 0)
  //   // {
  //   //   vel.vel_l = 0;
  //   //   vel.vel_r = min_vel + 0.5;
  //   // }
  //   // else
  //   // {
  //   //   vel.vel_r = 0;
  //   //   vel.vel_l = min_vel + 0.5;
  //   // }
  // }
  // return vel;
}

/*
Vel RearDriveRobot::ensure_w(double v, double w)
{
  Vel vel;

  if (abs(v) > 0)
  {
    Vel vel_d = uni_to_diff(abs(v), w); // w_lim);

    double vel_rl_max, vel_rl_min;
    if (vel_d.vel_l > vel_d.vel_r)
    {
      vel_rl_min = vel_d.vel_r;
      vel_rl_max = vel_d.vel_l;
    }
    else
    {
      vel_rl_min = vel_d.vel_l;
      vel_rl_max = vel_d.vel_r;
    }

    if (vel_rl_max > max_vel)
    {
      vel.vel_r = vel_d.vel_r - (vel_rl_max - max_vel);
      vel.vel_l = vel_d.vel_l - (vel_rl_max - max_vel);
      if ((vel_rl_min - (vel_rl_max - max_vel)) < min_vel) //大拐弯？
      {
        vel = zeroMinVel(vel);
      }
    }
    else if (vel_rl_min < min_vel)
    {
      vel.vel_r = vel_d.vel_r + (min_vel - vel_rl_min);
      vel.vel_l = vel_d.vel_l + (min_vel - vel_rl_min);
      // if (vel_rl_max + (min_vel - vel_rl_min) > max_vel) //大拐弯
      // {
      //   vel = zeroMinVel(vel);
      // }
    }
    else
    {
      vel.vel_r = vel_d.vel_r;
      vel.vel_l = vel_d.vel_l;
    }

    if (vel.vel_l > max_vel)
      vel.vel_l = max_vel;
    else if (vel.vel_r > max_vel)
      vel.vel_r = max_vel;

    if (v < 0)
    {
      vel.vel_l = -vel.vel_l;
      vel.vel_r = -vel.vel_r;
    }
  }
  else
  {
    vel = uni_to_diff(0, w);
    vel = zeroMinVel(vel);
    // if (vel.vel_l < 0)
    // {
    //   vel.vel_l = 0;
    //   vel.vel_r = min_vel + 0.5;
    // }
    // else
    // {
    //   vel.vel_r = 0;
    //   vel.vel_l = min_vel + 0.5;
    // }
  }
  return vel;
}

Vel RearDriveRobot::zeroMinVel(Vel vel)
{
  if (vel.vel_l > vel.vel_r)
  {
    vel.vel_r = 0;
    vel.vel_l = min(max(min_vel, vel.vel_l), (min_vel + 5));
  }
  else
  {
    vel.vel_l = 0;
    vel.vel_r = min(max(min_vel, vel.vel_r), (min_vel + 5));
    //    vel.vel_r = min_vel;
  }
  return vel;
}
*/

double RearDriveRobot::vel_l_to_pwm(double vel)
{
  double nvel = abs(vel);
  if( nvel < min_vel )
    return 0;

  if( nvel > max_vel )
    nvel = max_vel;
  double pwm = vel_pwm_a * nvel + vel_pwm_b;

  if (vel < 0)
    return -pwm;
  return pwm;
}

double RearDriveRobot::vel_r_to_pwm(double vel)
{
  double nvel = abs(vel);
  if( nvel < min_vel )
    return 0;

  if( nvel > max_vel )
    nvel = max_vel;
  double pwm = vel_pwm_a * nvel + vel_pwm_b;

  if (vel < 0)
    return -pwm;
  return pwm;
}

double RearDriveRobot::pwm_to_ticks_l(double pwm, double dt)
{
  if (pwm == 0)
    return 0;

  double npwm = abs(pwm);
  double ticks = dt * (pta_l * npwm * npwm + ptb_l * npwm + ptc_l);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}

double RearDriveRobot::pwm_to_ticks_r(double pwm, double dt)
{
  if (pwm == 0)
    return 0;

  double npwm = abs(pwm);
  double ticks = dt * (pta_r * npwm * npwm + ptb_r * npwm + ptc_r);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}
