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
  // init(0.0312,  0.1638, 990, 990, 19, 150, GP2Y0A21);
  init(0.0311,  0.1622, 990, 990, 19, 150, GP2Y0A21);
  
  max_w = 3; // 1.5; 
  mSettings.max_w = 3; //1.5;

  vel_pwm_a = 13;
  vel_pwm_b = 24;

//pwm to tickes paramaters
  //  y = -0.0341x2 + 16.786x - 633.3
  //y = -0.0585x2 + 21.217x - 564.14
  pta_l = -0.0585;
  ptb_l = 21.217;
  ptc_l = -564.14;

  //y = -0.0367x2 + 17.236x - 630.72
//y = -0.0707x2 + 23.943x - 709.5
  pta_r = -0.0707;
  ptb_r = 23.943;
  ptc_r = -709.5;

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
  irSensors[0] = new IRSensor(-0.085, 0.067, PI / 2, A1, GP2Y0A21);
  irSensors[1] = new IRSensor(0.052, 0.057, PI / 4, A2, GP2Y0A21); // 0.16,0.045, PI/6 0.075, 0.035
  irSensors[2] = new IRSensor(0.063, 0.0, 0, A3, GP2Y0A21);
  irSensors[3] = new IRSensor(0.052, -0.057, -PI / 4, A4, GP2Y0A21);
  irSensors[4] = new IRSensor(-0.085, -0.067, -PI / 2, A5, GP2Y0A21);

	
//	-0.085	0.067
//	0.052	0.057
//	0.063	0
//	0.052	-0.057
//	-0.085	-0.067
	

  haveIrSensor[0] = true;
  haveIrSensor[1] = true;
  haveIrSensor[2] = true;
  haveIrSensor[3] = true;
  haveIrSensor[4] = true;


  mSettings.atObstacle = 0.3;
  mSettings.unsafe = 0.1;
  mSettings.dfw = 0.25;
  mSettings.max_w = max_w;




 // m_dkp = 10, m_dki = 0.20, m_dkd = 0.1; // direction
  //平衡之家
  // m_pkp = 0.5, m_pki = 0.1, m_pkd = 0.0; // position
  // m_tkp = 20, m_tki = 0.7, m_tkd = 0.0;  // theta

  //1：90
  // m_pkp = 1.50, m_pki = 0.01, m_pkd = 0.0; // position
  // m_tkp = 4, m_tki = 0.1, m_tkd = 0.0; // theta

  //平衡之家
  // mPIDSettings.kp = 5;   // 5; //25;  //20 0.5 2; 2019-01-26:   5, 0.02, 0.9; 5, 0.05, 1.2; 5,0.08,1.2 2019-02-09 5, 0.01, 0.2
  // mPIDSettings.ki = 0.2; //.4; // 0.01;
  // mPIDSettings.kd = 0.0; //0.02; //0.2

  //1:90
  mSettings.kp = 0.8;     //5,  2; //25;  //20 0.5 2; 2019-01-26:   5, 0.02, 0.9; 5, 0.05, 1.2; 5,0.08,1.2 2019-02-09 5, 0.01, 0.2
  mSettings.ki = 0.8;   //0, .4; // 0.01;
  mSettings.kd = 0.0005;     //0, 0.02; //0.2

  mSettings.pkp = 1.5;
  mSettings.pki = 0.1;
  mSettings.pkd = 0;

  mSettings.tkp = 1.5;
  mSettings.tki = 0.2;
  mSettings.tkd = 0;

//50ms sample time param
  // mSettings.dkp = 0.8; 
  // mSettings.dki = 0.6;
  // mSettings.dkd = 0.003;


//30ms sample time param 0.18 0.4 0.003
  mSettings.dkp = 1; // 0.3; //0.38; 
  mSettings.dki = 0.2; // 0.8; //0.26;
  mSettings.dkd = 0.005; //0; //0.003;


  //   bool useIMU, irFilter;
  // double imuAlpha, irAlpha;
  mSettings.useIMU = false;
  mSettings.irFilter = false;
  mSettings.imuAlpha = 0.7;
  mSettings.irAlpha = 0.5;

  mSettings.sampleTime = 30; //50;
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


		if( v == 0 ) //
		{
  		Vel vel = uni_to_diff(v, w);
			if( abs( vel.vel_l ) > 1.5*min_vel )
			{
				if( vel.vel_l < 0 )
				{
					vel.vel_l = -1.5*min_vel;
					vel.vel_r = 1.5*min_vel;
				}
				else
				{
					vel.vel_l = 1.5*min_vel;
					vel.vel_r = -1.5*min_vel;

				}
			}

    Serial.print(w, 3);
    Serial.write(',');
    Serial.print(v, 3);
    Serial.write(',');
    Serial.print(v, 3);
    Serial.write(',');
    Serial.print(vel.vel_l, 3);
    Serial.write(',');
    Serial.println(vel.vel_r, 3);
			return vel;			
		}

		double sw = w;
		if (sw > max_w)
		  sw = max_w;  
		else if (sw < -max_w)
		  sw = -max_w;
    double sv = abs(v);
    sv = -sv/max_w * abs(sw) + sv;
    if( sv <= 0 )
      sv = 0.01;

    if( v < 0  )
      sv = -sv;
		
    Vel vel = uni_to_diff(sv, sw);
	  
		if (vel.vel_l * vel.vel_r >= 0)
    {
    Serial.print(sw, 3);
    Serial.write(',');
    Serial.print(v, 3);
    Serial.write(',');
    Serial.print(sv, 3);
    Serial.write(',');
    Serial.print(vel.vel_l, 3);
    Serial.write(',');
    Serial.println(vel.vel_r, 3);      
		  return vel;
    }  
		if (abs(vel.vel_l) > abs(vel.vel_r)) 
		{
		  vel.vel_r = 0;
		}
		else
		  vel.vel_l = 0;
	  
    Serial.print(sw, 3);
    Serial.write(',');
    Serial.print(v, 3);
    Serial.write(',');
    Serial.print(sv, 3);
    Serial.write(',');
    Serial.print(vel.vel_l, 3);
    Serial.write(',');
    Serial.println(vel.vel_r, 3);




		return vel;		


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
