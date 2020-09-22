#include "Robot.h"
#include "ZMCRobot.h"

extern double ultrasonicDistance;

Robot::Robot()
{
  //R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
  // init(0.0325, 0.0325, 0.125, 20, 20, 40, 200, GP2Y0A41);
}

// Robot::Robot(double R, double L, double ticksr_l, double ticksr_r, double minRpm, double maxRpm)
// {
//   init(R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
// }

void Robot::init(double R, double L, double ticksr_l, double ticksr_r, double minRpm, double maxRpm, SENSOR_TYPE sensorType)
{
  x = 0;
  y = 0;
  theta = 0;
  //  vel_l = 0;
  //  vel_r = 0;
  prev_left_ticks = 0;
  prev_right_ticks = 0;

  wheel_radius = R;           //0.065 / 2;
  wheel_base_length = L;      // 0.127;
  ticks_per_rev_l = ticksr_l; //20;
  ticks_per_rev_r = ticksr_r; //20;
  m_per_tick_l = 2 * PI * wheel_radius / ticks_per_rev_l;
  m_per_tick_r = 2 * PI * wheel_radius / ticks_per_rev_r;

  max_rpm = maxRpm; //160; //267
  max_vel = max_rpm * 2 * PI / 60;

  min_rpm = minRpm; // 70; //113
  min_vel = min_rpm * 2 * PI / 60;

  pwm_diff = 0;
  prev_left_ticks = 0;
  prev_right_ticks = 0;
  vel_l = 0;
  vel_r = 0;

  mSettings.radius = R;
  mSettings.length = L;

  mSettings.min_rpm = min_rpm;
  mSettings.max_rpm = max_rpm;


}

void Robot::setIRSensorType(SENSOR_TYPE sensorType)
{
  for (int i = 0; i < 5; i++)
    irSensors[i]->SetSensorType(sensorType);
}

// void Robot::updatePID(SETTINGS settings)
// {
//   if( settings.type == 1 )
//   {
//     mSettings.kd = settings.kd;
//     mSettings.ki = settings.ki;
//     mSettings.kp = settings.kp;
//   }
//   else if(settings.type == 2 )
//   {
//     mSettings.pkd = settings.pkd;
//     mSettings.pki = settings.pki;
//     mSettings.pkp = settings.pkp;

//   }
//   else if(settings.type == 3 )
//   {
//     mSettings.tkd = settings.tkd;
//     mSettings.tki = settings.tki;
//     mSettings.tkp = settings.tkp;

//   }
//   else if(settings.type == 4 )
//   {
//     mSettings.dkd = settings.dkd;
//     mSettings.dki = settings.dki;
//     mSettings.dkp = settings.dkp;

//   }

// }

// void Robot::updatePID(int type, double kp, double ki, double kd )
// {
//   if( type == 1 )
//   {
//     mSettings.kd = kd;
//     mSettings.ki = ki;
//     mSettings.kp = kp;
//   }
//   else if(type == 2 )
//   {
//     mSettings.pkd = pkd;
//     mSettings.pki = pki;
//     mSettings.pkp = pkp;

//   }
//   else if(type == 3 )
//   {
//     mSettings.tkd = tkd;
//     mSettings.tki = tki;
//     mSettings.tkp = tkp;

//   }
//   else if(type == 4 )
//   {
//     mSettings.dkd = dkd;
//     mSettings.dki = dki;
//     mSettings.dkp = dkp;

//   }  
// }

void Robot::updateSettings(SETTINGS settings)
{

  if( settings.sType == 0 || settings.sType == 5 || settings.sType == 6)
  {
    wheel_radius =  settings.radius;
    wheel_base_length = settings.length;
    max_rpm = settings.max_rpm; //267
    max_vel = max_rpm * 2 * PI / 60;
    min_rpm = settings.min_rpm; //113
    min_vel = min_rpm * 2 * PI / 60;
    max_w = settings.max_w;
    pwm_diff = settings.pwm_diff;

    mSettings.radius = wheel_radius;
    mSettings.length = wheel_base_length;
    mSettings.min_rpm = min_rpm;
    mSettings.max_rpm = max_rpm;
    mSettings.max_w = max_w;

    mSettings.atObstacle = settings.atObstacle;
    mSettings.unsafe = settings.unsafe;
    mSettings.dfw = settings.dfw;
    mSettings.sampleTime = settings.sampleTime;
  }
  else if( settings.sType == 1)
  {
      mSettings.kp = settings.kp;
      mSettings.ki = settings.ki;
      mSettings.kd = settings.kd;
  }
  else if( settings.sType == 2 )
  {
      mSettings.pkp = settings.pkp;
      mSettings.pki = settings.pki;
      mSettings.pkd = settings.pkd;

  }
  else if( settings.sType == 3 )
  {
      mSettings.tkp = settings.tkp;
      mSettings.tki = settings.tki;
      mSettings.tkd = settings.tkd;

  }

  else if( settings.sType == 4 )
  {
      mSettings.dkp = settings.dkp;
      mSettings.dki = settings.dki;
      mSettings.dkd = settings.dkd;
  }


}

void Robot::setPIDParams( int type, double kp, double ki, double kd )
{

  if( type == 1)
  {
      mSettings.kp = kp;
      mSettings.ki = ki;
      mSettings.kd = kd;
  }
  else if( type == 2 )
  {
      mSettings.pkp = kp;
      mSettings.pki = ki;
      mSettings.pkd = kd;

  }
  else if( type == 3 )
  {
      mSettings.tkp = kp;
      mSettings.tki = ki;
      mSettings.tkd = kd;

  }

  else if( type == 4 )
  {
      mSettings.dkp = kp;
      mSettings.dki = ki;
      mSettings.dkd = kd;
  }

}


void Robot::reset(long left_ticks, long right_ticks)
{
  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;
  //  x = 0;
  //  y = 0;
  //  theta = 0;
}


//融合IMU 的陀螺仪信息，计算转向 gyro为 度/秒  alpha 转弯角度融合参数 1-0； 1时只用编码器的

void Robot::updateState(long left_ticks, long right_ticks, double yaw,  double dt)
{
  //  long left_ticks, right_ticks;
  if (prev_right_ticks == right_ticks && prev_left_ticks == left_ticks)
  {
    w = 0;
    velocity = 0;
    vel_l = 0;
    vel_r = 0;
    readIRSensors( dt );
    return; //no change
  }

  double d_right, d_left, d_center;

  vel_l = ((double)(left_ticks - prev_left_ticks) / dt) / (double)ticks_per_rev_l;
  vel_r = ((double)(right_ticks - prev_right_ticks) / dt) / (double)ticks_per_rev_r;
  vel_l = 2 * PI * vel_l;
  vel_r = 2 * PI * vel_r;

  d_left = (left_ticks - prev_left_ticks) * m_per_tick_l;
  d_right = (right_ticks - prev_right_ticks) * m_per_tick_r;

  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  d_center = (d_right + d_left) / 2;
  velocity = d_center / dt;
  double phi = (d_right - d_left) / wheel_base_length;

  if( mSettings.useIMU )
  {
    double alpha = 0;
    double delta_theta = yaw - prev_yaw;
    delta_theta = atan2(sin(delta_theta), cos(delta_theta));
    prev_yaw = yaw;
    alpha = mSettings.imuAlpha;
    delta_theta = delta_theta * alpha + (1 - alpha ) * phi;
    x+= d_center * cos(theta + (delta_theta / 2.0));
    y+= d_center* sin(theta + (delta_theta / 2.0));
    theta = theta + delta_theta;
    theta = atan2(sin(theta), cos(theta));
  // compute odometric instantaneouse velocity
    w = delta_theta / dt;
  }
  else
  {
    w = phi / dt;
    // x = x + d_center * cos(theta + phi/2);
    // y = y + d_center * sin(theta + phi/2);
    x = x + d_center * cos(theta);
    y = y + d_center * sin(theta);
    theta = theta + phi;
    theta = atan2(sin(theta), cos(theta));
  }

  readIRSensors(dt);
}


double Robot::normalizeVel(double refVel, double inVel )
{
    if( refVel * inVel < 0 )
      return 0;
    
    if( inVel > max_vel )
      return max_vel;
    if( inVel < -max_vel )
      return -max_vel;
    
    return inVel;

}

void Robot::updateState(long left_ticks, long right_ticks, double dt)
{
  //  long left_ticks, right_ticks;
  if (prev_right_ticks == right_ticks && prev_left_ticks == left_ticks)
  {
    w = 0;
    velocity = 0;
    vel_l = 0;
    vel_r = 0;
    readIRSensors(dt);
    return; //no change
  }

  double d_right, d_left, d_center;

  vel_l = ((double)(left_ticks - prev_left_ticks) / dt) / (double)ticks_per_rev_l;
  vel_r = ((double)(right_ticks - prev_right_ticks) / dt) / (double)ticks_per_rev_r;
  vel_l = 2 * PI * vel_l;
  vel_r = 2 * PI * vel_r;

 
  d_left = (left_ticks - prev_left_ticks) * m_per_tick_l;
  d_right = (right_ticks - prev_right_ticks) * m_per_tick_r;



  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  d_center = (d_right + d_left) / 2;
  velocity = d_center / dt;

  double phi = (d_right - d_left) / wheel_base_length;

  w = phi / dt;

  x = x + d_center * cos(theta);
  y = y + d_center * sin(theta);
  theta = theta + phi;
  theta = atan2(sin(theta), cos(theta));

  readIRSensors(dt);
}

void Robot::readIRSensors(double dt)
{
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);

  for (int i = 0; i < 5; i++)
  {
    if (haveIrSensor[i] != 0 )
      irSensors[i]->readPosition();
  }

  double maxDis = velocity * dt;

  for (int i = 0; i < 5; i++)
  {
    //如果读入的障碍物距离突变（大于速度*时间），重新读入
    if ((haveIrSensor[i] != 0 ) && abs(irSensors[i]->lastDistance - irSensors[i]->distance) > maxDis)
      irSensors[i]->readPosition();
  }

  if (haveIrSensor[2] == 2)
  {
    irSensors[2]->setDistance(ultrasonicDistance);
  }
  else if (haveIrSensor[2] == 3)
  {
    irSensors[2]->setDistance((ultrasonicDistance + irSensors[2]->distance) / 2);
  }
  // if (haveIrSensor[2] && irSensors[2]->distance >= irSensors[2]->getMaxDistance())
  // {
  //   irSensors[2]->setDistance(ultrasonicDistance);
  // }
  // else if (!haveIrSensor[2])
  // {
  //   irSensors[2]->setDistance(ultrasonicDistance);
  // }

  for (int i = 0; i < 5; i++)
    irSensors[i]->applyGeometry(x, y, sinTheta, cosTheta);
}

void Robot::setObstacleDistance(double dis[5])
{
  for (int i = 0; i < 5; i++)
  {
    irSensors[i]->setDistance(dis[i]);
  }
}

void Robot::getRobotInfo()
{

  log("x:%s,y:%s,Q:%s,v:%s\n",
      floatToStr(0, x),
      floatToStr(1, y),
      floatToStr(2, theta),
      floatToStr(3, velocity));

  log("vel-l:%s, vel-r:%s;\nmax_vel:%s, min_vel:%s\nmax_rpm:%s,min_rpm:%s\n",
      floatToStr(0, vel_l),
      floatToStr(1, vel_r),
      floatToStr(2, max_vel),
      floatToStr(3, min_vel),
      floatToStr(4, max_rpm),
      floatToStr(5, min_rpm));

  log("robot(R,L,tks):%s, %s, %d, %d\n",
      floatToStr(1, 1000 * wheel_radius),
      floatToStr(2, 1000 * wheel_base_length),
      ticks_per_rev_l,
      ticks_per_rev_r);

  log("Balance, Q=%s, gyro=%s\n",
      floatToStr(0, angle),
      floatToStr(1, gyro));

  if (irSensors[0]->getSensorType() == GP2Y0A41) //GP2Y0A41 = 0,     //4-30cm  GP2Y0A21
    Serial.print("I R [GP2Y0A41]:");
  else
    Serial.print("I R [GP2Y0A21]:");

  readIRSensors( 0 );

  for (int i = 0; i < 5; i++)
  {
    Serial.print(irSensors[i]->distance);
    Serial.print(",");
  }
  Serial.println(";");
  Serial.print("Use IMU:");
  Serial.print(mSettings.useIMU);
  Serial.print(", alpha:");
  Serial.println( mSettings.imuAlpha );

}

IRSensor **Robot::getIRSensors()
{
  return irSensors;
}

//get the most closer obstacle distance of the front 3;
double Robot::getObstacleDistance()
{

  return irSensors[2]->distance;

  double d = irSensors[1]->distance;
  if (d > irSensors[2]->distance)
    d = irSensors[2]->distance;
  if (d > irSensors[3]->distance)
    d = irSensors[3]->distance;
  return d;
}



//用单边减速的方式转弯；uni_to_diff 采用的是一边加，一边减的方式，速度不变，容易冲
Vel Robot::uni_to_diff_oneside(double v, double w)
{

  Vel vel;
  double wv = (w*wheel_base_length)/(2*wheel_radius);
  if( abs(v) <= 0.001 )
  {
    double wvel = wv;
    if( abs(wv) < 1.2*min_vel )
    {
      if( wv < 0 )
        wvel = -1.2*min_vel;
      else
      {
        wvel = 1.2*min_vel;
      }
    }
    
    vel.vel_r =  wvel; //(2 * v + w * l) / (2 * r);
    vel.vel_l = -wvel; // (2 * v - w * l) / (2 * r);
    return vel;
  }

  double vvel = v/wheel_radius;
  if( abs(vvel) < min_vel )
  {
    if( vvel < 0 )
      vvel = -min_vel;
    else
      vvel = min_vel;
    
  }
  if( v*w >= 0 )
  {
      vel.vel_r = vvel;
      vel.vel_l = vvel - wv*2;
      if( vel.vel_l * v < 0 )
        vel.vel_l = 0;
  }
  else
  {
      vel.vel_l = vvel;
      vel.vel_r = vvel + wv*2;
      if( vel.vel_r * v < 0 )
        vel.vel_r = 0;
  }

  return vel;

}



Vel Robot::uni_to_diff(double v, double w)
{
  Vel vel;
  vel.vel_r = (2 * v + w * wheel_base_length) / (2 * wheel_radius);
  vel.vel_l = (2 * v - w * wheel_base_length) / (2 * wheel_radius);
  return vel;
}

  Vel Robot::uni_to_diff_v(double v, double w )
  {
    Vel vel;
    if( abs(v) <= 0.001 )
    {
      vel.vel_r = (2 * v + w * wheel_base_length) / (2 * wheel_radius);
      vel.vel_l = (2 * v - w * wheel_base_length) / (2 * wheel_radius);
      return vel;
    }

    double vvel = v/wheel_radius;
    if( abs(vvel) < 1.2*min_vel )
    {
        if( v < 0 )
          vvel =  -1.2*min_vel;
        else
        {
          vvel =  1.2*min_vel;
        }
         
    }

    vel.vel_l = vvel;
    vel.vel_r = vvel;


    double wv =  ( w * wheel_base_length) / (2 * wheel_radius);
    if( w < 0 )
    {
      if( v < 0 )
      {
          vel.vel_l = vel.vel_l - wv;
          if( vel.vel_l > 0 )
            vel.vel_l = 0;
      }
      else
      {
          vel.vel_r = vel.vel_r + wv;
          if( vel.vel_r < 0 )
            vel.vel_r = 0;
      }      
    }
    else
    {
      if( v < 0 )
      {
          vel.vel_r = vel.vel_r + wv;
          if( vel.vel_r > 0 )
            vel.vel_r = 0;
      }
      else
      {
          vel.vel_l = vel.vel_l - wv;
          if( vel.vel_l < 0 )
            vel.vel_l = 0;
      }      
    }
    return vel;

  }


Output Robot::diff_to_uni(double vel_l, double vel_r)
{
  Output out;
  if (vel_l + vel_r == 0)
  {
    // Serial.println("div by o...in robot 1");
    out.v = 0.3;
    return out;
  }
  else
    out.v = wheel_radius / 2 * (vel_l + vel_r);

  if (vel_r - vel_l == 0)
  {
    // Serial.println("div by o...in robot 2");
    out.w = PI / 2;
  }
  else
    out.w = wheel_radius / wheel_base_length * (vel_r - vel_l);

  return out;
}
