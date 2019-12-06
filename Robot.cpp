#include "Robot.h"
#include "ZMCRobot.h"

extern double ultrasonicDistance;

Robot::Robot()
{
  //R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
  init(0.065 / 2, 0.125, 20, 20, 40, 200, GP2Y0A41);
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

  // max_v = max_vel * wheel_radius;
  // min_v = min_vel * wheel_radius;
  // max_w = (wheel_radius / wheel_base_length) * (max_vel); // - min_vel);
  // min_w = (wheel_radius / wheel_base_length) * (min_vel);

  pwm_diff = 0;
  // angleOff = 0;

  // irSensors[0] = new IRSensor(-0.045, 0.05, PI / 2, A1, sensorType);
  // irSensors[1] = new IRSensor(0.160, 0.045, PI / 6, A2, sensorType);
  // irSensors[2] = new IRSensor(0.162, 0.0, 0, A3, sensorType);
  // irSensors[3] = new IRSensor(0.160, -0.045, -50.0*PI /180.0, A4, sensorType);
  // irSensors[4] = new IRSensor(-0.045, -0.05, -PI / 2, A5, sensorType);

  // irSensors[0] = new IRSensor(-0.1, 0.055, PI / 2, A1, sensorType); //A1
  // irSensors[1] = new IRSensor(0.075, 0.06, PI / 4, A2, sensorType);
  // irSensors[2] = new IRSensor(0.085, 0., 0, A3, sensorType);
  // irSensors[3] = new IRSensor(0.075, -0.06, -PI / 4, A4, sensorType);
  // irSensors[4] = new IRSensor(-0.1, -0.055, -PI / 2, A5, sensorType);
}

void Robot::setIRSensorType(SENSOR_TYPE sensorType)
{
  for (int i = 0; i < 5; i++)
    irSensors[i]->SetSensorType(sensorType);
}

void Robot::updatePID(SETTINGS pids)
{
  mPIDSettings.kd = pids.kd;
  mPIDSettings.ki = pids.ki;
  mPIDSettings.kp = pids.kp;
}

void Robot::updateSettings(SETTINGS settings)
{

  wheel_radius = settings.radius;
  wheel_base_length = settings.length;

  max_rpm = settings.max_rpm; //267
  max_vel = max_rpm * 2 * PI / 60;

  min_rpm = settings.min_rpm; //113
  min_vel = min_rpm * 2 * PI / 60;

  max_w = settings.max_w;

  // max_v = max_vel * wheel_radius;
  // min_v = min_vel * wheel_radius;
  // max_w = (wheel_radius / wheel_base_length) * (max_vel); // - min_vel);
  // min_w = (wheel_radius / wheel_base_length) * (min_vel);
  // max_w = (wheel_radius / wheel_base_length) * (max_vel - min_vel);
  // min_w = (wheel_radius / wheel_base_length) * (2 * min_vel);

  pwm_diff = settings.pwm_diff;
  // angleOff = settings.angleOff;
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

void Robot::updateState(long left_ticks, long right_ticks, double gyro, double alpha, double dt)
{
  //  long left_ticks, right_ticks;
  if (prev_right_ticks == right_ticks && prev_left_ticks == left_ticks)
  {
    w = 0;
    velocity = 0;
    readIRSensors( dt );
    return; //no change
  }

  double d_right, d_left, d_center;

  vel_l = ((double)abs(left_ticks - prev_left_ticks) / dt) / (double)ticks_per_rev_l;
  vel_r = ((double)abs(right_ticks - prev_right_ticks) / dt) / (double)ticks_per_rev_r;

  vel_l = 2 * PI * vel_l;
  vel_r = 2 * PI * vel_r;

  d_left = (left_ticks - prev_left_ticks) * m_per_tick_l;
  d_right = (right_ticks - prev_right_ticks) * m_per_tick_r;

  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  d_center = (d_right + d_left) / 2;
  velocity = d_center / dt;

  double phi = (d_right - d_left) / wheel_base_length;

  double gyro_ = gyro * PI / 360.0;
  double dgyt = gyro_ * dt;

  phi = alpha * phi + (1 - alpha) * dgyt;

  w = phi / dt;

  x = x + d_center * cos(theta);
  y = y + d_center * sin(theta);
  theta = theta + phi;
  theta = atan2(sin(theta), cos(theta));

  readIRSensors(dt);
}


void Robot::updateState(long left_ticks, long right_ticks, double dt)
{
  //  long left_ticks, right_ticks;
  if (prev_right_ticks == right_ticks && prev_left_ticks == left_ticks)
  {
    w = 0;
    velocity = 0;
    readIRSensors(dt);
    return; //no change
  }

  double d_right, d_left, d_center;

  vel_l = ((double)abs(left_ticks - prev_left_ticks) / dt) / (double)ticks_per_rev_l;
  vel_r = ((double)abs(right_ticks - prev_right_ticks) / dt) / (double)ticks_per_rev_r;

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
    if (haveIrSensor[i])
      irSensors[i]->readPosition();
  }

  double maxDis = velocity * dt;

  for (int i = 0; i < 5; i++)
  {
    //如果读入的障碍物距离突变（大于速度*时间），重新读入
    if (haveIrSensor[i] && abs(irSensors[i]->lastDistance - irSensors[i]->distance) > maxDis)
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
      floatToStr(0, 100 * wheel_radius),
      floatToStr(1, 100 * wheel_base_length),
      ticks_per_rev_l,
      ticks_per_rev_r);

  log("Balance, Q=%s, gyro=%s\n",
      floatToStr(0, angle),
      floatToStr(1, gyro));

  if (irSensors[0]->getSensorType() == GP2Y0A41) //GP2Y0A41 = 0,     //4-30cm  GP2Y0A21
    Serial.print("IR [GP2Y0A41]:");
  else
    Serial.print("IR [GP2Y0A21]:");

  readIRSensors( 0 );

  for (int i = 0; i < 5; i++)
  {
    Serial.print(irSensors[i]->distance);
    Serial.print(",");
  }
  Serial.println(";");
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

Vel Robot::uni_to_diff(double v, double w)
{
  Vel vel;
  vel.vel_r = (2 * v + w * wheel_base_length) / (2 * wheel_radius);
  vel.vel_l = (2 * v - w * wheel_base_length) / (2 * wheel_radius);
  return vel;

  //double vv = -0.0537*w + 0.8054;
  //if( vv < 0.1)
  //  vv = 0.1;
  // if( vv > 0.8 )
  //  vv = 0.8;
  //
  //  vel.vel_r = (2 * vv + w * wheel_base_length) / (2 * wheel_radius);
  //  vel.vel_l = (2 * vv - w * wheel_base_length) / (2 * wheel_radius);
  //
  //  return vel;
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
