
#include "Supervisor.h"
#include "ZMCRobot.h"
#include "utils.h"

#define MAX_IRSENSOR_DIS 0.3

Supervisor::Supervisor()
{
  d_fw = 0.25; //distance to follow wall
  d_stop = 0.02;
  d_at_obs = 0.3;
  d_unsafe = 0.1;
  d_prog = 100;

  m_input.x_g = 1;
  m_input.y_g = 0;
  m_input.v = 0.15;
  m_FollowWall.d_fw = 0.25;
  m_FollowWall.dir = 0;

     mUseIMU = false;
   alpha = 0.7;  //计算角度时编码器占比


  //  robot.setVel2PwmParam(0, 6.4141, 14.924); // vel to pwm parameters

  // robot.setVel2PwmParam(0,9.59,18.73);
  // robot.setIRSensorType(GP2Y0A21);
  // robot.setHaveIrSensor(0, true);
  // robot.setHaveIrSensor(1, true);
  // robot.setHaveIrSensor(2, false);
  // robot.setHaveIrSensor(3, true);
  // robot.setHaveIrSensor(4, true);

  // m_dkp = 10, m_dki = 0.20, m_dkd = 0.1; // direction
  //平衡之家
  // m_pkp = 0.5, m_pki = 0.1, m_pkd = 0.0; // position
  // m_tkp = 20, m_tki = 0.7, m_tkd = 0.0;  // theta

  //1：90
  m_pkp = 1.50, m_pki = 0.01, m_pkd = 0.0; // position
  m_tkp = 4, m_tki = 0.1, m_tkd = 0.0; // theta

  mSimulateMode = false;
  mIgnoreObstacle = false;

  m_left_ticks = 0;
  m_right_ticks = 0;
  danger = false;
  execTime = 0;
}

void Supervisor::setHaveIRSensor(int idx, byte val)
{
  robot.setHaveIrSensor(idx, val);
}

void Supervisor::updateSettings(SETTINGS settings)
{
  if (settings.sType == 0 || settings.sType == 5)
  {
    d_at_obs = settings.atObstacle;
    d_unsafe = settings.unsafe;
    d_fw = settings.dfw;
    // m_input.v = settings.velocity;
    robot.updateSettings(settings);
    m_FollowWall.d_fw = settings.dfw;
    m_SlidingMode.d_fw = settings.dfw;
  }

  if (settings.sType == 0 || settings.sType == 1 || settings.sType == 2)
  {
    robot.updatePID(settings);
    m_GoToGoal.updateSettings(settings);
    m_AvoidObstacle.updateSettings(settings);
    m_FollowWall.updateSettings(settings);
    // m_dkp = settings.kp;
    // m_dki = settings.ki;
    // m_dkd = settings.kd;
  }
  else if (settings.sType == 3)
  {
    m_pkp = settings.kp;
    m_pki = settings.ki;
    m_pkd = settings.kd;
    m_GoToGoal.setPID(3, m_pkp, m_pki, m_pkd); // updateSettings(settings);
    m_DiffCtrl.updateSettings(settings);    
  }
  else if (settings.sType == 4)
  {
    m_tkp = settings.kp;
    m_tki = settings.ki;
    m_tkd = settings.kd;
    m_GoToGoal.setPID(4, m_tkp, m_tki, m_tkd);
  }
}

SETTINGS Supervisor::getSettings(byte settingsType)
{
  SETTINGS settings;

  if (settingsType == 0 || settingsType == 5 || settingsType == 1 || settingsType == 2)
  {
    settings.sType = settingsType;

    settings.atObstacle = d_at_obs;
    settings.unsafe = d_unsafe;
    settings.dfw = d_fw;
    settings.max_w = robot.max_w;
    
    settings.velocity = m_input.v;
    settings.max_rpm = robot.max_rpm;
    settings.min_rpm = robot.min_rpm;

    settings.radius = robot.rl; //wheel_radius;
    settings.length = robot.wheel_base_length;
    SETTINGS pidSettings = robot.getPIDParams();
    settings.kp = pidSettings.kp;
    settings.ki = pidSettings.ki;
    settings.kd = pidSettings.kd;
  }

  else if (settingsType == 3) //position control pid
  {
    settings.sType = 3;
    settings.kp = m_pkp;
    settings.ki = m_pki;
    settings.kd = m_pkd;
  }
  else if (settingsType == 4) //theta control pid
  {
    settings.sType = 4;
    settings.kp = m_tkp;
    settings.ki = m_tki;
    settings.kd = m_tkd;
  }

  // m_GoToGoal.getSettings(&settings);

  return settings;
}

void Supervisor::init()
{
  SETTINGS settings = robot.getPIDParams();
  settings.sType = 1;
  m_GoToGoal.updateSettings(settings);
  m_AvoidObstacle.updateSettings(settings);
  m_FollowWall.updateSettings(settings);

  m_GoToGoal.setPID(3, m_pkp, m_pki, m_pkd);
  m_GoToGoal.setPID(4, m_tkp, m_tki, m_tkd);

  // m_DiffCtrl.updateSettings(settings);// updateSettings(settings);
}

void Supervisor::setGoal(double x, double y, int theta, double v)
{
  m_Goal.x = x;
  m_Goal.y = y;
  m_input.x_g = x;
  m_input.y_g = y;

  // if (theta <= 180)
  //   m_input.targetAngle = (theta * PI) / 180.0;
  // else
  // {
  //   theta = theta - 360;
  //   m_input.targetAngle = (theta * PI) / 180.0;
  // }
  
  m_input.targetAngle = theta;
   m_input.v = v;
//  robot.theta = 2*PI*theta/360;
}

void Supervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;

  d_prog = 20;
  m_GoToGoal.reset();
  m_AvoidObstacle.reset();
  m_FollowWall.reset();
  m_DiffCtrl.reset();

  m_FollowWall.dir = 0; //left

  m_state = S_GTG; //gotoGoal;
  m_currentController = &m_GoToGoal;

  progress_made = false;
  at_goal = false;
  at_obstacle = false;
  unsafe = false;
  danger = false;
}

void Supervisor::reset(long leftTicks, long rightTicks)
{

  // robot.x = 0;
  // robot.y = 0;
  // robot.theta = 0;

  d_prog = 20;
  m_GoToGoal.reset();
  m_AvoidObstacle.reset();
  m_FollowWall.reset();
  m_DiffCtrl.reset();

  m_FollowWall.dir = 0; //left

  m_state = S_GTG; //gotoGoal;
  m_currentController = &m_GoToGoal;

  progress_made = false;
  at_goal = false;
  at_obstacle = false;
  unsafe = false;
  danger = false;

  if (mSimulateMode)
  {
    m_left_ticks = 0;
    m_right_ticks = 0;
    robot.reset(m_left_ticks, m_right_ticks);
  }
  else
    robot.reset(leftTicks, rightTicks);
}

void Supervisor::setObstacleDistance(double dis[5])
{
  robot.setObstacleDistance(dis);
}

void Supervisor::setRobotPosition(double x, double y, double theta)
{
  robot.x = x;
  robot.y = y;
  robot.theta = theta;
}

void Supervisor::setSimulateMode(bool val)
{
  mSimulateMode = val;
  if (mSimulateMode)
  {
    for (int i = 0; i < 5; i++)
      setHaveIRSensor(i, false);
  }
}

void Supervisor::setIRFilter(bool open, float filter)
{
  robot.setIRFilter(open, filter);
}

void Supervisor::execute(long left_ticks, long right_ticks, double gyro, double dt)
{

  long startTime = micros();

  if (mSimulateMode)
    robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
  else
  {
    if (mUseIMU)
      robot.updateState(left_ticks, right_ticks, gyro, alpha, dt);
    else
      robot.updateState(left_ticks, right_ticks, dt);
  }

  if (m_state == S_STOP && at_goal)
    return;

  check_states();

  if (at_goal)
  {
    if (m_state != S_STOP)
      Serial.println("At Goal!");
    m_state = S_STOP; //s_stop;
    StopMotor();

    
  log("\nRP%d,%d,%d,%d,%d\n",
      (int)(1000 * robot.x),
      (int)(1000 * robot.y),
      (int)(1000 * robot.theta),
      (int)(1000 * robot.w),
      (int)(1000 * robot.velocity));

    return;
  }
  else if (!mSimulateMode && danger)
  {
    if (m_state != S_STOP)
      Serial.println("Danger!");
    m_state = S_STOP; //s_stop;
    StopMotor();
    return;
  }
  /////////////////////////////////////////////////////////
  executeAvoidAndGotoGoal(dt);

  if (m_currentController == NULL) //unsafe stoped
    return;

  m_output.v = 0;
  m_output.w = 0;

  m_currentController->execute(&robot, &m_input, &m_output, dt);

  Input in;
  in.v = m_output.v;
  in.w = m_output.w;

  m_DiffCtrl.execute(&robot, &in, &m_output, dt);
  
  int pwm_l = robot.vel_l_to_pwm(m_output.vel_l );
  int pwm_r = robot.vel_r_to_pwm(m_output.vel_r );


  // PWM_OUT pwm = robot.getPWMOut(m_output.v, m_output.w);

  if (mSimulateMode)
  {
    m_left_ticks = m_left_ticks + robot.pwm_to_ticks_l(pwm_l, dt);
    m_right_ticks = m_right_ticks + robot.pwm_to_ticks_r(pwm_r, dt);
  }
  else
  {
    MoveLeftMotor(pwm_l);
    MoveRightMotor(pwm_r);
  }

 log("RP%d,%d,%d,%d,%d\n",
      (int)(1000 * robot.x),
      (int)(1000 * robot.y),
      (int)(1000 * robot.theta),
      (int)(1000 * robot.w),
      (int)(1000 * robot.velocity));

  long etime = micros() - startTime;
  if (execTime < etime)
    execTime = etime;
}

void Supervisor::executeAvoidAndGotoGoal(double dt)
{

  if (m_state == S_STOP && !unsafe) // recover from stop
  {
    m_state = S_GTG; // gotoGoal;
    m_currentController = &m_GoToGoal;
    m_GoToGoal.reset();
  }

  if (m_state == S_GTG) // GTG state
  {
    if (at_obstacle)
    {

      bool ret = changeToFollowWall();

      if (ret)
      {
        mFollowWallPoint.x = robot.x;
        mFollowWallPoint.y = robot.y;
        log("To FLW From GTG %s, %s\n",
          floatToStr(0, robot.x),
          floatToStr(1, robot.y));
      }
      else //no follow wall condition
      {
        m_state = S_AVO;
        m_currentController = &m_AvoidObstacle;
        m_AvoidObstacle.reset();
        log("Change to AVO from gtg...\n");
      }
    }
  }
  else if (m_state == S_AVO)
  {

    // if (at_obstacle)
    // {

    //   bool ret = changeToFollowWall();
    //   if (ret)
    //     log("To FLW from AVO\n");
    // }

    bool ret = m_AvoidObstacle.beQuiteAvo();
    if (ret)
    {
      m_state = S_GTG; // gotoGoal;
      m_currentController = &m_GoToGoal;
      log("Change to GTG from avo ...\n");
      m_GoToGoal.reset();
    }
    else if (at_obstacle)
    {
      bool ret = changeToFollowWall();
      if (ret)
        log("To FLW from AVO\n");
    }
  }
  else
  { // follow wall
    m_SlidingMode.execute(&robot, &m_input, &m_output, 0.02);

    if (progress_made)
    {
      if (m_FollowWall.dir == 0 && m_SlidingMode.quitSlidingLeft()) // !m_SlidingMode.slidingLeft())
      {
        m_state = S_GTG; // gotoGoal;
        m_currentController = &m_GoToGoal;
        m_GoToGoal.reset();
        log("To GTG From FLW %s, %s\n",
          floatToStr(0, robot.x),
          floatToStr(1, robot.y));
      }
      else if (m_FollowWall.dir == 1 && m_SlidingMode.quitSlidingRight()) // !m_SlidingMode.slidingRight())
      {
        m_state = S_GTG; // gotoGoal;
        m_currentController = &m_GoToGoal;
        m_GoToGoal.reset();
        log("To GTG From FLW %s, %s\n",
          floatToStr(0, robot.x),
          floatToStr(1, robot.y));
      }
    }
      // else
      {
        
        Vector p0, p1;
        p0.x = robot.x;
        p0.y = robot.y;
        p1.x = robot.x + 0.2 * cos( robot.theta );
        p1.y = robot.y + 0.2 * sin( robot.theta );

        bool ret = doesVectorCross(p0, p1, mFollowWallPoint, m_Goal);
        if (ret == true)
        {
          m_state = S_GTG; // gotoGoal;
          m_GoToGoal.reset();
          m_currentController =  &m_GoToGoal;
        log("To GTG FLW C: %s, %s\n",
          floatToStr(0, robot.x),
          floatToStr(1, robot.y));
        }
      }

  }
}

bool Supervisor::changeToFollowWall()
{
  bool ret = false;

  m_SlidingMode.execute(&robot, &m_input, &m_output, 0.02);

  if (m_SlidingMode.slidingLeft())
  {
    m_FollowWall.dir = 0; //left
    m_currentController = &m_FollowWall;
    m_state = S_FLW; //followWall;
    m_FollowWall.reset();
    Serial.println("FLW-L");
    set_progress_point();

    return true;
  }
  else if (m_SlidingMode.slidingRight())
  {
    m_FollowWall.dir = 1; //right
    m_currentController = &m_FollowWall;
    m_state = S_FLW; //followWall;
    m_FollowWall.reset();
    Serial.println("FLW-R");
    set_progress_point();
    return true;
  }
  else //强制follow wall
  {
    m_FollowWall.dir = 0; //right
    m_currentController = &m_FollowWall;
    m_state = S_FLW; //followWall;
    m_FollowWall.reset();
    Serial.println("FLW-L F");
    set_progress_point();
    return true;
    // return false;
  }
}

void Supervisor::set_progress_point()
{
  double d = sqrt(sq(robot.x - m_Goal.x) + sq(robot.y - m_Goal.y));
  d_prog = d;
}

void Supervisor::check_states()
{
  double d = sqrt(sq(robot.x - m_Goal.x) + sq(robot.y - m_Goal.y));

  noObstacle = true;

  m_distanceToGoal = d;

  if (d < (d_prog - 0.3))
    progress_made = true;
  else
    progress_made = false;

  at_goal = false;
  if (d < d_stop)
  {
    if (abs(robot.theta - m_input.targetAngle ) < 0.05) //0.05
    {
      at_goal = true;
    }
    else
      at_goal = false;
    // at_goal = true;
  }

  at_obstacle = false;
  unsafe = false;

  IRSensor **irSensors = robot.getIRSensors();

  if( irSensors[2]->distance < d_at_obs )
    at_obstacle = true;
  else if( irSensors[1]->distance < d_fw || irSensors[3]->distance < d_fw )
    at_obstacle = true;

  // for (int i = 1; i < 4; i++)
  // {
  //   if (irSensors[i]->distance < d_at_obs)
  //     at_obstacle = true;
  //   if (irSensors[i]->distance < irSensors[i]->getMaxDistance() - 0.01) //  MAX_IRSENSOR_DIS)
  //     noObstacle = false;
  // }

  // if (noObstacle)
  // {
  //   if (irSensors[0]->distance < irSensors[0]->getMaxDistance() - 0.01)
  //     noObstacle = false;
  //   else if (irSensors[4]->distance < irSensors[4]->getMaxDistance() - 0.01)
  //     noObstacle = false;
  // }

  if (mIgnoreObstacle)
    at_obstacle = false;

  if (irSensors[1]->distance < d_unsafe || irSensors[2]->distance < d_unsafe || irSensors[3]->distance < d_unsafe)
    unsafe = true;

  danger = false;
  if (irSensors[1]->distance < d_unsafe && irSensors[2]->distance < d_unsafe && irSensors[3]->distance < d_unsafe)
    danger = true;
}

Position Supervisor::getRobotPosition()
{
  Position pos;
  pos.x = robot.x;
  pos.y = robot.y;
  pos.theta = robot.theta;
  return pos;
}

void Supervisor::getIRDistances(double dis[5])
{
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
}

void Supervisor::readIRDistances(double dis[5])
{
  robot.readIRSensors( 0 );
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
}