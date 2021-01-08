
#include "Supervisor.h"
#include "ZMCRobot.h"
#include "utils.h"




void sendBleMessages(byte *tmp, uint8_t len );

//defined in motor.
extern byte *counterBuf;

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


  // SETTINGS settings = robot.getSettings();

  // d_at_obs = settings.atObstacle;
  // d_unsafe = settings.unsafe;
  // d_fw = settings.dfw;
  // m_FollowWall.d_fw = settings.dfw;
  // m_SlidingMode.d_fw = settings.dfw;


  mIgnoreObstacle = false;

  danger = false;
  execTime = 0;
}


//不区分settings类别，默认为包含全部参数的settings
void Supervisor::updateSettings(SETTINGS settings)
{
  d_at_obs = settings.atObstacle;
  d_unsafe = settings.unsafe;
  d_fw = settings.dfw;
  m_FollowWall.d_fw = settings.dfw;
  m_SlidingMode.d_fw = settings.dfw;

  m_GoToGoal.setSettings( settings );
  m_AvoidObstacle.setSettings( settings );
  m_FollowWall.setSettings( settings );
  m_DriveCtrl.setSettings( settings );

}


void Supervisor::setGoal(double x, double y, double theta, double v)
{
  m_Goal.x = x;
  m_Goal.y = y;
  m_input.x_g = x;
  m_input.y_g = y;


  m_input.targetTheta = theta;
  m_input.v = v;

  m_GoToGoal.reset();

//  robot.theta = 2*PI*theta/360;
}

void Supervisor::reset()
{

  d_prog = 20;
  m_GoToGoal.reset();
  m_AvoidObstacle.reset();
  m_FollowWall.reset();
  m_DriveCtrl.reset();

  m_FollowWall.dir = 0; //left
  m_state = S_GTG; //gotoGoal;
  m_currentController = &m_GoToGoal;
  progress_made = false;
  at_goal = false;
  at_obstacle = false;
  unsafe = false;
  danger = false;
}



Output Supervisor::execute(Robot *robot, double yaw, double dt)
{


  m_output.v = 0;
  m_output.w = 0;
  m_output.vel_l = 0;
  m_output.vel_r = 0;

  long startTime = micros();

  if (m_state == S_STOP && at_goal == true )
    return m_output;

  check_states(robot);

  if( at_goal == true )
  {
    if (m_state != S_STOP)
      Serial.println("At Goal!");
    m_state = S_STOP; //s_stop;
    at_goal = true;
    StopMotor();
  
  log("\nRP%d,%d,%d,%d,%d\n",
      (int)(1000 * robot->x),
      (int)(1000 * robot->y),
      (int)(1000 * robot->theta),
      (int)(1000 * robot->w),
      (int)(1000 * robot->velocity));

    return m_output;

  }

  if ( m_GoToGoal.isAtGoal( robot, &m_input ) )
  {
    if (m_state != S_STOP)
      Serial.println("At Goal!");
    m_state = S_STOP; //s_stop;
    at_goal = true;
    StopMotor();
  
  log("\nRP%d,%d,%d,%d,%d\n",
      (int)(1000 * robot->x),
      (int)(1000 * robot->y),
      (int)(1000 * robot->theta),
      (int)(1000 * robot->w),
      (int)(1000 * robot->velocity));

    return m_output;
  }
  else if ( danger)
  {
    if (m_state != S_STOP)
      Serial.println("Danger!");
    m_state = S_STOP; //s_stop;
    StopMotor();
    return m_output;
  }
  /////////////////////////////////////////////////////////
  executeAvoidAndGotoGoal(robot, dt);

  if (m_currentController == NULL) //unsafe stoped
    return m_output;


  m_currentController->execute(robot, &m_input, &m_output, dt);

  Input in;
  in.v = m_output.v;
  in.w = m_output.w;

  m_DriveCtrl.execute(robot, &in, &m_output, dt);
  
  long etime = micros() - startTime;
  if (execTime < etime)
    execTime = etime;

  return m_output;

}

void Supervisor::executeAvoidAndGotoGoal(Robot *robot, double dt)
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

      Serial.print("At OB:");
      IRSensor **irSensors = robot->getIRSensors();
        log("%s,%s,%s,%s,%s\n",
          floatToStr(0, irSensors[0]->distance),
          floatToStr(1, irSensors[1]->distance),
          floatToStr(2, irSensors[2]->distance),
          floatToStr(3, irSensors[3]->distance),
          floatToStr(4, irSensors[4]->distance));
      


      bool ret = changeToFollowWall(robot);

      if (ret)
      {
        mFollowWallPoint.x = robot->x;
        mFollowWallPoint.y = robot->y;
        log("To FLW From GTG %s, %s\n",
          floatToStr(0, robot->x),
          floatToStr(1, robot->y));
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
      bool ret = changeToFollowWall(robot);
      if (ret)
        log("To FLW from AVO\n");
    }
  }
  else
  { // follow wall
    m_SlidingMode.execute(robot, &m_input, &m_output, 0.02);

    if (progress_made)
    {
      if (m_FollowWall.dir == 0 && m_SlidingMode.quitSlidingLeft()) // !m_SlidingMode.slidingLeft())
      {
        m_state = S_GTG; // gotoGoal;
        m_currentController = &m_GoToGoal;
        m_GoToGoal.reset();
        log("To GTG From FLW %s, %s\n",
          floatToStr(0, robot->x),
          floatToStr(1, robot->y));
      }
      else if (m_FollowWall.dir == 1 && m_SlidingMode.quitSlidingRight()) // !m_SlidingMode.slidingRight())
      {
        m_state = S_GTG; // gotoGoal;
        m_currentController = &m_GoToGoal;
        m_GoToGoal.reset();
        log("To GTG From FLW %s, %s\n",
          floatToStr(0, robot->x),
          floatToStr(1, robot->y));
      }
    }
      // else
      {
        
        Vector p0, p1;
        p0.x = robot->x;
        p0.y = robot->y;
        p1.x = robot->x + 0.2 * cos( robot->theta );
        p1.y = robot->y + 0.2 * sin( robot->theta );

        bool ret = doesVectorCross(p0, p1, mFollowWallPoint, m_Goal);
        if (ret == true)
        {
          m_state = S_GTG; // gotoGoal;
          m_GoToGoal.reset();
          m_currentController =  &m_GoToGoal;
        log("To GTG FLW C: %s, %s\n",
          floatToStr(0, robot->x),
          floatToStr(1, robot->y));
        }
      }

  }
}

bool Supervisor::changeToFollowWall(Robot *robot)
{
  bool ret = false;

  m_SlidingMode.execute(robot, &m_input, &m_output, 0.02);

  if (m_SlidingMode.slidingLeft())
  {
    m_FollowWall.dir = 0; //left
    m_currentController = &m_FollowWall;
    m_state = S_FLW; //followWall;
    m_FollowWall.reset();
    Serial.println("FLW-L");
    set_progress_point(robot->x, robot->y);

    return true;
  }
  else if (m_SlidingMode.slidingRight())
  {
    m_FollowWall.dir = 1; //right
    m_currentController = &m_FollowWall;
    m_state = S_FLW; //followWall;
    m_FollowWall.reset();
    Serial.println("FLW-R");
    set_progress_point(robot->x, robot->y);
    return true;
  }
  else //强制follow wall
  {
    m_FollowWall.dir = 0; //right
    m_currentController = &m_FollowWall;
    m_state = S_FLW; //followWall;
    m_FollowWall.reset();
    Serial.println("FLW-L F");
    set_progress_point(robot->x, robot->y);
    return true;
    // return false;
  }
}

void Supervisor::set_progress_point(double x, double y)
{
  double d = sqrt(sq(x - m_Goal.x) + sq(y - m_Goal.y));
  d_prog = d;
}

void Supervisor::check_states(Robot *robot)
{
  double d = sqrt(sq(robot->x - m_Goal.x) + sq(robot->y - m_Goal.y));

  noObstacle = true;

  m_distanceToGoal = d;

  if (d < (d_prog - 0.3))
    progress_made = true;
  else
    progress_made = false;

  at_goal = false;
  if (d <= DISTANCE_GOAL )
  {
    if (abs(robot->theta - m_input.targetTheta ) < THETA_GOAL ) // min_vel w = 0.84 * 0.03 = 0.025;最小控制精度
    {
      at_goal = true;
    }
    else
      at_goal = false;
    // at_goal = true;
  }

  at_obstacle = false;
  unsafe = false;

  IRSensor **irSensors = robot->getIRSensors();

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

