
#include "DriveSupervisor.h"
#include "ZMCRobot.h"

void sendBleMessages(byte *tmp, uint8_t len );
void SendMessages(const char *format, ...);


#define REG2RAD 0.017453292
#define RAD2REG 57.2957795

DriveSupervisor::DriveSupervisor()
{
  d_unsafe = 0.11;

  m_input.x_g = 0;
  m_input.y_g = 0;
  m_input.v = 0.3;
  m_input.w = 0;
  ctrl_w = 0;
  ctrl_v = 0;
  target_theta = 0;
  keepTheta = false;
  //  robot.setVel2PwmParam(0, 6.4141, 14.924); // vel to pwm parameters
  //   robot.setVel2PwmParam(0,9.59,18.73);

  robot.setIRSensorType(GP2Y0A21);

  robot.setHaveIrSensor(0, true);
  robot.setHaveIrSensor(1, true);
  robot.setHaveIrSensor(2, true);
  robot.setHaveIrSensor(3, true);
  robot.setHaveIrSensor(4, true);

  mSimulateMode = false;
  mIgnoreObstacle = false;
  danger = false;
  // mUseIMU = false;
  // alpha = 0.5;
  m_left_ticks = 0;
  m_right_ticks = 0;
  inTurnState = false;

}

void DriveSupervisor::setIRFilter(bool open, float val)
{
  robot.setIRFilter(open, val);
}

void DriveSupervisor::setHaveIRSensor(int idx, byte val)
{
  robot.setHaveIrSensor(idx, val);
}


SETTINGS DriveSupervisor::getSettings()
{
  return robot.getSettings();
}


void DriveSupervisor::updateSettings(SETTINGS settings)
{
  if (settings.sType == 0 || settings.sType == 5)
  {
    d_unsafe = settings.unsafe;
  }
  robot.updateSettings(settings);

  init();

}

void DriveSupervisor::init()
{
  SETTINGS settings = robot.getSettings();
  m_Controller.setSettings(settings);
}

void DriveSupervisor::setPIDParams(int type, double kp, double ki, double kd )
{
    robot.setPIDParams(type, kp, ki, kd);
    init(); //update controller's PID
}


// drive the robot velocity and turning w
void DriveSupervisor::setGoal(double v, double w)
{

  if( ctrl_w != 0 && w == 0 )
  {
    keepTheta = true;
    keepThetaTime = millis();
  }
  ctrl_v = v;
  ctrl_w = w;

  m_input.v = v;
  m_input.w = w;
  // m_Controller.setGoal(v, w, robot.theta);
  inTurnState = false;
  m_state = s_DRIVE;
}


void DriveSupervisor::turnAround(int dir, int angle, bool useIMU, double yaw )
{
  // turnDir = dir;
  
  targetTurnTheta = angle* REG2RAD;
  turnedTheta = 0;
  inTurnState = true;
  ctrlTurnByIMU = useIMU;
  if( useIMU )
  {
    targetYaw = yaw + angle * REG2RAD;
    lastYaw = yaw;
  }
  else
  {
      targetYaw = robot.theta + angle* REG2RAD;
      lastYaw = robot.theta;
  }

  if( targetYaw > PI )
    targetYaw = targetYaw - 2*PI;

#define RAD2REG 57.2957795

  int ca = (int)(RAD2REG * lastYaw);
  int ta = (int)(RAD2REG * targetYaw);

  if( ca < 0 )
    ca+=360;
  if( ta < 0 )
    ta+= 360;

  SendMessages("-tl:%d,%d\n", ca, ta);

  // Serial.print("-TL:");
  // Serial.print(ca);
  // Serial.print(", ");
  // Serial.println( ta );

  m_state = S_TURN;

  double w = 0.5;
  switch( dir )
  {
      case 0:
        m_input.v = 0;
        m_input.w = w;
        break;
      case 1:
        m_input.v = 0.01;
        m_input.w = w;
        break;
      case 2:
        m_input.v = 0.01;
        m_input.w = -w;
        break;
      case 3:
        m_input.v = 0;
        m_input.w = -w;
        break;
      case 4:
        m_input.v = -0.01;
        m_input.w = w;
        break;
      case 5:
        m_input.v = -0.01;
        m_input.w = -w;
        break;
  }
  ctrl_v = m_input.v;
  ctrl_w = m_input.w;
  
}

void DriveSupervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.w = 0;
  robot.theta = 0;
  ctrl_v = 0;
  ctrl_w = 0;
  target_theta = 0;

  m_Controller.reset(&robot);
  // m_DifController.reset();
  reset(0, 0);
}

void DriveSupervisor::reset(long leftTicks, long rightTicks)
{

  danger = false;
  if (mSimulateMode)
  {
    m_left_ticks = 0;
    m_right_ticks = 0;
    robot.reset(m_left_ticks, m_right_ticks);
    m_Controller.reset(&robot);
    // m_DifController.reset();
  }
  else
  {
    robot.reset(leftTicks, rightTicks);
    m_Controller.reset(&robot);
  }
  ctrl_v = 0;
  ctrl_w = 0;
  target_theta = 0;

    // m_DifController.reset();
}


void DriveSupervisor::update(long left_ticks, long right_ticks, double dt)
{
  if (mSimulateMode)
    robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
  else
  {
      robot.updateState(left_ticks, right_ticks, dt);
  }
}

void DriveSupervisor::execute(long left_ticks, long right_ticks, double yaw, double dt)
{

  //  uint32_t timer = micros();

  double theta = robot.theta;

  if (mSimulateMode)
    robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
  else
  {
      robot.updateState(left_ticks, right_ticks, yaw, dt);
    // if (mUseIMU)
    //   robot.updateState(left_ticks, right_ticks, yaw, alpha, dt);
    // else
    //   robot.updateState(left_ticks, right_ticks, dt);
  }
  

  if( inTurnState )
  {
    double yawErr, delta_theta;

    if( ctrlTurnByIMU )
    {
      yawErr = yaw - targetYaw;
      delta_theta = yaw - lastYaw;
      lastYaw = yaw;
    }
    else
    {
      yawErr = robot.theta - targetYaw;
      delta_theta = robot.theta - theta;
      lastYaw = robot.theta;
    }

    delta_theta = atan2(sin(delta_theta), cos(delta_theta));
    turnedTheta = turnedTheta + abs(delta_theta);

    // if( abs(yawErr) < 0.09 && curYaw != startYaw )

    Serial.println(yawErr);

    //0.075 弧度的分辨率
    if( abs( turnedTheta - targetTurnTheta ) < 0.09 || turnedTheta >= targetTurnTheta )
    {

      inTurnState = false;
      StopMotor();
      m_state = S_STOP;

      // SendMessages("ta1:%d\n", turnedAngle);
      double endYaw = yaw;

      if( !mSimulateMode )
      {
        theta = robot.theta;
        delay(300);
        readCounter();
        robot.updateState(readLeftEncoder(), readRightEncoder(), yaw, dt);
        endYaw = robot.theta;
      }

      if( !ctrlTurnByIMU )
        endYaw = robot.theta;
      else
      {
        endYaw = yaw; //
      }
          
      int ta = (int)(RAD2REG*turnedTheta);
      int ea = (int)(RAD2REG*endYaw);
      int er = (int)(RAD2REG*yawErr);
      if( ta < 0 )
        ta+=360;
      if( ea < 0 )
        ea+= 360;

      Serial.println();
      SendMessages("-tle:%d,%d,%d\n",ta, ea,er);




      // SendMessages("ta2:%d\n", turnedAngle);
      // SendMessages("rt:%s\n", floatToStr(0, robot.theta ));

      // Serial.print("-turned:");
      // Serial.print( turnedTheta, 4);
      // Serial.print(":");
      // Serial.println(turnedAngle);

      // Serial.print("-turn end:");
      // Serial.print( robot.theta, 4);
      // Serial.print(":");
      // int curAngle = (int)(180.0*robot.theta / PI );
      // if( curAngle < 0 )
      //   curAngle = 360 + curAngle;
      // Serial.println(curAngle);
      return;
    }
  }

  check_states();

  if (!mSimulateMode && m_input.v > 0 && danger)
  {
    if (m_state != S_STOP)
      Serial.println("Danger!");
    m_state = S_STOP; //s_stop;
    StopMotor();
    return;
  }

  if( m_state == S_STOP )
    return;

  m_input.w = ctrl_w;

  if( keepTheta )
  {
    if( abs(robot.w) < 0.05 )
    {
      keepTheta = false;
      target_theta = robot.theta;

    }
    // if( millis() - keepThetaTime > 100 )
    // {
    //   keepTheta = false;
    //   target_theta = robot.theta;
    // }
  }

  if( ctrl_w == 0 && !keepTheta )
  {
     double e = target_theta - robot.theta;
     e = atan2(sin(e), cos(e));
      m_input.w = e;
  }
  
  m_Controller.execute(&robot, &m_input, &m_output, dt);
  
  int pwm_l = robot.vel_l_to_pwm(m_output.vel_l );
  int pwm_r = robot.vel_r_to_pwm(m_output.vel_r );


  if (mSimulateMode)
  {
    m_left_ticks = m_left_ticks + robot.pwm_to_ticks_l( pwm_l, dt);
    m_right_ticks = m_right_ticks + robot.pwm_to_ticks_r(pwm_r, dt);
  }
  else
  {
    MoveLeftMotor(pwm_l);
    MoveRightMotor(pwm_r);
  }

  byte *ctrl_info = m_Controller.getCtrlInfo();
  sendBleMessages(ctrl_info, 18);


}

extern double ultrasonicDistance;

void DriveSupervisor::check_states()
{

  IRSensor **irSensors = robot.getIRSensors();
  //    for( int i=0; i<5; i++)
  //    {
  //      if( irSensors[i]->distance < d_at_obs )
  //        at_obstacle = true;
  //      if( irSensors[i]->distance < d_unsafe )
  //        unsafe = true;
  //    }

  //if ( irSensors[1]->distance < d_unsafe || irSensors[2]->distance < d_unsafe || irSensors[3]->distance < d_unsafe )

  if (ultrasonicDistance < MAX_ULTRASONIC_DIS)
  {
    if (ultrasonicDistance < 0.05)
      danger = true;
    else
      danger = false;
    return;
  }

  if (irSensors[2]->distance < d_unsafe)
    danger = true;
  else
    danger = false;
}

void DriveSupervisor::setRobotPosition(double x, double y, double theta)
{
  robot.x = x;
  robot.y = y;
  robot.theta = theta;
  robot.prev_yaw = theta;
  target_theta = theta;


}

Position DriveSupervisor::getRobotPosition()
{
  Position pos;
  pos.x = robot.x;
  pos.y = robot.y;
  pos.theta = robot.theta;
  pos.v = robot.velocity;
  pos.w = robot.w;
  return pos;
}

void DriveSupervisor::getIRDistances(double dis[5])
{
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
}

void DriveSupervisor::getRobotVel(double dis[5])
{
  dis[0] = robot.vel_l;
  dis[1] = robot.vel_r;
}
