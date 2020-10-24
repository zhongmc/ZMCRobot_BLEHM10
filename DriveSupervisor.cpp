
#include "DriveSupervisor.h"
#include "ZMCRobot.h"

void sendBleMessages(byte *tmp, uint8_t len );
void SendMessages(const char *format, ...);

DriveSupervisor::DriveSupervisor()
{
  d_unsafe = 0.11;

  m_input.x_g = 0;
  m_input.y_g = 0;
  m_input.v = 0.3;
  m_input.w = 0;

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
  m_input.v = v;
  m_input.w = w;
  // m_Controller.setGoal(v, w, robot.theta);
  inTurnState = false;
  m_state = s_DRIVE;
}


void DriveSupervisor::turnAround(int dir, int angle )
{
  turnDir = dir;
  turnAngle = angle;
  turnedTheta = 0;
  inTurnState = true;
  // m_DifController.reset();
  Serial.print("-turn around:");
  Serial.print( dir );
  Serial.print(", ");
  Serial.println( angle );
  Serial.print("-cur theta:");
  Serial.print( robot.theta, 4);
  Serial.print(":");
  int curAngle = (int)(180.0*robot.theta / PI );
  if( curAngle < 0 )
    curAngle = 360 + curAngle;
  Serial.println(curAngle);
  m_state = S_TURN;

  double w = 0.5;
  switch( turnDir )
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
}

void DriveSupervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.w = 0;
  robot.theta = 0;
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
    robot.reset(leftTicks, rightTicks);
    m_Controller.reset(&robot);
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
    double delta_theta = robot.theta - theta;
    delta_theta = atan2(sin(delta_theta), cos(delta_theta));
    turnedTheta = turnedTheta + abs(delta_theta);
    int turnedAngle = (int)(180.0*turnedTheta / PI );
    if( abs(turnedAngle - turnAngle) < 5  || turnedAngle >= turnAngle )
    {

      inTurnState = false;
      StopMotor();
      m_state = S_STOP;
      SendMessages("ta1:%d\n", turnedAngle);
      if( !mSimulateMode )
      {
        theta = robot.theta;
        delay(300);
        robot.updateState(readLeftEncoder(), readRightEncoder(), yaw, dt);
        delta_theta = robot.theta - theta;
        delta_theta = atan2(sin(delta_theta), cos(delta_theta));
        turnedTheta = turnedTheta + abs(delta_theta);
        turnedAngle = (int)(180.0*turnedTheta / PI );
      }

      SendMessages("ta2:%d\n", turnedAngle);
      SendMessages("rt:%s\n", floatToStr(0, robot.theta ));

      Serial.print("-turned:");
      Serial.print( turnedTheta, 4);
      Serial.print(":");
      Serial.println(turnedAngle);

      Serial.print("-turn end:");
      Serial.print( robot.theta, 4);
      Serial.print(":");
      int curAngle = (int)(180.0*robot.theta / PI );
      if( curAngle < 0 )
        curAngle = 360 + curAngle;
      Serial.println(curAngle);
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
