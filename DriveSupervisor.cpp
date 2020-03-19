
#include "DriveSupervisor.h"
#include "ZMCRobot.h"

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
  robot.setHaveIrSensor(2, false);
  robot.setHaveIrSensor(3, true);
  robot.setHaveIrSensor(4, true);

  mSimulateMode = false;
  mIgnoreObstacle = false;
  danger = false;
  mUseIMU = false;
  alpha = 0.5;
  m_left_ticks = 0;
  m_right_ticks = 0;
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
  m_DifController.setSettings(settings);
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
  m_Controller.setGoal(v, w);
}

void DriveSupervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.w = 0;
  m_Controller.setGoal(m_input.v, 0, 0);
  m_Controller.reset(&robot);
  m_DifController.reset();
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
    m_DifController.reset();
  }
  else
    robot.reset(leftTicks, rightTicks);
    m_Controller.reset(&robot);
    m_DifController.reset();
}

void DriveSupervisor::execute(long left_ticks, long right_ticks, double yaw, double dt)
{

  //  uint32_t timer = micros();

  if (mSimulateMode)
    robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
  else
  {
      robot.updateState(left_ticks, right_ticks, yaw, alpha, dt);
    // if (mUseIMU)
    //   robot.updateState(left_ticks, right_ticks, yaw, alpha, dt);
    // else
    //   robot.updateState(left_ticks, right_ticks, dt);
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

  m_Controller.execute(&robot, &m_input, &m_output, dt);

  Input in;
  in.v = m_output.v;
  in.w = m_output.w;

  m_DifController.execute(&robot, &in, &m_output, dt);
  
  int pwm_l = robot.vel_l_to_pwm(m_output.vel_l );
  int pwm_r = robot.vel_r_to_pwm(m_output.vel_r );




  // v = m_output.v;
  // w = m_output.w;
  // PWM_OUT pwm = robot.getPWMOut(v, w);


// #ifdef _DEBUG_
//   Serial.print(v);
//   Serial.print(",");
//   Serial.print(w);

//   Serial.print(",");
//   Serial.print(vel.vel_l);
//   Serial.print(",");
//   Serial.print(vel.vel_r);

//   Serial.print(",");
//   Serial.print(pwm.pwm_l);
//   Serial.print(",");
//   Serial.println(pwm.pwm_r);

// #endif

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

//  log("RP%d,%d,%d,%d,%d\n",
//       (int)(1000 * robot.x),
//       (int)(1000 * robot.y),
//       (int)(1000 * robot.theta),
//       (int)(1000 * robot.w),
//       (int)(1000 * robot.velocity));

  //   uint32_t nowMicros = micros();

  //     Serial.print(",");
  //   Serial.println( nowMicros - timer);
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
