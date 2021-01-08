
#include "DriveSupervisor.h"
#include "ZMCRobot.h"

void sendBleMessages(byte *tmp, uint8_t len );
void SendMessages(const char *format, ...);

extern long count1, count2;
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
  mIgnoreObstacle = false;
  danger = false;
  inTurnState = false;

}



void DriveSupervisor::updateSettings(SETTINGS settings)
{
  d_unsafe = settings.unsafe;

  lastError = 0;
  lastErrorIntegration = 0;
  lastVError = 0;
  lastVErrorIntegration = 0;

  kp = settings.kp;
  ki = settings.ki;
  kd = settings.kd;

  vkp = settings.dkp;
  vki = settings.dki;
  vkd = settings.dkd;

}

// drive the robot velocity and turning w
void DriveSupervisor::setGoal(double v, double w)
{

  inTurnState = false;
  m_state = s_DRIVE;
  m_input.v = v;
  m_input.w = w;
  if( v == 0 && w == 0 )  //stop
  {
    lastError = 0;
    lastErrorIntegration = 0;
    lastVError = 0;
    lastVErrorIntegration = 0;

      m_state == S_STOP;
  }

  if( ctrl_w != 0 && w == 0 )
  {
    keepTheta = true;
  }
  ctrl_v = v;
  ctrl_w = w;
}

void DriveSupervisor::turnAround(int dir, int angle, double w,  double theta, double yaw )
{

  targetTurnTheta = angle* REG2RAD;
  turnedTheta = 0;
  inTurnState = true;
  ctrlTurnByIMU = false;
  
  lastTheta = theta;

  targetYaw = theta + angle* REG2RAD;

  if( targetYaw > PI )
    targetYaw = targetYaw - 2*PI;

// #define RAD2REG 57.2957795
  int ct = (int)(RAD2REG * theta );
  int cy = (int)(RAD2REG * yaw);

  if( ct < 0 )
    ct+=360;

  if( cy < 0 )
    cy+=360;

  SendMessages("-tl:%d,%d;\n", ct, cy );

  m_state = S_TURN;
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

void DriveSupervisor::turnAround(int dir, int angle, bool useIMU, double theta, double yaw )
{
  // turnDir = dir;
  turnAround(dir, angle, 0.8, theta, yaw );

}

void DriveSupervisor::reset()
{
  ctrl_v = 0;
  ctrl_w = 0;
  target_theta = 0;
  danger = false;

    lastError = 0;
    lastErrorIntegration = 0;
    lastVError = 0;
    lastVErrorIntegration = 0;

      m_state == S_STOP;

}


void DriveSupervisor::reset(double theta)
{
  reset();
  target_theta = theta;
}


Output DriveSupervisor::execute(Robot *robot, double yaw, double dt)
{
 //  uint32_t timer = micros();
 // double theta = robot->theta;
  m_output.v = 0;
  m_output.w = 0;
  m_output.vel_l = 0;
  m_output.vel_r = 0;

  if( inTurnState )
  {
    double yawErr, delta_theta;

      yawErr = robot->theta - targetYaw;
      delta_theta = robot->theta - lastTheta;
      lastTheta = robot->theta;

    delta_theta = atan2(sin(delta_theta), cos(delta_theta));
    delta_theta = abs(delta_theta);

    if( delta_theta > 0.78 )
      delta_theta = 0; ////////////
    turnedTheta = turnedTheta + abs(delta_theta);

    // if( abs(yawErr) < 0.09 && curYaw != startYaw )
    // Serial.println(yawErr);
    //0.075 弧度的分辨率
    if( abs( turnedTheta - targetTurnTheta ) < 0.09 || turnedTheta >= targetTurnTheta )
    {

      inTurnState = false;
      StopMotor();
      m_state = S_STOP;
      // SendMessages("ta1:%d\n", turnedAngle);
      double endYaw = yaw;

        // theta = robot->theta;
        delay(500);
        readCounter();
        robot->updateState(readLeftEncoder(), readRightEncoder(), yaw, dt);
        endYaw = robot->theta;

      if( !ctrlTurnByIMU )
        endYaw = robot->theta;
      else
      {
        endYaw = yaw; //
      }

      int ct = (int)(RAD2REG * robot->theta);   
      int ta = (int)(RAD2REG*turnedTheta);
      int cy = (int)(RAD2REG*yaw);

      if( ct < 0 )
        ct+=360;
      if( ta < 0 )
        ta+=360;
      if( cy < 0 )
        cy+= 360;

      Serial.println();
      SendMessages("-tle:%d,%d,%d;\n",ct, cy, ta);

      return m_output;
    }
  }

  check_states(robot);

  if ( m_input.v > 0 && danger)
  {
    if (m_state != S_STOP)
      Serial.println("Danger!");
    m_state = S_STOP; //s_stop;
    StopMotor();
    return m_output;
  }

  if( m_state == S_STOP )
    return m_output;

  m_input.w = ctrl_w;

  if( keepTheta )
  {
    if( abs(robot->w) < 0.05 )
    {
      keepTheta = false;
      target_theta = robot->theta;
      lastError = 0;
      lastErrorIntegration = 0;
    }
  }

  if( ctrl_w == 0 && !keepTheta )  //方向控制
  {
     double e = target_theta - robot->theta;
     e = atan2(sin(e), cos(e));

    double e_D = (e - lastError) / dt;
    double e_I = lastErrorIntegration + e * dt;
    m_input.w = kp * e + ki * e_I + kd * e_D;
    lastError = e;
  }

  doVControll( robot, dt);
  return m_output;

}


void DriveSupervisor::doVControll(Robot *robot, double dt)
{
    double v = m_input.v;
    double w = m_input.w;

    m_output.v = v;
    m_output.w = w;
    
    Vel vel = robot->uni_to_diff_velmin(v, w); 
    if( v == 0 )
    {

      double dif =0.5*( robot->vel_l + robot->vel_r);  //控制左右一致的转速
      if( dif * vel.vel_l > 0 ) //1. vel_l < 0 dif <0 vel_r = vel_r - dif; 2. vel_l > 0 dif > 0 vel_r - dif
      {
        vel.vel_r = vel.vel_r - dif;
      }
      else // 1. vel_l < 0 dif > 0 vel_l - dif 2. vel_l > 0 dif < 0 vel_l - dif
      {
        vel.vel_l = vel.vel_l - dif;
      }
      
      m_output.vel_l = vel.vel_l;
      m_output.vel_r = vel.vel_r;
      lastVError = 0;
      lastVErrorIntegration = 0;
      return;
    }


  if( abs( w) > 0.2 ) //拐弯，不做速度控制
  {
    m_output.vel_l = vel.vel_l;
    m_output.vel_r = vel.vel_r;
    // lastError = 0;
    // lastErrorIntegration = 0;
  }
  else
  {
      double e, ei,ed;
      e = v - robot->velocity;
      ei = lastVErrorIntegration + e* dt;
      ed = (e-lastVError)/dt;

      double cv = vkp * e + vki*ei + vkd*ed;
      lastVErrorIntegration = ei;
      lastVError = e;

      cv = v + vkp * e;
      if( cv * v < 0 ) //不能倒车的方式减速
        cv = 0;
      // vel = robot->uni_to_diff_oneside(ctrl_v, ctrl_w); 
      // vel = robot->ensure_w(ctrl_v, ctrl_w); 
      m_output.v = cv;
      vel = robot->uni_to_diff_velmin(cv, w); 
      m_output.vel_l = vel.vel_l;
      m_output.vel_r = vel.vel_r; 
  }
      


}



extern double ultrasonicDistance;

void DriveSupervisor::check_states(Robot *robot)
{

  IRSensor **irSensors = robot->getIRSensors();
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

