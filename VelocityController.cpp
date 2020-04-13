
#include "VelocityController.h"

VelocityController::VelocityController()
{
  Kp = 8;
  Ki = 0.01;
  Kd = 0.01;
  lastError = 0;
  lastErrorIntegration = 0;
  mTheta = 0;
  mW = 0;
  count = 0;
  curW = 0;
  keepTheta = false;
}

void VelocityController::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
  curW = 0;
  mTheta = 0;
}

void VelocityController::reset(Robot *robot)
{
  lastError = 0;
  lastErrorIntegration = 0;
  curW = 0;
  mTheta = robot->theta;
}

void VelocityController::setGoal(double v, double w, double curTheta )
{

  if (w == 0 && curW != 0) //remain the current theta; 加速过程中会有晃动；保留初始角度？
  {
    keepTheta = true;
    keepThetaTimer = (1 + 2 * abs(m_robot_w)) * 60;
    thetaPrevMillis = millis();
    mTheta = curTheta; //转弯结束，保留当前角度
  }
  curW = w;
  mW = w;

  //reset the controller ???
  // lastErrorIntegration = 0;
  // lastError = 0;

  m_v = v;
  m_w = w;

  if( m_w != 0  && m_v != 0 ) //转弯，控制速度？？
  {
    m_v = (0.1-abs(v))/3.14 * abs(m_w) + abs(v);
    if( v < 0 )
      m_v = -m_v;
  }
    // log("SG %s,%s: %s,%s\n",
    //     floatToStr(0, v),
    //     floatToStr(1, w),
    //     floatToStr(2, m_v),
    //     floatToStr(3, m_w));
}


void VelocityController::execute(Robot *robot, Input *input, Output *output, double dt)
{
  double e, e_I, e_D, w;

  m_robot_w = robot->w;

  if (mW != 0) //转弯，自由控制
  {
    output->v = m_v;
    output->w = m_w; 
    return;
  }

  if (keepTheta)
  {
    if (millis() - thetaPrevMillis > keepThetaTimer) //
    {
      keepTheta = false; //next circle to keep the theta??
      okToKeep = true;
    }
    else
    {
      e = 0;
      output->v = input->v;
      output->w = 0;
  //reset the controller ???
      lastErrorIntegration = 0;
      lastError = 0;

      return;
    }
  }

  if (okToKeep)
  {
    okToKeep = false;
    mTheta = robot->theta; // keep current direction
  }

  output->v = input->v;
  e = mTheta - robot->theta;
  e = atan2(sin(e), cos(e));
  e_D = (e - lastError) / dt;
  e_I = lastErrorIntegration + e * dt;
  output->w = Kp * e + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;
  lastError = e;
}
