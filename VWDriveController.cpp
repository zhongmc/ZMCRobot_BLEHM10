
#include "VWDriveController.h"

VWDriveController::VWDriveController()
{
  Kp = 0.3;
  Ki = 0.00;
  Kd = 0.005;
  lastError = 0;
  lastErrorIntegration = 0;

}

void VWDriveController::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
  m_w = 0;
  m_v = 0;
}

void VWDriveController::reset(Robot *robot)
{
  lastError = 0;
  lastErrorIntegration = 0;
  m_w = 0;
  m_v = 0;
}

void VWDriveController::setGoal(double v, double w, double curTheta )
{
  m_v = v;
  m_w = w;
}


void VWDriveController::execute(Robot *robot, Input *input, Output *output, double dt)
{
  double e, e_I, e_D, w;
  output->v = m_v;

  if( m_v == 0 ) //原地转弯
  {
    output->w = m_w; 
    return;
  }

  e = m_w - robot->w;
  e_D = (e - lastError) / dt;
  e_I = lastErrorIntegration + e * dt;
  output->w = m_w +  Kp * e + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;
  lastError = e;
}
