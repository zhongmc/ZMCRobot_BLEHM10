
#include "VWDriveController.h"

void floatToByte(byte *arrayBuf, double val, double scale);



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
  lastVError = 0;
  lastVErrorIntegration = 0;
  m_w = 0;
  m_v = 0;
}

// void VWDriveController::setGoal(double v, double w, double curTheta )
// {
//   m_v = v;
//   m_w = w;
// }


void VWDriveController::execute(Robot *robot, Input *input, Output *output, double dt)
{

  m_v = input->v;
  m_w = input->w;

  doWControll(robot, input, output, dt);
  doVControll(robot, input, output, dt );

      vel_l =  output->vel_l;
      vel_r = output->vel_r; 

  ctrl_info[0] = 0xA3;
  ctrl_info[1] = 16;
  floatToByte(ctrl_info + 2, robot->velocity, 1000);
  floatToByte(ctrl_info + 4, robot->w, 1000);
  floatToByte(ctrl_info + 6, m_v, 1000);
  floatToByte(ctrl_info + 8, m_w, 1000);
  floatToByte(ctrl_info + 10, ctrl_v, 1000);
  floatToByte(ctrl_info + 12, ctrl_w, 1000);
  floatToByte(ctrl_info + 14, vel_l, 1000);
  floatToByte(ctrl_info + 16, vel_r, 1000);

}


  void VWDriveController::doWControll(Robot *robot, Input *input, Output *output, double dt)
  {
      double e, e_I, e_D, w;
        output->v = m_v;
      if( m_v == 0 ) //原地转弯
      {
        lastErrorIntegration = 0;
        lastError = 0;
        output->w = m_w; 
        return;
      }

      if( abs(m_w) > 0.3 )
      {
        lastErrorIntegration = 0;
        lastError = 0;
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



  void VWDriveController::doVControll(Robot *robot, Input *input, Output *output, double dt)
  {
 
    ctrl_v = output->v;
    ctrl_w = output->w;
    Vel vel = robot->uni_to_diff_oneside(ctrl_v, ctrl_w); //uni_to_diff_v(sv, sw);
    if( abs(m_v) <= 0.001 )
    {

      double dif = robot->vel_l + robot->vel_r;  //控制左右一致的转速
      if( dif * vel.vel_l > 0 ) //1. vel_l < 0 dif <0 vel_r = vel_r - dif; 2. vel_l > 0 dif > 0 vel_r - dif
      {
        vel.vel_r = vel.vel_r - dif;
      }
      else // 1. vel_l < 0 dif > 0 vel_l - dif 2. vel_l > 0 dif < 0 vel_l - dif
      {
        vel.vel_l = vel.vel_l - dif;
      }
      
      output->vel_l = vel.vel_l;
      output->vel_r = vel.vel_r;
      lastVError = 0;
      lastVErrorIntegration = 0;
      return;
    }


  if( abs(ctrl_w) > 0.2 ) //拐弯，不做速度控制
  {
    output->vel_l = vel.vel_l;
    output->vel_r = vel.vel_r;
    // lastError = 0;
    // lastErrorIntegration = 0;
  }
  else
  {
      double e, ei,ed;
      e = input->v - robot->velocity;
      ei = lastVErrorIntegration + e* dt;
      ed = (e-lastVError)/dt;

      ctrl_v = vkp * e + vki*ei + vkd*ed;
      lastVErrorIntegration = ei;
      lastVError = e;

      ctrl_v = input->v + vkp * e;
      if( ctrl_v * m_v < 0 ) //不能倒车的方式减速
        ctrl_v = 0;
      vel = robot->uni_to_diff_oneside(ctrl_v, ctrl_w); //uni_to_diff_v(sv, sw);
      output->vel_l = vel.vel_l;
      output->vel_r = vel.vel_r; 
  }
      

  }

