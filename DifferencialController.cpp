
#include "DifferencialController.h"

DifferencialController::DifferencialController()
{
  Kp = 0.4;
  Ki = 0.6;
  Kd = 0.00;
  
  lastError = 0;
  lastErrorIntegration = 0;

  lastError1 = 0;
  lastErrorIntegration1 = 0;
  lastV = 0;
}

void DifferencialController::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;

  lastError1 = 0;
  lastErrorIntegration1 = 0;
  lastV = 0;
  
}


void DifferencialController::execute(Robot *robot, Input *input, Output *output, double dt)
{

    double sv = input->v;
    double sw = input->w;

    // Serial.print(sv);
    // Serial.print(',');
    // Serial.print(sw);
    // Serial.print(',');

    double max_w = robot->max_w;

		if (sw > max_w)   //限制拐弯时的转速
		  sw = max_w;  
		else if (sw < -max_w)
		  sw = -max_w;

    if( sv != 0 )  //拐弯减速
    {
      double av = abs(sv);
      av = (0.08-av)/max_w * abs(sw) + av;
      if( av <= 0 )
        av = 0.01;

      if( sv < 0  )
        av = -av;
      
      sv = av;
    }

  Vel vel = robot->uni_to_diff(sv, sw);

//限定拐弯时，不出现左右反转
	  if( sv != 0 && vel.vel_l *vel.vel_r < 0)
	  {
      if( abs(vel.vel_l) > abs(vel.vel_r ) )
      vel.vel_r = 0;
      else
      {
        vel.vel_l = 0;
      }
	  } 

  if( sv == 0 && lastV != 0 )
  {
    lastError = 0;
    lastErrorIntegration = 0;
    lastError1 = 0;
    lastErrorIntegration1 = 0;

    if( Ki != 0) //直接初始化转圈？？？
    {
      lastErrorIntegration = vel.vel_l / Ki;
      lastErrorIntegration1 = vel.vel_r /Ki;
    }
    output->vel_l = vel.vel_l;
    output->vel_r = vel.vel_r;

    lastV = sv;

    // Serial.print(output->vel_l);
    // Serial.print(',');
    // Serial.println(output->vel_r);

    return;

  }

  double e1, e2, ei1, ei2, ed1,ed2;
  
  e1 = vel.vel_l - robot->vel_l;
  e2 = vel.vel_r - robot->vel_r;

  ei1 = lastErrorIntegration + e1 * dt;
  ei2 = lastErrorIntegration1 + e2 * dt;
  
	  if( abs(ei1)*Ki > robot->max_vel  )
		  ei1 = ei1/3; 
	  if( abs(ei2)*Ki > robot->max_vel  )
		  ei2 = ei2/3; 

  ed1 = (e1-lastError)/dt;
  ed2 = (e2-lastError1)/dt;

  lastError = e1;
  lastError1 = e2;

  double vel_l = Kp * e1 + Ki * ei1 + Kd * ed1;
  double vel_r = Kp * e2 + Ki * ei2 + Kd * ed2;

  // Serial.print(e1, 3);
  // Serial.print(',');
  // Serial.print(vel_l, 3);
  // Serial.print(',');
  
  vel_l = robot->normalizeVel(vel.vel_l, vel_l );
  vel_r = robot->normalizeVel(vel.vel_r, vel_r );

  if( sv != 0 && vel_l * vel_r < 0)
  {
    if( abs(vel_l) > abs(vel_r ) )
      vel_r = 0;
    else
    {
      vel_l = 0;
    }
    
  }

  // Serial.println(vel_l, 3);

  lastErrorIntegration = ei1;
  lastErrorIntegration1 = ei2;

  output->vel_l = vel_l;
  output->vel_r = vel_r;

    // Serial.print(output->vel_l);
    // Serial.print(',');
    // Serial.println(output->vel_r);

  lastV = sv;
  return;
}

