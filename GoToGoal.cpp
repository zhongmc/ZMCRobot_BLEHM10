#include "GoToGoal.h"

GoToGoal::GoToGoal()
{
  Kp = 5;
  Ki = 0.06;
  Kd = 0.01;
  lastError = 0;
  lastErrorIntegration = 0;
  lastVE = 0;
  lastVEI = 0;
  lastTE = 0;
  lastTEI = 0;

  state = 0;
}

void GoToGoal::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
  lastVE = 0;
  lastVEI = 0;
  lastTE = 0;
  lastTEI = 0;
  state = 0;
}



void GoToGoal::execute(Robot *robot, Input *input, Output *output, double dt)
{

  double u_x, u_y, e, e_I, e_D, w, theta_g;
  u_x = input->x_g - robot->x;
  u_y = input->y_g - robot->y;

  theta_g = atan2(u_y, u_x);
  double d = sqrt(pow(u_x, 2) + pow(u_y, 2));

  if( state == 1)
  {
    output->v = 0;
    theta_g = input->targetTheta;
    e = theta_g - robot->theta;
    e = atan2(sin(e), cos(e));

    if( abs(e) < THETA_GOAL )
    {
      StopMotor();
      output->w = 0;
      lastErrorIntegration = e_I;
      lastError = e;
      state = 0;
      return;
    }

    output->w = e/2;
    return;
  }

  if( state == 2)
  {
    output->v = 0;
    e = theta_g - robot->theta;
    e = atan2(sin(e), cos(e));
    if( abs(e) > 0.1 )
    {
      output->w = e;
      return;
    }
  
    lastErrorIntegration = e_I;
    lastError = e;
    state = 0;
  }

  output->v = input->v - (input->v - robot->velocity )/3; //
  e = theta_g - robot->theta;
  e = atan2(sin(e), cos(e));
  e_D = (e - lastError) / dt;
  e_I = lastErrorIntegration + e * dt;
  output->w = Kp * e + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;
  lastError = e;
  //no need of pid; let vw control do it
  // output->w = e;

  if( abs(e) > 0.5 )
  {
    state = 2;
    output->v = 0;
    return;
  }

  if( d < 0.3 && d > 0.03)
  {
    output->v = d *(input->v - 0.05)/0.3 + 0.05;
    if( output->v < 0 )
      output->v = 0;
  }

  if( d > DISTANCE_GOAL )
    return;  

  // robot->stopMotor(); //brake
  StopMotor();
  Serial.println("Theta ctrl ...");
  state = 1;
//now theta controll
  output->v = 0;
  output->w = 0;
  // theta_g = input->targetTheta;
  // e = theta_g - robot->theta;
  // e = atan2(sin(e), cos(e));
  lastError = 0;
  lastErrorIntegration = 0;

}


bool GoToGoal::isAtGoal(Robot *robot, Input *input )
{
  double u_x = input->x_g - robot->x;
  double u_y = input->y_g - robot->y;
  double d = sqrt(pow(u_x, 2) + pow(u_y, 2));

  if( d <= DISTANCE_GOAL)
  {
    if (abs(robot->theta - input->targetTheta ) < 0.06) // min_vel w = 1.0 * 0.03 = 0.03;最小控制精度
    {
      return true;
    }
  }
  return false;
}


/**
void GoToGoal::execute(Robot *robot, Input *input, Output *output, double dt)
{

  double u_x, u_y, e, e_I, e_D, w, theta_g;
  u_x = input->x_g - robot->x;
  u_y = input->y_g - robot->y;

  theta_g = atan2(u_y, u_x);
  double d = sqrt(pow(u_x, 2) + pow(u_y, 2));

  output->v = input->v;

  // state 0: normal, 1: d control 2: theta control
  if (d >= 0.3)
  {
    if (state != 0)
    {
      state = 0;
      lastError = 0;
      lastErrorIntegration = 0;
      Serial.println("CHG to Normal GTG...");
    }
  }
  else if (d < 0.3 && d > 0.02)
  {
    if (state == 2 && d > 0.03)
    {
      Serial.println("CHG to D Ctrl 1...");
      state = 1;         // change to d cntrol
    }
    else if (state == 0) // if d small enougth, stay theta control
    {
      Serial.println("CHG to D Ctrl 2 ...");
      state = 1;
    }
  }

  if (d <= 0.02)  // 0.15的速度，0.03的控制周期，最小精度 0.0045；
  {
    if (state != 2)
    {
      Serial.println("CHG to Theta Ctrl...");
      state = 2;
      lastErrorIntegration = 0;
      lastError = 0;
    }
  }

  if (state == 2)
    theta_g = input->targetTheta;

  e = theta_g - robot->theta;
  e = atan2(sin(e), cos(e));
  e_D = (e - lastError) / dt;

  if (state == 0 || state == 1) //1 D 控制时，保留角度控制，保证方向
  {
    double p = Kp;
    double ae = abs(e);
    if (ae > 1)
    {
      p = p / 2;
      e_I = 0;
    }
    else
    {
      e_I = lastErrorIntegration + e * dt;
    }
    if (ae > 2)
      p = p / 3;

    w = p * e + Ki * e_I + Kd * e_D;
    lastErrorIntegration = e_I;
    lastError = e;
    output->v = input->v; // / (1 + abs(robot->w) / 2);
    output->w = w;

      // Serial.print(e, 3);
      // Serial.write(',');
      // Serial.print(e_I, 3);
      // Serial.write(',');
      // Serial.print(w, 3);
      // Serial.write(',');

  }
  // else
  if (state == 1) //距离控制
  {
    output->v = d *(input->v - 0.9)/0.3 + 0.9; // d*(input->v - 0.05)/0.3 + 0.05;  //速度控制在v - 0.9间？ 
  }
  else if (state == 2)
  {
    // double p = tkp;
    // if (abs(e) > 1)
    // {
    //   p = p / 2;
    //   e_I = 0;
    // }
    // else
    // {
    //   e_I = lastErrorIntegration + e * dt;
    // }
    // if (abs(e) > 2)
    //   p = p / 3;

    // w = p * e + tki * e_I + tkd * e_D;
    // // log.info(String.format("T: %.3f, %.3f, %.3f", d, e, w));

    // lastErrorIntegration = e_I;
    // lastError = e;

    lastErrorIntegration = 0;
    lastError = e;


    w = 0.095 * abs( e ) + 0.9; //1.5;  // w 1.2 - 0.9; e 0 - pi
    if( abs(e) < 0.06 ) //0.05*1.2 一个控制周期的最小转角？ 
      w = 0;

    if( e < 0 )
      w =-w;
    output->w = w; //w;
    output->v = 0;
  }
}

*/





// void GoToGoal::execute(Robot *robot, Input *input, Output *output, double dt)
// {
//   double u_x, u_y, e, e_I, e_D, w, theta_g;

//   u_x = input->x_g - robot->x;
//   u_y = input->y_g - robot->y;
//   theta_g = atan2(u_y, u_x);

//   e = theta_g - robot->theta;
//   e = atan2(sin(e), cos(e));
//   e_I = lastErrorIntegration + e * dt;
//   e_D = (e - lastError) / dt;
//   w = Kp * e + Ki * e_I + Kd * e_D;
//   lastErrorIntegration = e_I;

// #ifdef _DEBUG_

//   Serial.print(",");
//   Serial.print(e);
//   Serial.print(",");
//   Serial.print(w);

// #endif

//   double d = sqrt(pow(u_x, 2) + pow(u_y, 2));
//   output->v = input->v;

//   if (d < 0.03) //slow down
//   {
//     output->v = 0;
//   }
//   else if (d < 0.5) //target controll
//   {
//     double vei = lastVEI + d * dt;
//     double ved = (d - lastVE) / dt;
//     output->v = Kp * d / 20.0 + Kd * ved; // 不能有超调
//     lastVEI = vei;
//     lastVE = d;
//   }

//   output->w = w;
//   lastError = e;

//   // Serial.print(output->v);
//   // Serial.print(",");
//   // Serial.print(e);
//   // Serial.print(",");
//   // Serial.println(w);
//   // Serial.print(",");
// }
