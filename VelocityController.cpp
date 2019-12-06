
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
  m_robot_w = 0;
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

void VelocityController::setGoal(double v, double w)
{

  if (w == 0 && curW != 0) //remain the current theta; 加速过程中会有晃动；保留初始角度？
  {
    keepTheta = true;
    keepThetaTimer = (1 + 2 * abs(m_robot_w)) * 30;
    log("kT %d,%s\n", keepThetaTimer, floatToStr(0, m_robot_w));
    thetaPrevMillis = millis();
    //    mTheta = robot.theta; //转弯结束，保留当前角度
  }
  curW = w;
  mW = w;

  //reset the controller ???
  lastErrorIntegration = 0;
  lastError = 0;
  m_v = v;
  m_w = w;

    log("SG %s,%s: %s,%s\n",
        floatToStr(0, v),
        floatToStr(1, w),
        floatToStr(2, m_v),
        floatToStr(3, m_w));
}

//depricated
void VelocityController::setGoal(double v, double theta, double curTheta)
{

  mTheta = curTheta;
  mW = theta;

  if (mW == 0)
  {
    Serial.println("zero mw!");
    lastErrorIntegration = 0;
    lastError = 0;
  }
}

void VelocityController::execute(Robot *robot, Input *input, Output *output, double dt)
{
  double e, e_I, e_D, w;

  m_robot_w = robot->w;

  if (mW != 0) //转弯，控制角速度？
  {

    output->v = m_v; // input->v / (1 + abs(robot->w) / 3); //转弯，控制速度
    output->w = m_w; //2 * mW;

    // output->v = input->v;

    // e = mW - robot->w;

    // e_I = lastErrorIntegration + e * dt;
    // e_D = (e - lastError) / dt;
    // w = Kp * e + Kd * e_D + Ki * e_I;

    // lastErrorIntegration = e_I;
    // if (abs(lastErrorIntegration) > 10)
    //   lastErrorIntegration = 0;

    // output->w = w;
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
      return;
    }
  }

  if (okToKeep)
  {
    okToKeep = false;
    mTheta = robot->theta; // keep current direction
    lastErrorIntegration = 0;
    lastError = 0;
  }

  e = mTheta - robot->theta;
  e = atan2(sin(e), cos(e));

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

  e_D = (e - lastError) / dt;
  w = p * e + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;

  // if (abs(lastErrorIntegration) > 30)
  //   lastErrorIntegration = 0;

  // count++;
  // if (count > 2)
  // {
  //   // Serial.print(input->v);
  //   // Serial.print(", ");
  //   Serial.print(e);
  //   Serial.print(", ");
  //   Serial.println(w);
  //   count = 0;
  // }

  // Serial.print(mTheta);
  // Serial.print(",");
  // Serial.print(robot->theta);
  // Serial.print(",");
  // Serial.print(e);
  // Serial.print(",");
  // Serial.println(w);

#ifdef _DEBUG_
  Serial.print(e);
  Serial.print(",");
  Serial.print(w);
  Serial.print(",");
#endif

  output->v = input->v;
  output->w = w;
  lastError = e;
}
