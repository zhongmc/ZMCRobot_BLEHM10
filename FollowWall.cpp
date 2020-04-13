#include "FollowWall.h"

FollowWall::FollowWall()
{
  Kp = 5;
  Ki = 0.01;
  Kd = 0.1;
  lastError = 0;
  lastErrorIntegration = 0;
}

void FollowWall::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;
}

void FollowWall::execute(Robot *robot, Input *input, Output *output, double dt)
{

  getWall(robot);

  u_fw_t.x = p0.x - p1.x;
  u_fw_t.y = p0.y - p1.y;

  //u_fw_tp = u_fw_t/norm(u_fw_t);
  double norm_ufwt = sqrt(u_fw_t.x * u_fw_t.x + u_fw_t.y * u_fw_t.y);
  Vector u_fw_tp;
  u_fw_tp.x = u_fw_t.x / norm_ufwt;
  u_fw_tp.y = u_fw_t.y / norm_ufwt;

  Vector u_fw_p;
  //   u_fw_p = ((u_a-u_p)-((u_a-u_p)'*u_fw_tp)*u_fw_tp);
  // u_a = p_1;  u_p = [x;y]; u_a_p = p_1 - u_p
  Vector u_a_p;
  u_a_p.x = p1.x - robot->x;
  u_a_p.y = p1.y - robot->y;

  //(u_a-u_p)'*u_fw_tp
  double alp = u_a_p.x * u_fw_tp.x + u_a_p.y * u_fw_tp.y;
  u_fw_p.x = u_a_p.x - alp * u_fw_tp.x;
  u_fw_p.y = u_a_p.y - alp * u_fw_tp.y;

  //            % 3. Combine u_fw_tp and u_fw_pp into u_fw;
  Vector u_fw_pp;
  double norm_ufwp = sqrt(u_fw_p.x * u_fw_p.x + u_fw_p.y * u_fw_p.y);
  u_fw_pp.x = u_fw_p.x / norm_ufwp;
  u_fw_pp.y = u_fw_p.y / norm_ufwp;

  //            u_fw_pp = u_fw_p/norm(u_fw_p);
  Vector u_fw;
  //            u_fw = d_fw*u_fw_tp+(u_fw_p-d_fw*u_fw_pp);

  u_fw.x = d_fw * u_fw_tp.x + (u_fw_p.x - d_fw * u_fw_pp.x);
  u_fw.y = d_fw * u_fw_tp.y + (u_fw_p.y - d_fw * u_fw_pp.y);

  double e, e_I, e_D, w, theta_fw;

  //          % Compute the heading and error for the PID controller
  theta_fw = atan2(u_fw.y, u_fw.x);

  e = theta_fw - robot->theta;
  e = atan2(sin(e), cos(e));

  double kp = Kp;
  if (abs(e) > 1)
  {
    e_I = 0;
    kp = Kp / 2;
  }
  else
  {
    e_I = lastErrorIntegration + e * dt;
  }
  e_D = (e - lastError) / dt;
  w = kp * e + Ki * e_I + Kd * e_D;
  lastErrorIntegration = e_I;
  lastError = e;

  // 控制W 拐弯半径，避免撞墙
  // D = (2*V - W*L)/(2*W)

  // if (m_beCorner)
  // {
  //   double aw = abs(w);
  //   double d = (2 * input->v - aw * robot->wheel_base_length) / (2 * aw);
  //   // this.d_fw + 车长 0.25 + 0.15 = 0.4
  //   if (d < 0.45)
  //   {
  //     aw = 2 * input->v / (0.9 + robot->wheel_base_length);
  //     if (w < 0)
  //       w = -aw;
  //     else
  //       w = aw;
  //   }
  // }

  output->v = input->v; // / (1 + abs(robot->w));
  output->w = w;

  if( abs(w) > 1 ) //控制拐弯速度？？
  {
    output->v = abs(w) * (0.1 - input->v)/2 + input->v;
  }

  // Serial.print(e, 3);
  // Serial.write(',');
  // Serial.print(e_I, 3);
  // Serial.write(',');
  // Serial.print(w, 3);
  // Serial.write(',');

  // log("FL:%s, %s, %s\n",
  //     floatToStr(0, output->v),
  //     floatToStr(1, e),
  //     floatToStr(2, w));
}

void FollowWall::getWall(Robot *robot)
{

  IRSensor **irSensors = robot->getIRSensors();

  double d = d_fw * 1.42;
  double d1 = 10;

  int idx = 0;
  if (dir == 0) // follow left
  {
    for (int i = 1; i < 3; i++)
    {
      if (irSensors[i]->distance >= irSensors[idx]->distance)
        idx = i;
    }

    switch (idx)
    {
    case 0:
      p1 = irSensors[1]->getWallVector(robot->x, robot->y, robot->theta, d);
      p0 = irSensors[2]->getWallVector(robot->x, robot->y, robot->theta, d);
      d1 = irSensors[2]->distance;
      break;
    case 1:
      p1 = irSensors[0]->getWallVector(robot->x, robot->y, robot->theta, d);
      p0 = irSensors[2]->getWallVector(robot->x, robot->y, robot->theta, d);
      d1 = irSensors[2]->distance;
      break;
    case 2:
      p1 = irSensors[0]->getWallVector(robot->x, robot->y, robot->theta, d);
      p0 = irSensors[1]->getWallVector(robot->x, robot->y, robot->theta, d);
      d1 = irSensors[1]->distance;
      break;
    }
  }

  else
  {
    //get the right wall

    idx = 2;
    for (int i = 3; i < 5; i++)
    {
      if (irSensors[i]->distance > irSensors[idx]->distance)
        idx = i;
    }

    switch (idx)
    {
    case 2:
      p1 = irSensors[4]->getWallVector(robot->x, robot->y, robot->theta, d);
      p0 = irSensors[3]->getWallVector(robot->x, robot->y, robot->theta, d);
      d1 = irSensors[3]->distance;
      break;
    case 3:
      p1 = irSensors[4]->getWallVector(robot->x, robot->y, robot->theta, d);
      p0 = irSensors[2]->getWallVector(robot->x, robot->y, robot->theta, d);
      d1 = irSensors[2]->distance;
      break;
    case 4:
      p1 = irSensors[3]->getWallVector(robot->x, robot->y, robot->theta, d);
      p0 = irSensors[2]->getWallVector(robot->x, robot->y, robot->theta, d);
      d1 = irSensors[2]->distance;
      break;
    }
  }

  if (d1 > irSensors[2]->getMaxDistance() - 0.1)
    m_beCorner = true;
  else
  {
    m_beCorner = false;
  }
}
