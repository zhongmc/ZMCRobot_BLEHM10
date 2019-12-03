
#include "SlidingMode.h"

SlidingMode::SlidingMode()
{
}

void SlidingMode::reset()
{
}

void SlidingMode::getWall(Robot *robot)
{
  IRSensor **irSensors = robot->getIRSensors();
  Vector p0, p1;

  double d = d_fw * 1.42;
  //get the left wall
  int idx = 0;
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
    break;
  case 1:
    p1 = irSensors[0]->getWallVector(robot->x, robot->y, robot->theta, d);
    p0 = irSensors[2]->getWallVector(robot->x, robot->y, robot->theta, d);
    break;
  case 2:
    p1 = irSensors[0]->getWallVector(robot->x, robot->y, robot->theta, d);
    p0 = irSensors[1]->getWallVector(robot->x, robot->y, robot->theta, d);
    break;
  }

  u_fw_l.x = p0.x - p1.x;
  u_fw_l.y = p0.y - p1.y;

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
    break;
  case 3:
    p1 = irSensors[4]->getWallVector(robot->x, robot->y, robot->theta, d);
    p0 = irSensors[2]->getWallVector(robot->x, robot->y, robot->theta, d);
    break;
  case 4:
    p1 = irSensors[3]->getWallVector(robot->x, robot->y, robot->theta, d);
    p0 = irSensors[2]->getWallVector(robot->x, robot->y, robot->theta, d);
    break;
  }
  u_fw_r.x = p0.x - p1.x;
  u_fw_r.y = p0.y - p1.y;
}

Vector SlidingMode::getSegma(Vector u_gtg, Vector u_ao, Vector u_fw)
{
  Vector sigma;
  sigma.x = 0;
  sigma.y = 0;


		if( u_ao.y == 0 || u_gtg.x == 0 )
		{
		  //Serial.println("Div by zero 1!");
		  return sigma;
		}
		double a1 = -u_ao.x/u_ao.y;
		double a2 = -u_gtg.y/u_gtg.x;

		double fv1,fv2;

		fv1 = (u_gtg.x + a1*u_gtg.y);
		fv2 =  (u_ao.y + a2*u_ao.x);
		if( fv1 == 0 || fv2 == 0 )
		{
		  //Serial.println("Div by zero 2!");
		  return;
		}
		sigma.x = ( u_fw.x + a1 *u_fw.y )/ fv1;
		sigma.y = (u_fw.y + a2 *u_fw.x) / fv2;		
    return sigma;

  // double ad_bc = u_gtg.x * u_ao.y - u_ao.x * u_gtg.y;
  // if (ad_bc == 0)
  // {
  //   Serial.println("0 in Segma!");
  //   return sigma;
  // }

  // sigma.x = (u_ao.y * u_fw.x - u_ao.x * u_fw.y) / ad_bc;
  // sigma.y = (u_gtg.x * u_fw.y - u_gtg.y * u_fw.x) / ad_bc;

  // return sigma;
}

void SlidingMode::execute(Robot *robot, Input *input, Output *output, double dt)
{

  double sensor_gains[] = {1, 0.5, 1, 0.5, 1};
  IRSensor **irSensors = robot->getIRSensors();

  double uao_x = 0, uao_y = 0;
  for (int i = 0; i < 5; i++)
  {
    uao_x = uao_x + (irSensors[i]->xw - robot->x) * sensor_gains[i];
    uao_y = uao_y + (irSensors[i]->yw - robot->y) * sensor_gains[i];
  }

  u_ao.x = uao_x;
  u_ao.y = uao_y;

  u_gtg.x = input->x_g - robot->x;
  u_gtg.y = input->y_g - robot->y;

  getWall(robot);

  leftObstacle = false;
  rightObstacle = false;

  double maxDistance = irSensors[0]->getMaxDistance();

  double obsL = min(irSensors[0]->distance, irSensors[1]->distance);
  double obsR = min(irSensors[3]->distance, irSensors[4]->distance);

  if (obsL < obsR)
  {
    if (obsL < maxDistance - 0.05)
      leftObstacle = true;
  }
  else
  {
    if (obsR < maxDistance - 0.05)
      rightObstacle = true;
  }

  sigma_l = getSegma(u_gtg, u_ao, u_fw_l);
  sigma_r = getSegma(u_gtg, u_ao, u_fw_r);

  slideLeft = sigma_l.x > 0 && sigma_l.y > 0;
  slideRight = sigma_r.x > 0 && sigma_r.y > 0;

#ifdef _DEBUG_
  Serial.print("Slide:sl=");
  Serial.print(slideLeft);
  Serial.print(",sr=");
  Serial.print(slideRight);
  Serial.print(",lo=");
  Serial.print(leftObstacle);
  Serial.print(",ro=");
  Serial.println(rightObstacle);
#endif

  output->v = 0;
  output->w = 0;
}

bool SlidingMode::slidingLeft()
{
  // if( !leftObstacle && !rightObstacle)
  //   return false;

  // if( slideLeft && slideRight )
  //       return  leftObstacle; //(sigma_l.x > 0 && sigma_l.y > 0 );
  // else
  //     return slideLeft;

  return leftObstacle && slideLeft;
}

bool SlidingMode::slidingRight()
{
  //  if( !leftObstacle && !rightObstacle)
  //   return false;

  // if( slideLeft && slideRight )
  //       return  rightObstacle; //(sigma_l.x > 0 && sigma_l.y > 0 );
  // else
  //     return slideRight;

  return rightObstacle && slideRight;
}

bool SlidingMode::quitSlidingLeft()
{

  if (!rightObstacle && !leftObstacle) //no obstacles quit
    return true;

  return !slideLeft; //!(sigma_l.x > 0 && sigma_l.y > 0 );
}

bool SlidingMode::quitSlidingRight()
{
  if (!rightObstacle && !leftObstacle) //no obstacles quit
    return true;
  return !slideRight; //!(sigma_r.x > 0 && sigma_r.y > 0 );
}
