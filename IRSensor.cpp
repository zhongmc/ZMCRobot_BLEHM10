#include "IRSensor.h"

//fallow wall 时给出的无障碍物时的坐标距离
#define WALL_DIS 0.35

IRSensor::IRSensor()
{
  mSensorType = GP2Y0A21;
  init();
}

IRSensor::IRSensor(SENSOR_TYPE sensorType)
{
  mSensorType = sensorType;
  init();
}

IRSensor::IRSensor(double xs, double ys, double thetas, byte pin, SENSOR_TYPE sensorType)
{
  x_s = xs;
  y_s = ys;
  anologPin = pin;
  theta_s = thetas;
  cos_theta = cos(thetas);
  sin_theta = sin(thetas);
  mSensorType = sensorType;
  init();
}

void IRSensor::init()
{
  bOpenFilter = false;
  filter = 0.4;
  prevDistance = getMaxDistance();
  setDistance(getMaxDistance());
  lastDistance = distance;
}

void IRSensor::SetSensorType(SENSOR_TYPE sensorType)
{
  mSensorType = sensorType;
  setDistance(getMaxDistance());
}

double IRSensor::readDistance(byte pin)
{
  int digVal = 0;
  digVal += analogRead(pin);
  digVal += analogRead(pin);
  digVal += analogRead(pin);
  return getDistance(digVal / 3);
}

void IRSensor::readPosition(double xs, double ys, double thetas, byte pin)
{
  x_s = xs;
  y_s = ys;
  anologPin = pin;
  theta_s = thetas;
  cos_theta = cos(thetas);
  sin_theta = sin(thetas);
  readPosition();
}

void IRSensor::readPosition()
{
  int digVal = 0;
  for( int i=0; i<5; i++ )
    digVal+=analogRead(anologPin);
    digVal = digVal/5;
    
  lastDistance = distance;
  distance = getDistance(digVal);
  if (bOpenFilter) //一阶滤波
  {
    distance = filter * prevDistance + (1 - filter) * distance;
    prevDistance = distance;
  }

  x = x_s + distance * cos_theta;
  y = y_s + distance * sin_theta;
}

void IRSensor::setDistance(double dis)
{
  distance = dis;
  x = x_s + distance * cos_theta;
  y = y_s + distance * sin_theta;
}

void IRSensor::applyGeometry(double xc, double yc, double sinTheta, double cosTheta)
{
  xw = xc + x * cosTheta - y * sinTheta;
  yw = yc + x * sinTheta + y * cosTheta;
}


Vector IRSensor::getWallVector(double xc, double yc, double theta, double d)
{
  Vector p;
  p.x = xw;
  p.y = yw;
  if (distance >= (getMaxDistance() - 0.01))
  {
    double dis = distance;
    double sinV = sin(theta);
    double cosV = cos(theta);

    setDistance(d);
    applyGeometry(xc, yc, sinV, cosV);
    p.x = xw;
    p.y = yw;

    setDistance(dis);
    applyGeometry(xc, yc, sinV, cosV);
  }
  // if (distance > d)
  // {
  //   double dis = distance;
  //   double sinV = sin(theta);
  //   double cosV = cos(theta);

  //   setDistance(d);
  //   applyGeometry(xc, yc, sinV, cosV);
  //   p.x = xw;
  //   p.y = yw;

  //   setDistance(dis);
  //   applyGeometry(xc, yc, sinV, cosV);
  // }
  return p;
}

void IRSensor::setIRFilter(bool open, float filt)
{
  bOpenFilter = open;
  filter = filt;
  prevDistance = getMaxDistance();
}

double IRSensor::getMaxDistance()
{
  if (mSensorType == GP2Y0A21)
    return 0.8;
  else if (mSensorType == GP2Y0A41)
    return 0.3;
  return 1;
}
double IRSensor::getMinDistance()
{
  if (mSensorType == GP2Y0A21)
    return 0.1;
  else if (mSensorType == GP2Y0A41)
    return 0.04;
  return 0.1;
}


#if OP_VOLTAGE == VOLT33
double IRSensor::getDistance(int digitalVal)
{
  double d;
  if (mSensorType == GP2Y0A21) //1/d = 0.0148x - 0.8085
  {
    if (digitalVal > 760)
      return 0.01;
    if (digitalVal < 120)
      return 0.8;
    d = 1.0 / (0.0148 * digitalVal - 0.8085);
    return d;
  }
  else if (mSensorType == GP2Y0A41) //  y = 0.0293x - 0.9342
  {
    if (digitalVal > 860)
      return 0.04;
    if (digitalVal < 105)
      return 0.3;

    d = 1.0 / (0.0293 * digitalVal - 0.9342);
    return d;
  }
  else
  {
    return 0.8;
  }
}

#else
//5V
double IRSensor::getDistance(int digitalVal)
{
  double d;
  if (mSensorType == GP2Y0A21) //1/d = 0.0148x - 0.8085
  {
    if (digitalVal > 471)
      return 0.01;
    if (digitalVal < 82)
      return 0.8;
    d = 1.0 / (0.0225 * digitalVal - 0.8085);
    return d;
  }
  else if (mSensorType == GP2Y0A41) //  y = 0.0293x - 0.9342
  {
    if (digitalVal > 563)
      return 0.04;
    if (digitalVal < 70)
      return 0.3;

    d = 1.0 / (0.044 * digitalVal - 0.7951);
    return d;
  }
  else
  {
    return 0.8;
  }
}

#endif