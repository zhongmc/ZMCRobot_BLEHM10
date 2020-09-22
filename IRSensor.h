#ifndef _IRSENSOR_H_
#define _IRSENSOR_H_

#include <Arduino.h>
#include "ZMCRobot.h"

typedef enum
{
  GP2Y0A41 = 0, //4-30cm
  GP2Y0A21 = 1  //10-80cm
} SENSOR_TYPE;

class IRSensor
{
public:
  IRSensor();
  IRSensor(SENSOR_TYPE sensorType);

  IRSensor(double xs, double ys, double thetas, byte pin, SENSOR_TYPE sensorType);
  void readPosition();
  void readPosition(double xs, double ys, double thetas, byte pin);
  void setDistance(double val);

  double distance, lastDistance; //obstacle distance
  double x, y;                   //the obtacle position in robot geometry
  double xw, yw;                 //the obtacle pos in real geometry

  // double w_x, w_y, w_xw, w_yw; // position for fallow wall;

  double readDistance(byte pin);

  void SetSensorType(SENSOR_TYPE sensorType);

  SENSOR_TYPE getSensorType()
  {
    return mSensorType;
  };

  void applyGeometry(double xc, double yc, double sinTheta, double cosTheta);
  double getDistance(int digitalVal);
  double getMaxDistance();
  double getMinDistance();

  void setIRFilter(bool open, float filter);

  // Vector getWallVector(double xc, double yc, double theta, double d);
  Vector getWallVector(double xc, double yc, double theta, double d_fw, double fw_v);

private:
  void init();

  byte anologPin = A1;

  double x_s, y_s, theta_s; //sensor position

  bool bOpenFilter;
  double filter, prevDistance;

  double cos_theta, sin_theta;
  SENSOR_TYPE mSensorType;
};

#endif /* _IRSENSOR_H_ */
