#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <Arduino.h>
#include "Robot.h"

class Controller
{
public:
  Controller();
  virtual void reset() = 0;
  virtual void execute(Robot *robot, Input *input, Output *output, double dt) = 0;

  virtual void setSettings(SETTINGS settings)
  {
      Kp = settings.kp;
      Ki = settings.ki;
      Kd = settings.kd;

      lastError = 0;
      lastErrorIntegration = 0;

  };

  void setPIDParam(double kp, double ki, double kd )
  {
    Kp = kp;
    Ki = ki;
    Kd = kd;
  };

  void PrintInfo()
  {

    log(" [p,i,d e,ei]:%s,%s,%s,%s,%s\n",
        floatToStr(0, Kp),
        floatToStr(1, Ki),
        floatToStr(2, Kd),
        floatToStr(3, lastError),
        floatToStr(4, lastErrorIntegration));
  }

 
protected:
  double lastError;
  double lastErrorIntegration;
  double Kp, Ki, Kd;
};

#endif /* _CONTROLLER_H_ */
