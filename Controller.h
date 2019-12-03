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

  void updateSettings(SETTINGS settings);

  void PrintInfo()
  {

    log(" [p,i,d e,ei]:%s,%s,%s,%s,%s\n",
        floatToStr(0, Kp),
        floatToStr(1, Ki),
        floatToStr(2, Kd),
        floatToStr(3, lastError),
        floatToStr(4, lastErrorIntegration));
  }

  void getSettings(SETTINGS *pSettings)
  {
    pSettings->kp = Kp;
    pSettings->ki = Ki;
    pSettings->kd = Kd;
  }

  void setCtrlParam(double val, int idx)
  {
    if (idx == 0)
      Kp = val;
    else if (idx == 1)
      Ki = val;
    else if (idx == 2)
      Kd = val;
  }

protected:
  double lastError;
  double lastErrorIntegration;
  double Kp, Ki, Kd;
};

#endif /* _CONTROLLER_H_ */
