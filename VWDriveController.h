#ifndef _VWDRIVE_CONTROLLER_H_
#define _VWDRIVE_CONTROLLER_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class VWDriveController : public Controller
{
public:
  VWDriveController();

  void reset();

  void reset(Robot *robot);
  void execute(Robot *robot, Input *input, Output *output, double dt);
  
  void setGoal(double v, double w, double curTheta );

  void PrintInfo()
  {
    log("Ctrl Info:%s,%s,%s;%s,%s\n",
        floatToStr(0, Kp),
        floatToStr(1, Ki),
        floatToStr(2, Kd),
        floatToStr(3, lastError),
        floatToStr(4, lastErrorIntegration));
  }

private:
  double m_v, m_w;  //设定的vw控制量
};

#endif /* _VELOCITY_CONTROLLER_H_ */
