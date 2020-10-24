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
  
  // void setGoal(double v, double w, double curTheta );

  void PrintInfo()
  {
    log("VW Ctrl Info:%s,%s,%s;%s,%s\n",
        floatToStr(0, Kp),
        floatToStr(1, Ki),
        floatToStr(2, Kd),
        floatToStr(3, lastError),
        floatToStr(4, lastErrorIntegration));
  }

  virtual void setSettings(SETTINGS settings )
  {
      Kp = settings.kp;
      Ki = settings.ki;
      Kd = settings.kd;

      vkp = settings.dkp;
      vki = settings.dki;
      vkd = settings.dkd;

      lastError = 0;
      lastErrorIntegration = 0;
      lastVError = 0;
      lastVErrorIntegration = 0;

  };  

  byte *getCtrlInfo()
  {
    return ctrl_info;
  }


private:
  void doWControll(Robot *robot, Input *input, Output *output, double dt);
  void doVControll(Robot *robot, Input *input, Output *output, double dt);


  double m_v, m_w;  //设定的vw控制量
  double vkp, vkd, vki;
  double lastVError, lastVErrorIntegration;

  byte ctrl_info[20];
  double ctrl_v, ctrl_w, vel_l, vel_r; 
};

#endif /* _VELOCITY_CONTROLLER_H_ */
