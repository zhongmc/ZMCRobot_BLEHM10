#ifndef _DIFFERENCIAL_CONTROLLER_H_
#define _DIFFERENCIAL_CONTROLLER_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class DifferencialController : public Controller
{
public:
  DifferencialController();

  void reset();
  void execute(Robot *robot, Input *input, Output *output, double dt);

  // void reset(Robot *robot);
  // void setGoal(double v, double w);
  // void setGoal(double v, double theta, double curTheta);

  void PrintInfo()
  {
    log("Ctrl Info:%s,%s,%s;%s,%s\nQ:%s,%s\n",
        floatToStr(0, Kp),
        floatToStr(1, Ki),
        floatToStr(2, Kd),
        floatToStr(3, lastError),
        floatToStr(4, lastErrorIntegration),
        floatToStr(5, lastError1),
        floatToStr(6, lastErrorIntegration1));
  }

private:

  double lastError1 = 0;
  double lastErrorIntegration1 = 0;
  double lastV = 0;

  // double mTheta;  //目标方向
  // double mW;      //转弯
  // double curW;    //当前的转弯状态
  // bool keepTheta; //是否需要保存当前方向
  // int keepThetaTimer;
  // long thetaPrevMillis;
  // bool okToKeep;
  // int count;
  // double m_robot_w; //目前的转弯角速度
  // double m_v, m_w;  //拐弯控制量
};

#endif /* _VELOCITY_CONTROLLER_H_ */
