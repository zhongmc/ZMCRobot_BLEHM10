
#ifndef _SLIDINGMODE_H_
#define _SLIDINGMODE_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class SlidingMode : public Controller
{
public:
  SlidingMode();
  void reset();
  void execute(Robot *robot, Input *input, Output *output, double dt);

  bool slidingLeft();
  bool slidingRight();
  bool quitSlidingLeft();
  bool quitSlidingRight();

  Vector u_gtg, u_ao, u_fw_l, u_fw_r;
  Vector sigma_l, sigma_r;

  double d_fw;

private:
  void getWall(Robot *robot, double fw_v);
  Vector getSegma(Vector u_gtg, Vector u_ao, Vector u_fw);

  bool leftObstacle, rightObstacle;
  bool slideLeft, slideRight;

  //    private:
  //        double fuckEk_l;
  //        double E_k;
  //        double Kp, Ki, Kd;
  //
};

#endif /* _SLIDINGMODE_H_ */
