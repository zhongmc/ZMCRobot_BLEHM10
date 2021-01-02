#ifndef _REAR_DRIVE_ROBOT_H_
#define _REAR_DRIVE_ROBOT_H_

#include <Arduino.h>
#include "Robot.h"

class RearDriveRobot : public Robot
{
public:
  RearDriveRobot();

  PWM_OUT getPWMOut(double v, double w);

  double vel_l_to_pwm(double vel);
  double vel_r_to_pwm(double vel);

  double pwm_to_ticks_r(double pwm, double dt);
  double pwm_to_ticks_l(double pwm, double dt);

private:
  // pwm = vel_pwm_a * vel + vel_pwm_b
  double vel_pwm_a, vel_pwm_b;
  //pwm to ticks ticks = dt*(pta * pwm^2 + ptb *pwm + ptc)
  double pta_l, pta_r, ptb_l, ptb_r, ptc_l, ptc_r;

  // Vel zeroMinVel(Vel vel);
};

#endif /* _REAR_DRIVE_ROBOT_H_ */
