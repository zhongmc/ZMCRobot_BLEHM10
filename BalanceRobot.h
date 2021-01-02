#ifndef _BALANCE_ROBOT_H_
#define _BALANCE_ROBOT_H_

#include <Arduino.h>
#include "Robot.h"

class BalanceRobot : public Robot
{
public:
    BalanceRobot();

    PWM_OUT getPWMOut(double v, double w);

    double vel_l_to_pwm(double vel);
    double vel_r_to_pwm(double vel);

    double pwm_to_ticks_r(double pwm, double dt);
    double pwm_to_ticks_l(double pwm, double dt);
};

#endif /* _BALANCE_ROBOT_H_ */
