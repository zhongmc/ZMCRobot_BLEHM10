#ifndef _TTROBOT_H_
#define _TTROBOT_H_

#include <Arduino.h>
#include "Robot.h"

class TTRobot :public Robot{
    public:
        TTRobot();
        double vel_l_to_pwm( double vel);
        double vel_r_to_pwm( double vel);

        double pwm_to_ticks_r(double pwm, double dt);
        double pwm_to_ticks_l(double pwm, double dt);


   };


#endif /* _TTROBOT_H_ */
