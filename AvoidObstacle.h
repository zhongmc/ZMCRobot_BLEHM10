#ifndef _AVOIDOBSTACLE_H_
#define _AVOIDOBSTACLE_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class AvoidObstacle : public Controller
{
public:
    AvoidObstacle();
    void reset();
    void execute(Robot *robot, Input *input, Output *output, double dt);
    bool beQuiteAvo()
    {
        if (m_state > 1)
        {
            // log("AVO:%d\n", m_state);
            return true;
        }
        else
            return false;
    }

private:
    short m_state;
    double m_w, m_theta;
};

#endif /* _AVOIDOBSTACLE_H_ */
