#include "Controller.h"

Controller::Controller()
{
        lastError = 0;
        lastErrorIntegration = 0;
}

void Controller::updateSettings(SETTINGS settings)
{
        Kp = settings.kp;
        Ki = settings.ki;
        Kd = settings.kd;
}
