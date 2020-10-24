#ifndef _GOTOGOAL_H_
#define _GOTOGOAL_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"


#define THETA_GOAL 0.05
#define DISTANCE_GOAL 0.02

class GoToGoal : public Controller
{
public:
  GoToGoal();
  void reset();
  void execute(Robot *robot, Input *input, Output *output, double dt);
  bool isAtGoal(Robot *robot, Input *input );

  virtual void setSettings(SETTINGS settings )
  {
      Kp = settings.kp;
      Ki = settings.ki;
      Kd = settings.kd;

      pkp = settings.pkp;
      pki = settings.pki;
      pkd = settings.pkd;

      tkp = settings.tkp;
      tki = settings.tki;
      tki = settings.tki;
      lastError = 0;
      lastErrorIntegration = 0;
  };

   void PrintInfo()
  {

    if( state == 0 )
      Serial.println(" GTG Ctrl...");
    else if( state == 1 )
      Serial.println(" DST Ctrl...");
    else
    {
      Serial.println(" Theta Ctrl...");
    }
    
    log(" Dir [p,i,d e,ei]:%s,%s,%s,%s,%s\n",
        floatToStr(0, Kp),
        floatToStr(1, Ki),
        floatToStr(2, Kd),
        floatToStr(3, lastError),
        floatToStr(4, lastErrorIntegration));
    log(" D [p,i,d e,ei]:%s,%s,%s,%s,%s\n",
        floatToStr(0, pkp),
        floatToStr(1, pki),
        floatToStr(2, pkd),
        floatToStr(3, lastVE),
        floatToStr(4, lastVEI));
    log(" Q [p,i,d e,ei]:%s,%s,%s,%s,%s\n",
        floatToStr(0, tkp),
        floatToStr(1, tki),
        floatToStr(2, tkd),
        floatToStr(3, lastTE),
        floatToStr(4, lastTEI));
  }



private:
  double lastVE, lastVEI;
  double lastTE, lastTEI;

  double pkp, pki, pkd;
  double tkp, tki, tkd;
  int state;
};

#endif /* _GOTOGOAL_H_ */
