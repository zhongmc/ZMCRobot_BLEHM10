#ifndef _BlinkLed_H_
#define _BlinkLed_H_

#include <Arduino.h>
#include "Robot.h"

class BlinkLed
{
public:
  BlinkLed();
  BlinkLed(int ledPin, int onTime, int offTime);

  void beSureToBlink();

  void normalBlink();
  void fastBlink();
  void slowBlink();
  void init(){};

  int m_onTime, m_offTime;
  int m_ledPin;

private:
  bool isLedOn;
  long m_onTimer, m_offTimer;
};

#endif /* _BlinkLed_H_ */
