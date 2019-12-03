#ifndef _BlinkMatrixLed_H_
#define _BlinkMatrixLed_H_
#define ARDUINO 10808

#include <Arduino.h>
#include "Robot.h"

#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

class BlinkMatrixLed
{
public:
  BlinkMatrixLed();

  void init();

  void beSureToBlink();

  void normalBlink(); //idle
  void slowBlink();   //bat low

  void balanceBlink();
  void laydownBlink();
  void runingBlink();

  // void fastBlink();

private:
  bool normal;
  long prevMillis;

  int curState;
  int runIdx;
  void showBackgroud();
  void showIdle();
  void showDead();
  void showBalance();
  void showLaydown();
  void showRunning();

  Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();
};

#endif /* _BlinkLed_H_ */
