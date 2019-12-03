#ifndef _IRRECEIVER_H_
#define _IRRECEIVER_H_

#include <Arduino.h>
#include "ZMCRobot.h"

typedef struct
{
  int addr;
  int code_h;
  int code_l;
} IRCode;

class IRReceiver
{
public:
  IRReceiver();
  IRReceiver(int irpin);

  int readIRCode(IRCode &code);

private:
  long prevMicros, curMicros;
  int pulse_width_l, pulse_width_h;

  int irpin;
  int ir_addr, ir_code_h, ir_code_l; //ir_code, adrL_code, adrH_code;

  void readCode();
  int logicValue();
};

#endif /* _IRRECEIVER_H_ */
