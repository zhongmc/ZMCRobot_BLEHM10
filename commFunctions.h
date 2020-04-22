#ifndef _COMM_FUNCTIONS_H
#define _COMM_FUNCTIONS_H


#define bluetooth Serial2  //17 Rx 16 Tx


#include "robot.h"

void SendRobotStates(Position pos, double irDistance[5], double voltage);
void SendSettings();
void SendMessages(const char *format, ...);

#endif