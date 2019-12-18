#ifndef _BLE_FUNCS_H
#define _BLE_FUNCS_H

#include "robot.h"

void processBleCommandPackage(byte *dataBuf);
void setConfigValue(byte *cfgArray);
void intToByte(byte *arrayBuf, int val);
void floatToByte(byte *arrayBuf, double val, double scale);
double byteToFloat(byte *arrayBuf, double scale);
int byteToInt(byte *arrayBuf);

void sendRobotStateValue(byte stateType, Position pos, double irDistance[5], double voltage);
void processSetingsRequire();
void SendSettings(SETTINGS settings);
void sendStatePkgToBle(byte *buf, int len );
void sendSettingsPkgToBle(byte *settingsArray, int len);
void logToBle(const char *format, ...);
void motorSpeedBle(int pwml, int pwmr);

void sendMessage(char *str);
void initBluetooth();
#endif