#ifndef _UTILS_H
#define _UTILS_H
#include "ZMCRobot.h"


int getCrossPoint(Vector p0, Vector p3, Vector p1, Vector p2, Vector &result);
bool doesVectorCross(Vector P0, Vector P1, Vector Q0, Vector Q1);

double CrossProduct(double x1, double y1, double x2, double y2);

#endif