/*
KalmanFilter.h KalmanFilter.cpp - Library for BST-Balance car code.
Created by SKY ZHU&ROOMS LUO, OCTOBER 2, 2016.
JUST FOR THE Company of Technology of yahboom.
In order to  avoid Infringement Act,this core is not for the commerce except being authorized by the writer.
*/

#include "./KalmanFilter.h"

KalmanFilter::KalmanFilter()
{

  // float Q_angle = 0.001, Q_gyro = 0.005; //角度数据置信度,角速度数据置信度
  // float R_angle = 0.5, C_0 = 1;

  // Q_angle = 0.001f;
  // Q_bias = 0.003f;
  // R_measure = 0.03f;

  Q_angle = 0.001f;
  Q_bias = 0.005f;
  R_measure = 0.5f;

  C_0 = 1;

  angle = 0.0f; // Reset the angle
  bias = 0.0f;  // Reset bias

  Pdot[0] = 0;
  Pdot[1] = 0;
  Pdot[2] = 0;
  Pdot[3] = 0;
  P[0][0] = 1;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 1;
}

void KalmanFilter::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float KalmanFilter::getRate() { return this->rate; };              // Return the unbiased rate

/* These are used to tune the Kalman filter */
void KalmanFilter::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void KalmanFilter::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void KalmanFilter::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float KalmanFilter::getQangle() { return this->Q_angle; };
float KalmanFilter::getQbias() { return this->Q_bias; };
float KalmanFilter::getRmeasure() { return this->R_measure; };

////////////////////////kalman/////////////////////////

float KalmanFilter::getAngle(float angle_m, float gyro_m, float dt) //, float Q_angle, float Q_gyro, float R_angle, float C_0)
{
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = Q_bias;

  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_measure + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias;

  return angle;
}
