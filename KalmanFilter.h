/*
KalmanFilter.h KalmanFilter.cpp - Library for BST-Balance car code.
Created by SKY ZHU&ROOMS LUO, OCTOBER 2, 2016.
JUST FOR THE Company of Technology of yahboom.
In order to  avoid Infringement Act,this core is not for the commerce except being authorized by the writer.
*/

#ifndef KalmanFilter_h
#define KalmanFilter_h

class KalmanFilter
{
  public:
	KalmanFilter();

	// float getAngle(float newAngle, float newRate, float dt);

	void setAngle(float angle); // Used to set angle, this should be set as the starting angle
	float getRate();			// Return the unbiased rate

	/* These are used to tune the Kalman filter */
	void setQangle(float Q_angle);
	void setQbias(float Q_bias);
	void setRmeasure(float R_measure);

	float getQangle();
	float getQbias();
	float getRmeasure();

	// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
	float getAngle(float angle_m, float gyro_m, float dt); //, float Q_angle, float Q_gyro, float R_angle, float C_0);

  private:
	/* Kalman filter variables */
	float Q_angle;   // Process noise variance for the accelerometer
	float Q_bias;	// Process noise variance for the gyro bias
	float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	float C_0;

	float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	float bias;  // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	float rate;  // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	// void Yiorderfilter(float angle_m, float gyro_m, float dt, float K1);
	// void Angletest(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro,
	// 			   float R_angle, float C_0, float K1);

	// float Gyro_x, Gyro_y, Gyro_z;
	// float accelz = 0;
	// float angle;
	// float angle6;

  private:
	float angle_err, q_bias;
	float Pdot[4] = {0, 0, 0, 0};
	float P[2][2] = {{1, 0}, {0, 1}};
	float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	float angle_dot;
};
#endif
//
// END OF FILE
//