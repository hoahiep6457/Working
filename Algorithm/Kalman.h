#include "stm32f4xx.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
#ifndef _Kalman_h
#define _Kalman_h
/*=====================================================================================================*/
/*=====================================================================================================*/
#ifdef __cplusplus
 extern "C" {
#endif 
/*=====================================================================================================*/
/*=====================================================================================================*/
typedef struct {
	float	angle;// The angle calculated by the Kalman filter - part of the 2x1 state matrix
	float 	bias;// The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
	float	rate;// Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
	float	P[2][2]={0,0,0,0};// Error covariance matrix - This is a 2x2 matrix
	float	K[2];// Kalman gain - This is a 2x1 matrix   | 1|
	float	y;// Angle difference - 1x1 matrix           | 2|
	float	S;// Estimate error - 1x1 matrix
}kalman_t;

typedef struct {
	float	R;
	float	Q;
	float	P;
	float	x_hat;
	float	P_;
	float	K;
}kalman_single_t;

extern kalman_t kalmanX;
extern kalman_t kalmanY;

extern kalman_single_t	kalman_single_X;
extern kalman_single_t	kalman_single_Y;
extern kalman_single_t	kalman_single_Z;
/*=====================================================================================================*/
/*=====================================================================================================*/
float kalman_filter_angleX(float newAngle, float newRate, float looptime);
float kalman_filter_angleY(float newAngle, float newRate, float looptime);
float kalman_filter_angle(kalman_t *kalman, float newAngle, float newRate, float looptime);

float kalmanX_single(float accx, float measure_noise_x, float process_noise_x);
float kalmanY_single(float accy, float measure_noise_y, float process_noise_y);
float kalmanZ_single(float accz, float measure_noise_z, float process_noise_z);
float kalman_single(kalman_single_t *kalman_single, float acc, float measure_noise, float process_noise);
/*=====================================================================================================*/
/*=====================================================================================================*/
#endif //Kalman.h
