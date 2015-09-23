#include "stm32f4xx.h"

#ifndef _Kalman_h
#define _Kalman_h

#ifdef __cplusplus
 extern "C" {
#endif 

float kalman_filter_angleX(float newAngle, float newRate, float looptime);
float kalman_filter_angleY(float newAngle, float newRate, float looptime);
float kalmanX_single(float accx, float measure_noise_x, float process_noise_x);
float kalmanY_single(float accy, float measure_noise_y, float process_noise_y);
float kalmanZ_single(float accz, float measure_noise_z, float process_noise_z);

#endif //Kalman.h
