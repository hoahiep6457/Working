//=====================================================================================================
// AHRS.h
// S.O.H. Madgwick
// 17th Dec 2015
//=====================================================================================================
//
// See AHRS.c file for description.
// 
//=====================================================================================================
#ifndef H_AHRS
#define H_AHRS

#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases

#define dt 0.0025

float exInt , eyInt , ezInt ;	// scaled integral error
float q0, q1 , q2 , q3 ;	// quaternion elements representing the estimated orientation
float p,r,y;

void AHRS_setup(void);
void AHRS_UpdateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRS_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void toEuler;
#endif
