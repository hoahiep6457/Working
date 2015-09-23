#include "Kalman.h"
#include "stm32f4xx.h"
/* Kalman filter variables for X*/
float Q_angle = 0.001; //0.0004 //0.005 // Process noise variance for the accelerometer
float Q_bias = 0.003; //0.0002 //0.0003 //Process noise variance for the gyro bias
float R_measure = 0.03; //0.002 //0.008 // Measurement noise variance - this is actually the variance of the measurement noise
    
float x_angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
float x_bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
float x_rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    
float Px[2][2]={0,0,0,0}; // Error covariance matrix - This is a 2x2 matrix
float Kx[2]; // Kalman gain - This is a 2x1 matrix   | 1|
float y_x; // Angle difference - 1x1 matrix           | 2|
float S_x; // Estimate error - 1x1 matrix

/* Kalman filter variables for Y*/
float y_angle;
float y_bias;
float y_rate;

float Py[2][2]={0,0,0,0}; 
float Ky[2]; 
float y_y; 
float S_y; 

float kalman_filter_angleX(float newAngle, float newRate, float looptime)
{
    //calculate dt
    //float dt = (float)(looptime)/1000;
    float dt = looptime;
    /* Step 1 
    Discrete Kalman filter time update equations - Time Update ("Predict")*/
    x_rate = newRate - x_bias;
    x_angle += dt * x_rate;

    /* Step 2 
    Update estimation error covariance - Project the error covariance ahead*/
    Px[0][0] += dt * (dt*Px[1][1] - Px[0][1] - Px[1][0] + Q_angle);
    Px[0][1] -= dt * Px[1][1];
    Px[1][0] -= dt * Px[1][1];
    Px[1][1] += Q_bias * dt;

    /* Step 4 
    Calculate Kalman gain - Compute the Kalman gain*/
    S_x = Px[0][0] + R_measure;   //(Pk-) +R
    /* Step 5 */
    Kx[0] = Px[0][0] / S_x;        // K[0] = Kk = (Pk-)/ [(Pk-) +R]
    Kx[1] = Px[1][0] / S_x;
          
    /* Step 3 
    Calculate angle and bias - Update estimate with measurement zk (newAngle)*/
    y_x = newAngle - x_angle;      // Zk - (Xk-)  = y
    /* Step 6 */
    x_angle += Kx[0] * y_x;         // Xk = (Xk-) + Kk * [Zk - (Xk-)]
    x_bias += Kx[1] * y_x;

    /* Step 7 
    Calculate estimation error covariance - Update the error covariance*/
    Px[0][0] -= Kx[0] * Px[0][0];   // Pk.new = (Pk-) - Kk*(Pk-)
    Px[0][1] -= Kx[0] * Px[0][1];
    Px[1][0] -= Kx[1] * Px[0][0];
    Px[1][1] -= Kx[1] * Px[0][1];
          
    return x_angle;

}

float kalman_filter_angleY(float newAngle, float newRate, float looptime)
{
    //calculate dt
    //float dt = (float)(looptime)/1000;
    float dt = looptime;
    /* Step 1 
    Discrete Kalman filter time update equations - Time Update ("Predict")*/
    y_rate = newRate - y_bias;
    y_angle += dt * y_rate;

    /* Step 2 
    Update estimation error covariance - Project the error covariance ahead*/
    Py[0][0] += dt * (dt*Py[1][1] - Py[0][1] - Py[1][0] + Q_angle);
    Py[0][1] -= dt * Py[1][1];
    Py[1][0] -= dt * Py[1][1];
    Py[1][1] += Q_bias * dt;

    /* Step 4 
    Calculate Kalman gain - Compute the Kalman gain*/
    S_y = Py[0][0] + R_measure;   //(Pk-) +R
    /* Step 5 */
    Ky[0] = Py[0][0] / S_y;        // K[0] = Kk = (Pk-)/ [(Pk-) +R]
    Ky[1] = Py[1][0] / S_y;
          
    /* Step 3 
    Calculate angle and bias - Update estimate with measurement zk (newAngle)*/
    y_y = newAngle - y_angle;      // Zk - (Xk-)  = y
    /* Step 6 */
    y_angle += Ky[0] * y_y;         // Xk = (Xk-) + Kk * [Zk - (Xk-)]
    y_bias += Ky[1] * y_y;

    /* Step 7 
    Calculate estimation error covariance - Update the error covariance*/
    Py[0][0] -= Ky[0] * Py[0][0];   // Pk.new = (Pk-) - Kk*(Pk-)
    Py[0][1] -= Ky[0] * Py[0][1];
    Py[1][0] -= Ky[1] * Py[0][0];
    Py[1][1] -= Ky[1] * Py[0][1];
          
    return y_angle;
}

float kalmanX_single(float accx, float measure_noise_x, float process_noise_x)
{
 const float Rx=measure_noise_x*measure_noise_x;
 const float Qx=process_noise_x*process_noise_x; 
 static float x_hat,P;
 static float P_,K;
 
 /********* noi suy kalman ***************/
    P_ = P + Qx;                     // P_ = A*P*A' + Q;
    K = P_/(P_ + Rx);                // K = P_*H'*inv(H*P_*H' + R);
    x_hat = x_hat + K*(accx - x_hat);  // x_hat = x_hat + K*(z - H*x_hat);
    P = ( 1 - K)*P_ ;               // P = ( 1 - K*H)*P_ ;
 /****************************************/ 
 return x_hat;
}
float kalmanY_single(float accy, float measure_noise_y, float process_noise_y)
{
 const float Ry=measure_noise_y*measure_noise_y;
 const float Qy=process_noise_y*process_noise_y; 
 static float x_hat,P;
 static float P_,K;
 
 /********* noi suy kalman ***************/
    P_ = P + Qy;                     // P_ = A*P*A' + Q;
    K = P_/(P_ + Ry);                // K = P_*H'*inv(H*P_*H' + R);
    x_hat = x_hat + K*(accy - x_hat);  // x_hat = x_hat + K*(z - H*x_hat);
    P = ( 1 - K)*P_ ;               // P = ( 1 - K*H)*P_ ;
 /****************************************/ 
 return x_hat;
}
float kalmanZ_single(float accz, float measure_noise_z, float process_noise_z)
{
 const float Rz=measure_noise_z*measure_noise_z;
 const float Qz=process_noise_z*process_noise_z; 
 static float x_hat,P;
 static float P_,K;
 
 /********* noi suy kalman ***************/
    P_ = P + Qz;                     // P_ = A*P*A' + Q;
    K = P_/(P_ + Rz);                // K = P_*H'*inv(H*P_*H' + R);
    x_hat = x_hat + K*(accz - x_hat);  // x_hat = x_hat + K*(z - H*x_hat);
    P = ( 1 - K)*P_ ;               // P = ( 1 - K*H)*P_ ;
 /****************************************/ 
 return x_hat;
}