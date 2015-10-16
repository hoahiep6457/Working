#include "stm32f4xx.h"
#include <stdlib.h>
#include <math.h>
#include "MPU6050.h"
#include "HMC5883L.h"
#include "Kalman.h"
#include "quad_pwm_ctrl.h"
#include "quad_i2c_ctrl.h"
#include "pid.h"
#include "delay_ctrl.h"
#include "Rx.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define GYRO_LSB  32.8 //Gyro FS1000
#define ACC_LSB   2048 //Accel FS16 
#define RAD_TO_DEG  57.2957795131
#define DEG_TO_RAD  0.01745329251
#define DT				0.001 // T sampling
/*	      MASTER                    SLAVE
	 PB8 --- I2C1_SCL              	MPU6050
	 PB9 --- I2C1_SDA               HMC5883L
	*/

/*=====================================================================================================*/
/*=====================================================================================================*/
void IMU_Get_Start(void);
void Led_Config(void);
void IMU_Get_Data(void);
void IMU_Get_Offset(void);
void TIM2_Config_Counter(void);
void PID_Update(void);
void PID_Init_Start(void);
/*=====================================================================================================*/
/*=====================================================================================================*/
int16_t accX=0,accY=0,accZ=0,gyroX=0,gyroY=0,gyroZ=0,magX=0,magY=0,magZ=0;
int16_t Gyro_zero[7];
int16_t temp;
int16_t MPU6050data[7];
int16_t HMC5883Ldata[3];
int16_t magX_offset=0, magY_offset=0,magZ_offset=0;

float accX_kalman, accY_kalman, accZ_kalman;
float Bfx, Bfy, My, Mx;
float gyroX_offset=0, gyroY_offset=0, gyroZ_offset=0;
float angleX, angleY, angleZ;
float gyroX_rate, gyroY_rate, gyroZ_rate;
float angleX_kalman, angleY_kalman, angleZ_kalman;
float gyroX_angle, gyroY_angle, gyroZ_angle;
float roll_angle, pitch_angle;
int16_t BasicThr;
/*=====================================================================================================*/
/*=====================================================================================================*/
int main(void)
{

  SystemInit();
	BLDC_Config();
  Rx_Configuration();//Configuration interrupt to calculate dutycycle received from Rx
  //I2C_Configuration(); 
  //HMC5883L_Initialize();
  delay_ms(10000); 
  //MPU6050_Initialize();//LSB gyro = 32.8 LSB acc = 2048
  delay_ms(10);//wait for MPU to stabilize
  //IMU_Get_Offset();//read MPU6050 to calib gyro
  Led_Config();
  //IMU_Get_Start();
  delay_ms(10);//delay to avoid hating
  //PID_Init_Start();
	//SysTick_Config(SystemCoreClock / 999);//start to read MPU each 1 ms
	GPIO_SetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
  //start PWM to test
  BasicThr = 800;
  while (1)
  {
		
  }
/*=====================================================================================================*/
/*=====================================================================================================*/
}
//read MPU once 1 ms
void IMU_Get_Data(void)
{
/*--------------------------Hanle MPU6050---------------------------------------*/
  MPU6050_GetRawAccelTempGyro(MPU6050data);
  accX = MPU6050data[0];
  accY = MPU6050data[1];
  accZ = MPU6050data[2];
  //temp = MPU6050data[3];
  gyroX = MPU6050data[4];
  gyroY = MPU6050data[5];
  gyroZ = MPU6050data[6];
	//Kalman fiter value of Accel
  accX_kalman = kalman_single(&kalman_single_X, accX, 5, 0.5);
  accY_kalman = kalman_single(&kalman_single_Y, accY, 5, 0.5);
  accZ_kalman = kalman_single(&kalman_single_Z, accZ, 5, 0.5);
  //Caculate Gyrorate 
  gyroX_rate = (gyroX - gyroX_offset)/GYRO_LSB;
  gyroY_rate = (gyroY - gyroY_offset)/GYRO_LSB;
  gyroZ_rate = (gyroZ - gyroZ_offset)/GYRO_LSB;
  //kalman filter
  gyroX_angle += gyroX_rate*DT;
  gyroY_angle += gyroY_rate*DT;
	gyroZ_angle += gyroZ_rate*DT;
	//Calculate Angle from Accel
  /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -180<=roll<=180    */
    /* roll is positive and increasing when moving downward                                     */
    /*                                                                                          */
    /*                                 y                                                        */
    /*             roll = atan(-----------------)                                               */
    /*                          sqrt(x^2 + z^2)                                                 */
    /* where:  x, y, z are returned value from accelerometer sensor                             */

    /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -90<=pitch<=90)     */
    /* pitch is positive and increasing when moving upwards                                     */
    /*                                                                                          */
    /*                                 x                                                        */
    /*            pitch = atan(-----------------)                                               */
    /*                          sqrt(y^2 + z^2)                                                 */
    /* where:  x, y, z are returned value from accelerometer
  */
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    angleX = atan2(accY_kalman,accZ_kalman) * RAD_TO_DEG;
    angleY = atan(-accX_kalman / sqrt(accY_kalman * accY_kalman + accZ_kalman * accZ_kalman)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    angleX = atan(accY_kalman / sqrt(accX_kalman * accX_kalman + accZ_kalman * accZ_kalman)) * RAD_TO_DEG;
    angleY = atan2(-accX_kalman, accZ_kalman) * RAD_TO_DEG;
  #endif

  #ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((angleX < -90 && angleX_kalman > 90) || (angleX > 90 && angleX_kalman < -90)) {
    kalmanX.angle = angleX;
    angleX_kalman = angleX;
  } 
  else
    angleX_kalman = kalman_filter_angle(&kalmanX, angleX, gyroX_rate, DT);

  if (abs(angleX_kalman) > 90)
    gyroY_rate = -gyroY_rate; // Invert rate, so it fits the restriced accelerometer reading
    angleY_kalman = kalman_filter_angle(&kalmanY, angleY, gyroY_rate, DT);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((angleY < -90 && angleY_kalman > 90) || (angleY > 90 && angleY_kalman < -90)) {
      kalmanY.angle = angleY;
      angleY_kalman = angleY;
    } 
    else
      angleY_kalman = kalman_filter_angle(&kalmanY, angleY, gyroY_rate, DT);

    if (abs(angleY_kalman) > 90)
      gyroX_rate = -gyroX_rate; // Invert rate, so it fits the restriced accelerometer reading
      angleX_kalman = kalman_filter_angle(&kalmanX, angleX, gyroX_rate, DT);
    #endif
 
  /*--------------------Handle HMC5883L---------------------*/
     /* Sensor rotates around Z-axis                                                           */
    /* yaw is the angle between the 'X axis' and magnetic north on the horizontal plane (Oxy) */
    /* jump between -180<=yaw<=180  																													*/
    /* Yaw = atan(-Bfy / Bfx)                                                                */
    /* rollAngle = to_radians(angleX);																						*/
    /* pitchAngle = to_radians(angleY);																						*/
    /* Bfy = -(magn->z * sin(rollAngle) - magn->y * cos(rollAngle));							*/
    /* Bfx = magn->x * cos(pitchAngle) +																					*/
    /*             magn->y * sin(pitchAngle) * sin(rollAngle) +										*/
    /*             magn->z * sin(pitchAngle) * cos(rollAngle);										*/
    /* yaw = to_degrees(atan2(Bfy, Bfx));																					*/

  HMC5883L_GetHeading(HMC5883Ldata);

  magX = m_scale*HMC5883Ldata[0];
  magY = m_scale*HMC5883Ldata[1];
  magZ = m_scale*HMC5883Ldata[2];

  //Calibration Mag sensor
  magX-=magX_offset;
  magY-=magY_offset;
  magZ-=magZ_offset;

  roll_angle = angleX_kalman*DEG_TO_RAD;
  pitch_angle = angleY_kalman*DEG_TO_RAD;
	
  Bfy = magY * cos(roll_angle) - magZ * sin(roll_angle);
  Bfx = magX * cos(pitch_angle) + magY * sin(pitch_angle) * sin(roll_angle) + magZ * sin(pitch_angle) * cos(roll_angle);

  angleZ = atan2(-Bfy, Bfx) * RAD_TO_DEG;
	
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((angleZ < -90 && angleZ_kalman > 90) || (angleZ > 90 && angleZ_kalman < -90)) {
    kalmanZ.angle = angleZ;
    angleZ_kalman = angleZ;
  } 
  else
    angleZ_kalman = kalman_filter_angle(&kalmanZ, angleZ, gyroZ_rate, DT);
   
   
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void IMU_Get_Start(void)
{
  //MPU6050 get starting
  MPU6050_GetRawAccelTempGyro(MPU6050data);
	accX = MPU6050data[0];
	accY = MPU6050data[1];
  accZ = MPU6050data[2];
	
  angleX = atan2(accY, accZ) * RAD_TO_DEG;
	angleY = -atan2(accX, accZ) * RAD_TO_DEG;

  gyroX_angle = angleX;
  kalmanX.angle = angleX;

	gyroY_angle = angleY;
  kalmanY.angle = angleY;

  //HMC5883L get starting
  HMC5883L_GetHeading(HMC5883Ldata);

  magX = m_scale*HMC5883Ldata[0];
  magY = m_scale*HMC5883Ldata[1];
  magZ = m_scale*HMC5883Ldata[2];
  
  //Calibration Mag sensor
  magX -= magX_offset;
  magY -= magY_offset;
  magZ -= magZ_offset;

  roll_angle = angleX_kalman*DEG_TO_RAD;
  pitch_angle = angleY_kalman*DEG_TO_RAD;
	
  Bfy = magY * cos(roll_angle) - magZ * sin(roll_angle);
  Bfx = magX * cos(pitch_angle) + magY * sin(pitch_angle) * sin(roll_angle) + magZ * sin(pitch_angle) * cos(roll_angle);

  angleZ = atan2(-Bfy, Bfx) * RAD_TO_DEG;
	
  gyroZ_angle = angleZ;
  kalmanZ.angle = angleZ;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void IMU_Get_Offset(void)
{ 
	static float magX_max=0, magX_min=0, magY_max=0, magY_min=0, magZ_max=0, magZ_min=0;
	int16_t i;
  /****************************Calib Gyro Sensor**********************/
  for(i=0; i<1000; i++)
  {
    MPU6050_GetRawAccelTempGyro(Gyro_zero);
    gyroX_offset += Gyro_zero[4];
    gyroY_offset += Gyro_zero[5];
    gyroZ_offset += Gyro_zero[6];
  }
  gyroX_offset /= 1000;
  gyroY_offset /= 1000;
  gyroZ_offset /= 1000;

  /*****************************Calib Magnetic sensor*****************/
  for(i=0; i<1500; i++)
  {
    HMC5883L_GetHeading(HMC5883Ldata);
    magX = HMC5883Ldata[0]*m_scale;
    magY = HMC5883Ldata[1]*m_scale;
    magZ = HMC5883Ldata[2]*m_scale;
    // Uses SCALED values, since scaled is the RAW adjusted for the gauss (sensitivity). 
    if(magX < magX_min) magX_min = magX;
    if(magY < magY_min) magY_min = magY;
    if(magZ < magZ_min) magZ_min = magZ;
    if(magX > magX_max) magX_max = magX;
    if(magX > magY_max) magY_max = magY;
    if(magX > magZ_max) magZ_max = magZ;
    // Calculate the magX_offset; magY_offset; magZ_offset 
    magX_offset = magX_max - (magX_max - magX_min)/2;  
    magY_offset = magY_max - (magY_max - magY_min)/2;  
    magZ_offset = magZ_max - (magZ_max - magZ_min)/2; 
  }
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void PID_Update(void)
{
  u16 BLDC_M[4] = {0};
  int16_t Pitch = 0, Roll = 0, Yaw = 0;
  //PID Angle
  Roll  = PID_Adjustment(&PID_Roll, 0, angleX_kalman);
  Pitch = PID_Adjustment(&PID_Pitch, 0, angleY_kalman);
  Yaw = PID_Adjustment(&PID_Yaw, 0, angleZ_kalman);
  /* Motor Ctrl */
  BLDC_M[0] = BasicThr + Pitch + Roll + Yaw;
  BLDC_M[1] = BasicThr - Pitch + Roll - Yaw;
  BLDC_M[2] = BasicThr - Pitch - Roll + Yaw;
  BLDC_M[3] = BasicThr + Pitch - Roll - Yaw;
  // Thr Ctrl
  //BLDC_CtrlPWM(BLDC_M[0], BLDC_M[1], BLDC_M[2], BLDC_M[3]);

}
/*=====================================================================================================*/
/*=====================================================================================================*/
void PID_Init_Start(void)
{
  PID_init(&PID_Roll, 0.002, 0, 0);
  PID_init(&PID_Pitch, 0.001, 0, 0);
  PID_init(&PID_Yaw, 0.001, 0, 0);
}

/*=====================================================================================================*/
/*=====================================================================================================*/
void Led_Config(void)
{
	GPIO_InitTypeDef           GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*=====================================================================================================*/
/*=====================================================================================================*/
void SysTick_Handler(void)
{
	IMU_Get_Data();
  //PID_Update();
}
/*=====================================================================================================*/
/*=====================================================================================================*/

/*=====================================================================================================*/
/*=====================================================================================================*/
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif
/*=====================================================================================================*/
/*=====================================================================================================*/
