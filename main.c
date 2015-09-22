#include "stm32f4xx.h"
#include <stdlib.h>
#include <math.h>
#include "MPU6050.h"
#include "Kalman.h"
#include "main.h"
#define GYRO_LSB  32.8 //Gyro FS1000
#define ACC_LSB   2048 //Accel FS16 
#define RAD_TO_DEG  57.2957795131
#define DT				0.01 // T sampling
/*	      MASTER                    SLAVE
	 PB8 --- I2C1_SCL              	MPU6050
	 PB9 --- I2C1_SDA
	*/


void MPU_Get_Start(void);
void Led_Config(void);
void I2C_Configuration(void);
void Delay(__IO uint32_t nCount);
void MPU6050_Get_Data(void);
void delay_ms(__IO unsigned long ms);
void MPU6050_Get_Offset(void);
void TIM2_Config_Counter(void);
//int16_t DeviceID;
int16_t accX=0,accY=0,accZ=0,gyroX=0,gyroY=0,gyroZ=0;
int16_t Gyro_zero[7];
int16_t temp;
int16_t MPU6050data[7]; 
float accX_kalman, accY_kalman, accZ_kalman;
float gyroX_offset=0, gyroY_offset=0, gyroZ_offset=0;
float angleX, angleY, angleZ;
float gyroX_rate, gyroY_rate, gyroZ_rate;
float angleX_kalman, angleY_kalman, angleZ_kalman;
float gyroX_angle, gyroY_angle;
extern float x_angle, y_angle;
int32_t nowtime=0,lasttime=0;
float lengthtime;
int main(void)
{
  I2C_Configuration();  
  MPU6050_Initialize();//LSB gyro = 32.8 LSB acc = 2048
  delay_ms(10);//wait for MPU to stabilize
  MPU6050_Get_Offset();//read MPU6050 to calib gyro
  Led_Config();
  MPU_Get_Start();
  delay_ms(10);
  //TIM2_Config_Counter();
	SysTick_Config(SystemCoreClock / 99);//start to read MPU each 10 ms
	GPIO_SetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

	//DeviceID =  MPU6050_GetDeviceID();
  while (1)
  {
			
  }
}
//read MPU once 10 ms
void MPU6050_Get_Data(void)
{
	MPU6050_GetRawAccelTempGyro(MPU6050data);
	accX = MPU6050data[0];
	accY = MPU6050data[1];
	accZ = MPU6050data[2];
	//temp = MPU6050data[3];
	gyroX = MPU6050data[4];
	gyroY = MPU6050data[5];
	gyroZ = MPU6050data[6];
	//Kalman fiter value of Accel
	accX_kalman = kalmanX_single(accX, 5, 0.5);
  accY_kalman = kalmanY_single(accY, 5, 0.5);
	accZ_kalman = kalmanZ_single(accZ, 5, 0.5);
	//Calculate Angle from Accel
  angleX = atan2(accY_kalman, accZ_kalman) * RAD_TO_DEG;
  angleY = -atan2(accX_kalman,accZ_kalman) * RAD_TO_DEG;
  //Caculate Gyrorate 
  gyroX_rate = (gyroX - gyroX_offset)/GYRO_LSB;
	gyroY_rate = (gyroY - gyroY_offset)/GYRO_LSB;
  //gyroZ_rate = (gyroZ - gyroZ_offset)/GYRO_LSB;
  //kalman filter
	gyroX_angle += gyroX_rate*DT;
	gyroY_angle += gyroY_rate*DT;
  angleX_kalman = kalman_filter_angleX(angleX, gyroX_rate, DT);
	angleY_kalman = kalman_filter_angleY(angleY, gyroY_rate, DT);

}
void MPU_Get_Start(void)
{
  MPU6050_GetRawAccelTempGyro(MPU6050data);
	accX = MPU6050data[0];
	accY = MPU6050data[1];
  accZ = MPU6050data[2];
	
  angleX = atan2(accY, accZ) * RAD_TO_DEG;
	angleY = -atan2(accX, accZ) * RAD_TO_DEG;
	
  x_angle = angleX;
  gyroX_angle = angleX;
	y_angle = angleY;
	gyroY_angle = angleY;
}
void MPU6050_Get_Offset(void)
{
  int8_t i;
  for(i=0; i<100; i++)
  {
    MPU6050_GetRawAccelTempGyro(Gyro_zero);
    gyroX_offset += Gyro_zero[4];
    gyroY_offset += Gyro_zero[5];
    gyroZ_offset += Gyro_zero[6];
  }
  gyroX_offset /= 100;
  gyroY_offset /= 100;
  gyroZ_offset /= 100;
}
void I2C_Configuration(void)
{
#ifdef FAST_I2C_MODE
#define I2C_SPEED 400000
#define I2C_DUTYCYCLE I2C_DutyCycle_16_9  
#else /* STANDARD_I2C_MODE*/
#define I2C_SPEED 100000
#define I2C_DUTYCYCLE I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);	

  /* I2C De-initialize */
  I2C_DeInit(I2C1);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStructure);
 /* I2C ENABLE */
  I2C_Cmd(I2C1, ENABLE); 
  /* Enable Interrupt */

}

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
void delay_ms(__IO unsigned long ms)
{
   volatile unsigned long i,j;
	for (i = 0; i < ms; i++ )
	for (j = 0; j < 3442; j++ );
}
void Led_Config(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

}
void SysTick_Handler(void)
{
	MPU6050_Get_Data();
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif

