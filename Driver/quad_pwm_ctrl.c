#include "stm32f4xx.h"
#include "quad_pwm_ctrl.h"
/*=====================================================================================================*/
/*=====================================================================================================*/

void BLDC_Config(void)
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	TIM_OCInitTypeDef          TIM_OCInitStructure;
	GPIO_InitTypeDef           GPIO_InitStructure;
	
	 /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
    /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);

  /************************** PWM GPIO Configuration **************************************/
  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
   
  /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
 
  /* Connect TIM3 pins to AF2 */ 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 

  /************************** PWM Output **************************************/
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 83;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 20000-1;//20 ms
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLDC_PWM_MIN;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLDC_PWM_MIN;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLDC_PWM_MIN;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLDC_PWM_MIN;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM3 counter enable */
  TIM_Cmd(TIM3, ENABLE);

  BLDC_M1 = BLDC_PWM_MIN;
  BLDC_M2 = BLDC_PWM_MIN;
  BLDC_M3 = BLDC_PWM_MIN;
  BLDC_M4 = BLDC_PWM_MIN;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void BLDC_CtrlPWM(s16 PWM_M1, s16 PWM_M2, s16 PWM_M3, s16 PWM_M4)
{
  /*
  if(PWM_M1 > BLDC_PWM_MAX)       PWM_M1 = BLDC_PWM_MAX;
  else if(PWM_M1 < BLDC_PWM_MIN)  PWM_M1 = BLDC_PWM_MIN;
  if(PWM_M2 > BLDC_PWM_MAX)       PWM_M2 = BLDC_PWM_MAX;
  else if(PWM_M2 < BLDC_PWM_MIN)  PWM_M2 = BLDC_PWM_MIN;
  if(PWM_M3 > BLDC_PWM_MAX)       PWM_M3 = BLDC_PWM_MAX;
  else if(PWM_M3 < BLDC_PWM_MIN)  PWM_M3 = BLDC_PWM_MIN;
  if(PWM_M4 > BLDC_PWM_MAX)       PWM_M4 = BLDC_PWM_MAX;
  else if(PWM_M4 < BLDC_PWM_MIN)  PWM_M4 = BLDC_PWM_MIN;
  */
  BLDC_M1 = PWM_M1;
  BLDC_M2 = PWM_M2;
  BLDC_M3 = PWM_M3;
  BLDC_M4 = PWM_M4;
}
/*=====================================================================================================*/
/*=====================================================================================================*/