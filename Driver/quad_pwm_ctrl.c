#include "stm32f4xx.h"
#include "quad_pwm_ctrl.h"
#include "delay_ctrl.h"
/*=====================================================================================================*/
/*=====================================================================================================*/

void BLDC_Config(void)
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	TIM_OCInitTypeDef          TIM_OCInitStructure;
	GPIO_InitTypeDef           GPIO_InitStructure;
	
	 /* TIM1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM1, ENABLE);
    /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /************************** PWM GPIO Configuration **************************************/
	
	/* GPIOC Configuration: TIM1 CH1 (PA8), CH2 (PA9), CH3 (PA10), CH4(PA11) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
   

  /* Connect TIM1 pins to AF1 */ 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1); 

  /************************** PWM Output **************************************/
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 168-1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 20000-1;//20 ms
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLDC_PWM_MIN;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLDC_PWM_MIN;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLDC_PWM_MIN; 
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = BLDC_PWM_MIN;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);
  /* TIM counter enable */
  TIM_Cmd(TIM1, ENABLE);

  BLDC_M1 = BLDC_PWM_MIN;
  BLDC_M2 = BLDC_PWM_MIN;
  BLDC_M3 = BLDC_PWM_MIN;
  BLDC_M4 = BLDC_PWM_MIN;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void BLDC_CtrlPWM(s16 PWM_M1, s16 PWM_M2, s16 PWM_M3, s16 PWM_M4)
{
  
  if(PWM_M1 > BLDC_PWM_MAX)       PWM_M1 = BLDC_PWM_MAX;
  else if(PWM_M1 < BLDC_PWM_MIN)  PWM_M1 = BLDC_PWM_MIN;
  if(PWM_M2 > BLDC_PWM_MAX)       PWM_M2 = BLDC_PWM_MAX;
  else if(PWM_M2 < BLDC_PWM_MIN)  PWM_M2 = BLDC_PWM_MIN;
  if(PWM_M3 > BLDC_PWM_MAX)       PWM_M3 = BLDC_PWM_MAX;
  else if(PWM_M3 < BLDC_PWM_MIN)  PWM_M3 = BLDC_PWM_MIN;
  if(PWM_M4 > BLDC_PWM_MAX)       PWM_M4 = BLDC_PWM_MAX;
  else if(PWM_M4 < BLDC_PWM_MIN)  PWM_M4 = BLDC_PWM_MIN;

  BLDC_M1 = PWM_M1;
  BLDC_M2 = PWM_M2;
  BLDC_M3 = PWM_M3;
  BLDC_M4 = PWM_M4;
}
/*=====================================================================================================*/
/*=====================================================================================================*/