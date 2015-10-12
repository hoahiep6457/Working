#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "Rx.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
extern float DutyCycle, Frequency;
float IC1Value, IC2Value;
/*=====================================================================================================*/
/*=====================================================================================================*/
void Rx_Configuration(void)
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	TIM_ICInitTypeDef				   TIM_ICInitStructure;
	GPIO_InitTypeDef           GPIO_InitStructure;
	NVIC_InitTypeDef					 NVIC_InitStructure;
  RCC_ClocksTypeDef          RCC_ClocksStruct;
  
 	/* TIM1 and TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	
  /* Get clocks */
  RCC_GetClocksFreq(&RCC_ClocksStruct);
	
	/**************************************************************************/
  /* Compute the prescaler value                                            */
  /* PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1 = 3  */
	/**************************************************************************/

  /* GPIOC Configuration: TIM2 CH1 (PA0) and TIM2 CH2 (PA1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  /* connect TIM2 pins to AF2*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 1680-1; 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM2 configuration: Input Capture mode --------------------------------------
     The external signal is connected to TIM2 CH1 pin PA0 and TIM2 CH2 PA1
                                         TIM2 CH3 pin  and TIM2 CH4 
     The Bothedge is used as active edge,
     The TIM CCR1 and CCR2 is used to compute the frequency value 
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 ;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* Initialize PWM Input config */
  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

  /* Select input trigger */
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC1 and CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC1 , ENABLE);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void TIM2_IRQHandler(void)
{
    /* Interrupt handler for PWM Capture */
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1)) {
		
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    
    /* Get the Input Capture value */
    IC2Value = (float)TIM_GetCapture1(TIM2);

    if (IC2Value != 0) {
      /* Duty cycle and frequency */
      IC1Value = (float)TIM_GetCapture2(TIM2);
      DutyCycle = (IC1Value * 100) / IC2Value;
      Frequency = (SystemCoreClock/2) / (IC2Value*1680);

    } else {
      /* Reset data */
      DutyCycle = 0;
      Frequency = 0;
    }
  }
  if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) {
    /* Read data */
    
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    
    /* Get the Input Capture value */
    IC2Value = (float)TIM_GetCapture2(TIM2);

    if (IC2Value != 0) {
      /* Duty cycle and frequency */
      IC1Value = (float)TIM_GetCapture1(TIM2);
      DutyCycle = (IC1Value * 100) / IC2Value;
      Frequency = (SystemCoreClock/2) / IC2Value;
    } else {
      /* Reset data */
      DutyCycle = 0;
      Frequency = 0;
    }
  }
  
}