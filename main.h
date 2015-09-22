#include "stm32f4xx.h"
#ifndef _main_h
#define _main_h

/* Private typedef -----------------------------------------------------------*/
TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
TIM_OCInitTypeDef          TIM_OCInitStructure;
TIM_ICInitTypeDef          TIM_ICInitStructure;
NVIC_InitTypeDef           NVIC_InitStructure;
GPIO_InitTypeDef           GPIO_InitStructure;
DMA_InitTypeDef            DMA_InitStructure;
ADC_InitTypeDef            ADC_InitStructure;
ADC_CommonInitTypeDef      ADC_CommonInitStructure;
EXTI_InitTypeDef  				 EXTI_InitStructure;	
USART_InitTypeDef 				 USART_InitStructure;
I2C_InitTypeDef   				 I2C_InitStructure;
#endif 