#ifndef _RX_
#define _RX_
#include "stm32f4xx.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
#ifdef __cplusplus
 extern "C" {
#endif
/*=====================================================================================================*/
/*=====================================================================================================*/
typedef struct {
	uint8_t			Number;
	int32_t			Periodmeasure;
	int32_t 		Highmeasure;
	int32_t			Rising;
	int32_t			Falling;
	int32_t			PreRising;
	uint8_t			Frequency;
	uint16_t		NumCCR; 
}Rx_t;
/*=====================================================================================================*/
/*=====================================================================================================*/
extern Rx_t 	Rx_Throttle;
extern Rx_t 	Rx_Pitch;
extern Rx_t 	Rx_Roll;
extern Rx_t 	Rx_Yaw;
/*=====================================================================================================*/
/*=====================================================================================================*/
#define TIMRX_CLOCK 				84000000
#define TIMRX_TIME  				1000000
#define GPIO_CAPTURE				GPIOA
#define RCC_APB1Periph_TIM_RX	 	RCC_APB1Periph_TIM2
#define RCC_AHB1Periph_GPIO_RX		RCC_AHB1Periph_GPIOA
#define TIM_RX 						TIM2
#define GPIO_Pin_INT0				GPIO_Pin_0
#define GPIO_Pin_INT1 				GPIO_Pin_1
#define GPIO_Pin_INT2 				GPIO_Pin_2
#define GPIO_Pin_INT3 				GPIO_Pin_3
#define GPIO_Pin_INT0_SOURCE		GPIO_PinSource0
#define GPIO_Pin_INT1_SOURCE		GPIO_PinSource1
#define GPIO_Pin_INT2_SOURCE		GPIO_PinSource2
#define GPIO_Pin_INT3_SOURCE		GPIO_PinSource3

/*=====================================================================================================*/
/*=====================================================================================================*/
void Rx_Configuration(void);
void Calculate_Rx(Rx_t *Rx, uint32_t CCRx, uint16_t GPIO_Pin_x);
/*====================================================================================================*/
/*=====================================================================================================*/
#ifdef __cplusplus
}
#endif
#endif
/*=====================================================================================================*/
/*=====================================================================================================*/
