#ifndef _PWM_CTRL_H_
#define _PWM_CTRL_H_
/*=====================================================================================================*/
/*=====================================================================================================*/
#define BLDC_PWM_MIN      	1110
#define BLDC_PWM_MED      	1500    // Duty Cycle 50%
#define BLDC_PWM_MAX      	2000    // Duty Cycle 100%
#define Prescaler_pwm		168-1
#define Period_pwm			20000-1
#define BLDC_M1   			TIM3->CCR1    // MOTOR M1
#define BLDC_M2   			TIM3->CCR2    // MOTOR M2
#define BLDC_M3   			TIM3->CCR3    // MOTOR M3
#define BLDC_M4   			TIM3->CCR4    // MOTOR M4

#define RCC_AHBPeriph_TIM_PWM 	RCC_APB2Periph_TIM1
#define RCC_AHBPeriph_GPIO_PWM	RCC_AHB1Periph_GPIOA
#define TIM_PWM 				TIM1
#define GPIO_PWM 				GPIOA
#define GPIO_Pin_PWM_CH1		GPIO_Pin_8
#define GPIO_Pin_PWM_CH2 		GPIO_Pin_9
#define GPIO_Pin_PWM_CH3		GPIO_Pin_10
#define GPIO_Pin_PWM_CH4 		GPIO_Pin_11
#define GPIO_Pin_PWM_CH1_SOURCE GPIO_PinSource8
#define GPIO_Pin_PWM_CH2_SOURCE GPIO_PinSource9
#define GPIO_Pin_PWM_CH3_SOURCE GPIO_PinSource10
#define GPIO_Pin_PWM_CH4_SOURCE GPIO_PinSource11
#define GPIO_AF_TIM_PWM 		GPIO_AF_TIM1
/*=====================================================================================================*/
/*=====================================================================================================*/
#ifdef __cplusplus
 extern "C" {
#endif
/*=====================================================================================================*/
/*=====================================================================================================*/
void BLDC_Config(void); 
//void BLDC_CtrlPWM(s16 PWM_M1, s16 PWM_M2, s16 PWM_M3, s16 PWM_M4);

/*=====================================================================================================*/
/*=====================================================================================================*/
#ifdef __cplusplus
}
#endif
/*=====================================================================================================*/
/*=====================================================================================================*/
#endif
