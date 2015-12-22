#ifdef _USART_
#define _USART_
/*=====================================================================================================*/
/*=====================================================================================================*/
#ifdef __cplusplus
 extern "C" {
#endif
/*=====================================================================================================*/
/*=====================================================================================================*/
#define GPIO_USART					GPIOB;
#define RCC_AHB1Periph_GPIO_USART	RCC_AHB1Periph_GPIOB;
#define RCC_APBPeriph_USART			RCC_APB2Periph_USART1;
#define GPIO_Pin_USART_TX			GPIO_Pin_6;
#define GPIO_Pin_USART_RX			GPIO_Pin_7;
#define GPIO_Pin_USART_TX_SOURCE	GPIO_PinSource6;
#define GPIO_Pin_USART_RX_SOURCE	GPIO_PinSource7;
/*=====================================================================================================*/
/*=====================================================================================================*/
void USART_Configuration(unsigned int BaudRate);
void USARTx_SendChar(USART_TypeDef* USARTx, uint8_t Data);
void USARTx_SendString(USART_TypeDef* USARTx, uint8_t *Str);
uint8_t USARTx_GetChar(USART_TypeDef* USARTx);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
/*====================================================================================================*/
/*=====================================================================================================*/
#ifdef __cplusplus
}
#endif
/*=====================================================================================================*/
/*=====================================================================================================*/
#endif