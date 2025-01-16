#ifndef _usart_H
#define _usart_H

#include "stm32f4xx.h"
#include "bsp_vofa.h"
#include "string.h"


#define Tx_Buffer_Num		4*Vofa_Num+4
#define Rx_Buffer_Num		20

extern u8 Rx_Flag;
extern uint8_t Tx_Buffer[Tx_Buffer_Num];


//引脚定义
/*******************************************************/
#define DEBUG_USART                             USART1
#define DEBUG_USART_CLK                         RCC_APB2Periph_USART1
#define DEBUG_USART_BAUDRATE                    2000000  //串口波特率

#define DEBUG_USART_RX_GPIO_PORT                GPIOB
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOB
#define DEBUG_USART_RX_PIN                      GPIO_Pin_7
#define DEBUG_USART_RX_AF                       GPIO_AF_USART1
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource7

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define DEBUG_USART_TX_PIN                      GPIO_Pin_9
#define DEBUG_USART_TX_AF                       GPIO_AF_USART1
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource9

#define DEBUG_USART_IRQHandler                  USART1_IRQHandler
#define DEBUG_USART_IRQ                 				USART1_IRQn
/************************************************************/

void DMA_Debug_USART_Tx_Data(uint32_t size);
void Info_Proc(void);
void Debug_USART_DMA_Config(void);
void Debug_USART_Config(void);
void USART3_DEVICE(void);
void USART6_DEVICE(void);
void USART8_DEVICE(void);

#endif 

