/**
  ******************************************************************************
  * @file    main.h
  * @author  thiagosilva@trixlog.com
  * @version V1.1.0
  * @date    22-June-2017
  * @brief   This file contains all the functions prototypes for the USART
  *          interface firmware driver.
  ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef USART_H_
#define USART_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_usart.h"

/* Exported types ------------------------------------------------------------*/

/* RX_Buffer definitions */
#define RX_BUFFER_SIZE		128

typedef struct
{
	uint8_t index;
	uint8_t data[RX_BUFFER_SIZE];
} RX_Buffer;

/* Exported constants --------------------------------------------------------*/

/* USART1  interface pins */
#define USART0						USART1
#define USART1_PERIPHERAL_CLK		RCC_APB2Periph_USART1

#define USART1_GPIO_PORT			GPIOA
#define USART1_TX_PIN				GPIO_Pin_9
#define USART1_RX_PIN				GPIO_Pin_10
#define USART1_GPIO_CLK				RCC_AHBPeriph_GPIOA

#define USART1_AF					GPIO_AF_1
#define USART1_TX_SOURCE			GPIO_PinSource9
#define USART1_RX_SOURCE			GPIO_PinSource10


/* USART2  interface pins */
#define USART0_2					USART2
#define USART2_PERIPHERAL_CLK		RCC_APB1Periph_USART2

#define USART2_GPIO_PORT			GPIOA
#define USART2_TX_PIN				GPIO_Pin_2
#define USART2_RX_PIN				GPIO_Pin_15
#define USART2_GPIO_CLK				RCC_AHBPeriph_GPIOA

#define USART2_AF					GPIO_AF_1
#define USART2_TX_SOURCE			GPIO_PinSource2
#define USART2_RX_SOURCE			GPIO_PinSource15


// Baudrate defines
#define USART1_DEFAULT_BAUDRATE		115200
#define USART2_DEFAULT_BAUDRATE		115200

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Initialization and Configuration functions *********************************/
void USART1_IOConfig(void);
void USART1_Init(uint32_t baudrate);
void USART1_DeInit();
void USART1_EnableRxInterrupt(void);

void USART2_IOConfig(void);
void USART2_Init(uint32_t baudrate);
void USART2_DeInit();
void USART2_EnableRxInterrupt(void);


/* Data transmission functions ************************************************/
void USART1_PutByte(uint8_t byte);
void USART1_PutString(volatile char *string);
void USART1_SendMessage(uint8_t *msg);


void USART2_PutByte(uint8_t byte);
void USART2_PutString(volatile char *string);
void USART2_SendMessage(uint8_t *msg);


/* Data Receive Function ******************************************************/

uint8_t GetStatusRx(void);


#endif /* USART_H_ */

/*******************************END OF FILE************************************/
