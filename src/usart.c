/**
  ******************************************************************************
  * @file    usart.c
  * @author  thiagosilva@trixlog.com
  * @version V1.0.0
  * @date    19-April-2017
  * @brief   This file provides firmware functions to manage functionalities
  *          of the USART peripheral.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#define RBUF_SIZE 256
#define TBUF_SIZE 256

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RX_Buffer recvBuffer;
uint8_t statusRx;

/* Private function prototypes -----------------------------------------------*/
void USART1_LowLevel_Init();
void USART2_LowLevel_Init();

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the peripheral used by the USART1 driver.
  * @param  None
  * @retval None
  */
void USART1_LowLevel_Init()
{
	GPIO_InitTypeDef GPIO_InitStructTX;

	/* Enable APB2 peripheral clock for USART1 */
	RCC_APB2PeriphClockCmd(USART1_PERIPHERAL_CLK,ENABLE);

	/* Enable peripheral clock for pins used by USART1 */
	RCC_AHBPeriphClockCmd(USART1_GPIO_CLK,ENABLE);

	//Setting USART1 TX Pin individually as Open-drain and Pull-Up
	GPIO_InitStructTX.GPIO_Pin 							= (USART1_TX_PIN);
	GPIO_InitStructTX.GPIO_Mode	 						= GPIO_Mode_AF;
	GPIO_InitStructTX.GPIO_OType 						= GPIO_OType_OD;
	GPIO_InitStructTX.GPIO_PuPd 						= GPIO_PuPd_UP;
	GPIO_InitStructTX.GPIO_Speed 						= GPIO_Speed_50MHz;
	GPIO_Init(USART1_GPIO_PORT,&GPIO_InitStructTX);


	//Setting USART1 RX Pin individually as Push-Pull and no Pull-Up/Down
	GPIO_InitTypeDef GPIO_InitStructRX;


	GPIO_InitStructRX.GPIO_Pin 							= (USART1_RX_PIN);
	GPIO_InitStructRX.GPIO_Mode	 						= GPIO_Mode_AF;
	GPIO_InitStructRX.GPIO_OType 						= GPIO_OType_PP;
	GPIO_InitStructRX.GPIO_PuPd 						= GPIO_PuPd_NOPULL;
	GPIO_InitStructRX.GPIO_Speed 						= GPIO_Speed_50MHz;
	GPIO_Init(USART1_GPIO_PORT,&GPIO_InitStructRX);


	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(USART1_GPIO_PORT,USART1_TX_SOURCE,USART1_AF);
	GPIO_PinAFConfig(USART1_GPIO_PORT,USART1_RX_SOURCE,USART1_AF);
}


void USART2_LowLevel_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable APB2 peripheral clock for USART1 */
	RCC_APB1PeriphClockCmd(USART2_PERIPHERAL_CLK,ENABLE);

	/* Enable peripheral clock for pins used by USART1 */
	RCC_AHBPeriphClockCmd(USART2_GPIO_CLK,ENABLE);

	// Pins 9(TX) and 10(RX) are used
	GPIO_InitStruct.GPIO_Pin 						= (USART2_TX_PIN | USART2_RX_PIN);
	GPIO_InitStruct.GPIO_Mode	 					= GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType 						= GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd 						= GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed 						= GPIO_Speed_50MHz;
	GPIO_Init(USART2_GPIO_PORT,&GPIO_InitStruct);

	/* The RX and TX pins are now connected to their AF
	 * so that the USART2 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(USART2_GPIO_PORT,USART2_TX_SOURCE,USART2_AF);
	GPIO_PinAFConfig(USART2_GPIO_PORT,USART2_RX_SOURCE,USART2_AF);
}

/**
  * @brief  Initialize USART peripheral.
  * @param  (uint32_t) USART baudrate
  * @retval None
  */
void USART1_Init(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStruct;

	USART1_LowLevel_Init(); // Initial configuration for clock and GPIO pins used as TX and RX

	USART_InitStruct.USART_BaudRate 				= baudrate;
	USART_InitStruct.USART_WordLength 				= USART_WordLength_8b;
	USART_InitStruct.USART_StopBits 				= USART_StopBits_1;
	USART_InitStruct.USART_Parity 					= USART_Parity_No;
	USART_InitStruct.USART_Mode 					= (USART_Mode_Tx | USART_Mode_Rx);
	USART_InitStruct.USART_HardwareFlowControl 		= USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStruct);

	USART_Cmd(USART1,ENABLE);
}


void USART2_Init(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStruct;

	USART2_LowLevel_Init(); // Initial configuration for clock and GPIO pins used as TX and RX

	USART_InitStruct.USART_BaudRate 				= baudrate;
	USART_InitStruct.USART_WordLength 				= USART_WordLength_8b;
	USART_InitStruct.USART_StopBits 				= USART_StopBits_1;
	USART_InitStruct.USART_Parity 					= USART_Parity_No;
	USART_InitStruct.USART_Mode 					= (USART_Mode_Tx | USART_Mode_Rx);
	USART_InitStruct.USART_HardwareFlowControl 		= USART_HardwareFlowControl_None;
	USART_Init(USART2,&USART_InitStruct);

	USART_Cmd(USART2,ENABLE);
}




/**
  * @brief  Initialize USART peripheral with default configuration.
  * @param  None
  * @retval None
  */
void USART1_DeInit()
{
	USART1_Init(USART1_DEFAULT_BAUDRATE);
}

void USART2_DeInit()
{
	USART2_Init(USART1_DEFAULT_BAUDRATE);
}

/**
  * @brief  Configure and enable USART1 rx interrupt.
  * @param  None
  * @retval None
  */
void USART1_EnableRxInterrupt()
{
//	NVIC_InitTypeDef NVIC_InitStruct;

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE); //enable the USART1 receive interrupt

//	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_EnableIRQ(USART1_IRQn);
//	NVIC_Init(&NVIC_InitStruct);
}


void USART2_EnableRxInterrupt()
{
	NVIC_InitTypeDef NVIC_InitStruct;

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE); //enable the USART1 receive interrupt

	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

/**
  * @brief	Send a byte by the USART1.
  * @param  uint8_t byte
  * @retval None
  */
void USART1_PutByte(uint8_t byte)
{
	while(!USART_GetFlagStatus(USART1,USART_ISR_TC));
	USART_SendData(USART1, byte);
}

void USART2_PutByte(uint8_t byte)
{
	while(!USART_GetFlagStatus(USART2,USART_ISR_TC));
	USART_SendData(USART2,byte);
}

/**
  * @brief	Send a string by the USART1.
  * @param  char[] string
  * @retval None
  */
void USART1_PutString(volatile char *string)
{
	while(*string)
	{
		USART1_PutByte(*string++);
	}
}

void USART2_PutString(volatile char *string)
{
	while(*string)
	{
		USART2_PutByte(*string++);
	}
}

/**
  * @brief	Send a array message by the USART1.
  * @param  uint8_t *byte
  * @retval None
  */
void USART1_SendMessage(uint8_t *msg)
{
	while(*msg)
	{
		USART1_PutByte(*msg++);
	}
}

void USART2_SendMessage(uint8_t *msg)
{
	while(*msg)
	{
		USART2_PutByte(*msg++);
	}
}



uint8_t GetStatusRx(void){

	return statusRx;

}







/**
  * @brief	Handler for USART1 rx interrupt.
  * @param  None
  * @retval None
  */
struct buf_st {
  unsigned int in;                                // Next In Index
  unsigned int out;                               // Next Out Index
  char buf [RBUF_SIZE];                           // Buffer
};


static struct buf_st rbuf = { 0, 0, };
#define SIO_RBUFLEN ((unsigned short)(rbuf.in - rbuf.out))

static struct buf_st tbuf = { 0, 0, };
#define SIO_TBUFLEN ((unsigned short)(tbuf.in - tbuf.out))

static unsigned int tx_restart = 1;
/*
void USART1_IRQHandler(void)
{

	/*volatile unsigned int IIR;
	struct buf_st *p;
	 */
	//if(USART_GetITStatus(USART1,USART_IT_RXNE))
//	if (USART1->ISR & USART_FLAG_RXNE)
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//	{
//		// TODO USART message reception
//
////		    IIR = USART1->ISR;
////		if (USART1->ISR & USART_FLAG_RXNE) {                  // read interrupt
////		      USART1->ISR &= ~USART_FLAG_RXNE;	          // clear interrupt
//
//
//		      statusRx = 10;
//		      USART2_SendMessage(statusRx);
//
//	}
		    /*
		      p = &rbuf;

		      if (((p->in - p->out) & ~(RBUF_SIZE-1)) == 0) {

		        p->buf [p->in & (RBUF_SIZE-1)] = (USART1->TDR & 0x1FF);
		        p->in++;

		      }
		*/


/*
		    if (USART1->ISR & USART_FLAG_TXE) {
		      USART1->ISR &= ~USART_FLAG_TXE;	          // clear interrupt

		      p = &tbuf;

		      if (p->in != p->out) {
		        USART1->TDR = (p->buf [p->out & (TBUF_SIZE-1)] & 0x1FF);
		        p->out++;
		        tx_restart = 0;
		      }
		      else {
		        tx_restart = 1;
				USART1->CR1 &= ~USART_FLAG_TXE;		      // disable TX interrupt if nothing to send

		      }
		    }

     */



//	}*/
//}


void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		// TODO USART message reception
	}
}

/*******************************END OF FILE************************************/
