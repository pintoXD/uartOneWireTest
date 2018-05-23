/**
  ******************************************************************************
  * @file    main.c
  * @author  thiagosilva-trix
  * @version V1.0.0
  * @date    22 de jun de 2017
  * @brief   The main program body.
  ******************************************************************************   
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
//#include <misc.h>
#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


void delayMicroseconds(uint32_t value);
void OW_ReadBytes(void);
void OW_WriteBytes(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  A simple delay generate function.
  * @param  None
  * @retval None
  */
void Delay(int counter)
{
  int i;

  for(i = 0; i < counter*1000; i++);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */


void InitializeTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 7;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 65535;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &timerInitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

//
//void InitializeLEDs()
//{
////    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//
//    GPIO_InitTypeDef gpioStructure;
//    gpioStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
//    gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
//    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOD, &gpioStructure);
//
//    GPIO_WriteBit(GPIOD, GPIO_Pin_12 | GPIO_Pin_13, Bit_RESET);
//}
uint16_t rcvAux = 0;
uint8_t size = 8;

uint16_t brrValue;
uint16_t brrValue2;
uint8_t rcvBuff[8];
uint8_t flagWrite = 0;

//int i;

int main(void)
{
	/* Initialize USART1 with default params:
		8b,1 stop-bit,none parity, no flow-control,115200
	*/
//	uint16_t teste = 0;

	for( int i = 0; i < size; i++){
		rcvBuff[i] = 0;
	}

//	USART1_DeInit();
	USART1_Init(9600);

	InitializeTimer();

	brrValue = USART1->BRR;
	USART2_DeInit();
	USART1_EnableRxInterrupt();
//	USART1_EnableTxInterrupt();

	USART_ReceiveData(USART1);

	 while(1)
	 {


			USART1_PutByte(0xF0);



			 if(rcvAux == 0xE0){
	//			 USART2_PutString("Pegou ein!\r\n");
	//			 Delay(1);
				 OW_WriteBytes();
//				 delayMicroseconds(15);
//				 OW_ReadBytes();

			  if(rcvBuff[0] != 0){
	//			  USART2_PutString("Lai vai: \r\n");
						for( int i = 0; i < size; i++){

							USART2_PutByte(rcvBuff[i]);
	//						USART2_PutString(" ");

							rcvBuff[i] = 0;
						}



					}
			  USART_Cmd(USART1, DISABLE);
			  USART1_Init(9600);
	//		  brrValue = USART1->BRR;

	//		  USART2_PutString("New BR 9600: ");
	//		  USART2_PutByte(USART1->BRR);

			 }



	//		if(rcvAux == 0xE0)
	//			USART2_PutString("MAH OE\r\n");

		}
}



void delayMicroseconds(uint32_t value){
	TIM3->CNT = 0;
	while(TIM3->CNT < value);

}




void OW_WriteBytes(void){


    uint8_t bit;
    uint8_t byte = 0x33;
    uint8_t count;
    flagWrite = 1;
//    uint8_t bit, byte;
    flagWrite = 0;
    uint8_t masterPattern = 0xFF;
    uint8_t newByte;
//    uint8_t count;


    USART_Cmd(USART1, DISABLE);
    USART1_Init(115200);
    brrValue2 = USART1->BRR;//Troca de BR, porque é o pedido pelo iButton para leitura
//    USART_ReceiveData(USART1);
	for(bit = 0; bit < 8; bit++)
	 {



	  if((byte & 0x01) == 1){
		  USART1_PutByte(0xFF);
//		  USART2_PutString("Aqui");
//		  USART2_PutByte(rcvAux);
	  }

	  else if ((byte & 0x01) == 0){
		  USART1_PutByte(0x00);
//		  USART2_PutString("Aqui");
//	  	  USART2_PutByte(rcvAux);
	  }

	    byte >>= 1;

//	    for(count = 0; count < 250; count++);
//	    delayMicroseconds(15);

//	    Delay(1);
	 }






	for(byte = 0; byte < size; byte++){//Lê os 8 bytes do endereço

	             newByte = 0;

	                for(bit = 0; bit < 8; bit++){//Lê bit a bit de cada byte

	                        newByte >>= 1;
	                        rcvAux = 0;
	                        USART1_PutByte(masterPattern);

	//                        rcvAux = USART_ReceiveData(USART1);


	                   /*    if(i <= 0 ){

	                                // printf("End read\n");
	                                return byte;
	                        }
	                   */

	                        if(rcvAux == 0xFF){
	                        	/*Problema aqui. iButton sempre tá mandando um FF*/
	//                        	USART2_PutString("Entrou no IF");
	                        	//Pego só o 0xFF porque os demais valores represnetam um bit 0 na leitura
	                        	//POdem, assim, ser descartados

	                             newByte |= 0x80; //Configura o MSB do byte;
	                        }

	//                        else{
	//                        	USART2_PutString("Diff");
	//                        }


//	                        delayMicroseconds(12);

	//                        Delay(1);


	                 }

	//                if(newByte == 0xFE){
	//                	newByte = 0;
	//                	byte  = 0;
	//                }
	//
	//                else
	                	rcvBuff[byte] = newByte;
//	                delayMicroseconds(15);

	        }






}




void OW_ReadBytes(void){
//    int i;
    uint8_t bit, byte;
    flagWrite = 0;
//    int fd;
//    uint8_t rcvBuff[8];
//    int len;
    uint8_t masterPattern = 0xFF;
    uint8_t newByte;
    uint8_t count;





     for(byte = 0; byte < size; byte++){//Lê os 8 bytes do endereço

             newByte = 0;

                for(bit = 0; bit < 8; bit++){//Lê bit a bit de cada byte

                        newByte >>= 1;
                        rcvAux = 0;
                        USART1_PutByte(masterPattern);

//                        rcvAux = USART_ReceiveData(USART1);


                   /*    if(i <= 0 ){

                                // printf("End read\n");
                                return byte;
                        }
                   */

                        if(rcvAux == 0xFF){
                        	/*Problema aqui. iButton sempre tá mandando um FF*/
//                        	USART2_PutString("Entrou no IF");
                        	//Pego só o 0xFF porque os demais valores represnetam um bit 0 na leitura
                        	//POdem, assim, ser descartados

                             newByte |= 0x80; //Configura o MSB do byte;
                        }

//                        else{
//                        	USART2_PutString("Diff");
//                        }


                        delayMicroseconds(12);

//                        Delay(1);


                 }

//                if(newByte == 0xFE){
//                	newByte = 0;
//                	byte  = 0;
//                }
//
//                else
                	rcvBuff[byte] = newByte;
                delayMicroseconds(15);
//                delayMicroseconds(60);
                //                for(count = 0; count <= 42; count++){
//                                      }
//                 for(count = 0; count <= 1; count++){
//                 }
//                USART2_PutString("Aqui ");
//                USART2_PutByte(byte);
//                USART2_PutString(" = ");
//                USART2_PutByte(newByte);
//                USART2_PutString("\r\n");

        }

//        return byte;





}





void USART1_IRQHandler(void){

//	USART2_PutString("OE:\r\n");
//	USART2_PutString((char) USART_IT_RXNE);

//	if(USART_GetITStatus(USART1, USART_IT_TXE) == SET){
////		for(uint8_t count = 0; count < 1; count++);
////		USART2_PutString("TC");
//		delayMicroseconds(10);
//		rcvAux = USART_ReceiveData(USART1);
//
//	}


	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
//		// TODO USART message reception

				if(USART1->BRR == brrValue){
//					delayMicroseconds(10);
					rcvAux = USART_ReceiveData(USART1);
//					USART2_PutString("rcvAux brrValue: ");
//					USART2_PutByte(rcvAux);
//					USART2_PutString("\r\n");
	}
			   else if((USART1->BRR == brrValue2)){
//				   	rcvAux = USART_ReceiveData(USART1);


//					if(flagWrite == 0){
//						delayMicroseconds(15);
//					}


//					uint16_t rcvAuxTeste = USART_ReceiveData(USART1);
//					delayMicroseconds(15);
					rcvAux = 0;
//					if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//						USART2_PutString("OPA!");

					rcvAux = USART_ReceiveData(USART1);
//					rcvAux = rcvAux ^ 1;

//					if(flagWrite == 0){
//						USART2_PutByte(rcvAux);
//					}

//					USART2_PutString("rcvAux brrValue2: ");
//					USART2_PutByte(rcvAux);
//					USART2_PutString("\r\n");
				}
	}



	if(USART_GetFlagStatus(USART1, USART_IT_ORE) == SET){
//		USART2_PutString("ORE");
	}

}


/*******************************END OF FILE************************************/
