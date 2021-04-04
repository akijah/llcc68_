
#include "main.h"
#include "maincpp.h"
#include "stdio.h"
#include "string.h"

#include "cmsis_os.h"
#include "peripheral.h"
#include "parse.h"

#ifdef __cplusplus
extern "C" {
#endif





//Add to stm32f1xx_hal_dma.h


static uint8_t RXBuf1[MAX_RXCYCL_BUF];
static uint8_t const *rx1_tail_ptr;
extern UART_HandleTypeDef huart1;


//UART1 Cyclic Buf------------------------------------------------------------------------------------------
static char GetRX1Buf(void)
{  uint8_t const * head = RXBuf1+MAX_RXCYCL_BUF - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	 uint8_t const * tail = rx1_tail_ptr;
	// uint32_t  ll1 = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	 char c;
	  if(head!=tail)
		{ c =  *rx1_tail_ptr++;
			if(rx1_tail_ptr>=RXBuf1 +MAX_RXCYCL_BUF) rx1_tail_ptr-=MAX_RXCYCL_BUF;
			return c;
		}
		else return 0;
}
//------------------------------------------------------------------------------------------------------
static void ClrRX1Buf(void)
{	memset(RXBuf1,0,MAX_RXCYCL_BUF);
	__HAL_DMA_SET_COUNTER(huart1.hdmarx,MAX_RXCYCL_BUF);
	rx1_tail_ptr=RXBuf1;

}
//------------------------------------------------------------------------------------------------------



void StartCliTask(void const * argument)
{
   char c;
   static char p1;
   char R[200];
   uint8_t R_Ptr=0;
   HAL_UART_Receive_DMA(&huart1,RXBuf1,MAX_RXCYCL_BUF);
   ClrRX1Buf();
   printf("Init CliTask ok\n");
   p1=0;
   for(;;)
   {	 c= GetRX1Buf();
        if(c!='\0')
        { //if(((c=='.')||(c==','))&&(Cc==0)) {Cc=c; continue;};
		   //if(Cc==0) {cliProcess((uint8_t*)&c); continue;}
		   if((c!='\r')&&(c!='\n')&&(c!=';'))
			 { if(p1==0)
				 {  if((uint8_t)c<0x80)
				 	  { //To UpCase
							// if(((uint8_t)c>=0x61)&&((uint8_t)c<=0x7A)) c-=0x20;
							 R[R_Ptr++]=c;
						}
						else
						{ p1=c;
						}
				 }
				 else
				 { //To UpCase

					// if(((uint8_t)p1==0xD0)&&((uint8_t)c>=0xB0)&&((uint8_t)c<=0xBF)) c-=0x20;
					// else
					// if(((uint8_t)p1==0xD1)&&((uint8_t)c>=0x80)&&((uint8_t)c<=0x8F))	{p1=0xD0; c+=0x20;}
					// else
					// if(((uint8_t)p1==0xD1)&&((uint8_t)c==0x91)) {p1=0xD0; c=0x81;};
					 R[R_Ptr++]=p1;
					 R[R_Ptr++]=c;
					 p1=0;
				 };
				 continue;
			};
			if(((c=='\r')||(c==';'))&&(R_Ptr>0))
			{  R[R_Ptr++]='\r';
				 R[R_Ptr++]=0;
				 CmdPerform((char*)R);
				 R_Ptr=0;
			}

		 }
		 osDelay(100);
	 }

}






















/*void maincpp()
{
	HAL_UART_Receive_IT(&huart1, &uart1rxbyte, 1);
	SX1276_Init();
	bool rx = !isTX();


	if (rx)
	{//RX-MODE
		GoReceive();
	}

	uint32_t prevtick = HAL_GetTick();
	uint32_t currtick, tickdelta;
	uint32_t timeout = 0;

	while (1)
	{
		__WFE();
		currtick = HAL_GetTick();
		tickdelta = currtick - prevtick;
		prevtick = currtick;

		timeout += tickdelta;

		if (rx)
		{ //RX-Mode
			if (Flag)
			{ //Has got received Data from RFM
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				rx_bytes = Receive();
				//HAL_Delay(200); // for LED

		  	  	Flag = 0;
		  	  	Transmit(RX_BUF, sizeof(RX_BUF));
		  	  	uint32_t waitcnt;
		  	  	waitcnt = WaitFor(Flag, 1000);
		  	  	if (!waitcnt) // could not send reply
		  	  	{
		  	  		GoStandby();
		  	  	}

		  	  	if (RX_BUF[0] == 'S') // set received parameters
		  	  	{
		  	  		;
		  	  	}

				Flag = 0;
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				GoReceive();
				timeout = 0;
			}
			else
			{
				if (timeout > 10000)
				{
					GoReceive();
					timeout = 0;
				}
			}
		} // rx mode
		else
		{ // TX-MODE Send 1 time in 3 seconds
			if (timeout >= 3000)
			{
				i++;
		  	  	TX_BUF[13] = i;

		  	  	Flag = 0;
		  	  	Transmit(TX_BUF, sizeof(TX_BUF));

		  	  	uint32_t waitcnt;
		  	  	waitcnt = WaitFor(Flag, 1000);
		  	  	if (waitcnt)
		  	  	{
		  	  		//SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
		  	  		timeout -= 3000;
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		  	  		Flag = 0;
		  	  		GoReceive();
		  	  		waitcnt = WaitFor(Flag, 1000);
			  	  	if (waitcnt)
			  	  	{
			  	  		rx_bytes = Receive();
			  	  		if (TX_BUF[0] == 'S') // set sent parameters
			  	  		{
			  	  			;
			  	  		}
			  	  	}
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		  	  	}
		  	  	else
		  	  	{
		  	  		GoStandby();
		  	  		// error sending
		  	  	}
			}
		} // tx mode
	} // main while
}
*/
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  if (huart == &huart1)
  {
	  //rxqueue.push(uart1rxbyte);
	  HAL_UART_Receive_IT(&huart1, &uart1rxbyte, 1);
  }
}
*/
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  Flag = 1;
}
*/
//------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif
