
#include "main.h"
#include "maincpp.h"
#include "stdio.h"
#include "string.h"

//#include "cmsis_os2.h"
#include "peripheral.h"
#include "parse.h"
#include "LoraDrv.h"
#ifdef __cplusplus
extern "C" {
#endif





//Add to stm32f1xx_hal_dma.h
//extern osEventFlagsId_t evt_id;
static uint8_t RXBuf1[MAX_RXCYCL_BUF];
static uint8_t const *rx1_tail_ptr;
extern UART_HandleTypeDef huart1;
//extern osThreadId_t LoraTaskHandle;

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
void CliTask(void)
{
}
//----------------------------------------------------------------------------------------------------------------------
void mainloop()
{    char c;
     static char p1;
     char R[200];
     uint8_t R_Ptr=0;
     HAL_UART_Receive_DMA(&huart1,RXBuf1,MAX_RXCYCL_BUF);
     ClrRX1Buf();
     p1=0;

     printf("Init Cli Task ok\n");
 while(1)
 {
  	c= GetRX1Buf();
     if(c=='\0')
     { HAL_Delay(200);
       //return;
     }
     if((c!='\r')&&(c!='\n')&&(c!=';'))
		   { if(p1==0)
			 { if((uint8_t)c<0x80) R[R_Ptr++]=c;
			   	else  p1=c;

			 }
			 else
			 { R[R_Ptr++]=p1;
			   R[R_Ptr++]=c;
			   p1=0;
			 };
			 return;
			};
		 if(((c=='\r')||(c==';'))&&(R_Ptr>0))
			{  R[R_Ptr++]='\r';
				 R[R_Ptr++]=0;
				 CmdPerform((char*)R);
				 R_Ptr=0;
			}
 }
}
//----------------------------------------------------------------------------------------------------------------------
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{	//int32_t fl1,fl2,fl3;
	//osThreadId_t LoraTaskHandle;
	if(GPIO_Pin==GPIO_PIN_6)
	{	// osEventFlagsSet (evt_id, EV_PUSHBUT1);


		// printf("EXT6 %d\n",fl1);
		//llcc68_set_buffer_base_address(&llcc68, 0, 128);
		//Transmit();
		//osDelay(1000);

	}
	if(GPIO_Pin==GPIO_PIN_7)
	{
		//fl1= osEventFlagsGet	(evt_id);


		//osEventFlagsSet (evt_id, EV_LLCC68);
		//fl3= osEventFlagsGet	(evt_id);
		//fl=osSignalSet(LoraTaskHandle, 112);
		//printf("EXT7 %d\n",fl1);


		//osDelay(1000);
	}

}





#ifdef __cplusplus
}
#endif
