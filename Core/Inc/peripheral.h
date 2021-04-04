#ifndef __PERIPHERAL_H
#define __PERIPHERAL_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f1xx_hal.h"
#define T_USTR "E220 Module Test"



//--------------------------------------------------------------------------------------------------------------
//#define LEDTEST	 
//#define OLD_1WIRE // старая схема подключения датчиков
//#define BEEP_OFF	 
//#define SIMOFF   //Не запускать SIM800	

//#define USE_CC1101	 
//#define USE_OT
//#define USE_RCSWITCH
//#define NEED_CLRRAM	 
//#define USE_IWDG	 
//--------------------------------------------------------------------------------------------------------------	 
	 
#define IsDigit(c)	(((c)>='0')&&((c)<='9'))
#define IsNumber(c)	((((c)>='0')&&((c)<='9'))||((c)=='+'))	 
	 
#define LOG_ON			0x01
#define LOG_SIM			0x02				
#define dbg_printf(__msg, ...)	if(M2_STAT&LOG_ON)  printf(__msg, ##__VA_ARGS__)
#define sim_printf(__msg, ...)	if(M2_STAT&LOG_SIM) printf(__msg, ##__VA_ARGS__)
//#else /* DEBUG */
//#define dbg_printf(__msg, ...)	do{}while(0)	
//#endif /* DEBUG */	 

#define OUTn	 5 
#define INn 	 2

typedef enum 
{		LED1 =0, //
		CS   =1,
		RES  =2,
		TXEN =3,
		RXEN =4,
} Out_TypeDef;
//Buttons
typedef enum 
{
	INT = 0,
	MODE = 1,
	BUT = 2,
} In_TypeDef;

#define BS_ON     	0x01
#define BS_CHG_ON		0x02
#define BS_CHG_LONG 0x04
#define BS_CHG_OFF  0x08

#define B_NONE  	0
#define B_OFF  		1
#define B_PUSH  	2
#define B_LONG  	3
#define IO_SET		1
#define IO_RESET	0

typedef struct
{ uint8_t counter;
  uint8_t state;
  uint16_t pin;	
}  But_TypeDef;

#define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))

#define B_COUNTER_ON	2
#define B_COUNTER_OFF	1
#define B_COUNTER_LONG  30
//----------------------------------------------------------------------------------

#define MAX_RXCYCL_BUF	1000 //Циклический  буфер для UART1 UART2 
#define TEST(X,Y) ((X==Y)||(X==Y+1))

uint8_t FindName(char *S);
uint8_t IsValue(char *S);
uint8_t GetIn(In_TypeDef inn);
uint8_t GetOutState(Out_TypeDef outn);
void OUT_ON(Out_TypeDef outn);
void OUT_OFF(Out_TypeDef outn);
void OUT_TGL(Out_TypeDef outn);
//void Delay_us(uint16_t uSecs);
//void Delay_ms(uint16_t mSecs);
uint8_t KeyFind(char *S,const char **key);

uint32_t GetUID(uint8_t N);
uint16_t GetFlashSize(void);
extern uint8_t M2_STAT;

#ifdef __cplusplus
}
#endif


#endif
