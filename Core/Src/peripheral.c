
#include "peripheral.h"
#include "string.h"


uint8_t M2_STAT ;
extern UART_HandleTypeDef huart1;
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

//-------------------------------------------------------------
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
//----------------------------------------------------------------



const uint16_t  OUT_PIN[OUTn]  = {	GPIO_PIN_13,	GPIO_PIN_4,	GPIO_PIN_0,	GPIO_PIN_10, GPIO_PIN_11};
GPIO_TypeDef*   OUT_PORT[OUTn] = {	GPIOC,		 	GPIOA,		GPIOB,		GPIOB, 		 GPIOB};



const uint16_t  IN_PIN[INn]  = {	GPIO_PIN_1,		GPIO_PIN_1};
GPIO_TypeDef*   IN_PORT[INn] = {	GPIOA,			GPIOB};


//===============================================================================================

void ConvertUp(uint8_t *B,uint8_t len)
{ uint8_t i;
	for(i=0;i<len;i++)
	{ if((B[i]==0xD0)||(B[i]==0xD1)||(B[i]==0x00)) { continue;}
		if((i>0)&&(B[i-1]==0xD0)&&(B[i]>=0xB0)&&(B[i]<=0xBF)) 	B[i]-=0x20;
		else
		if ((i>0)&&(B[i-1]==0xD1)&&(B[i]>=0x80)&&(B[i]<=0x8F)) {B[i-1]=0xD0;B[i]+=0x20;}
    else
		if ((i>0)&&(B[i-1]==0xD1)&&(B[i]==0x91)) {B[i-1]=0xD0;B[i]=0x81;}		
		else
		if((B[i]>=0x61)&&(B[i]<=0x7A)) B[i]-=0x20;	
		//if((S[i]>=0x61)&&(S[i]<=0x7A)) S[i]-=0x20;
		//else
		//if((i<(k-1))&&(S[i+1]==0x04))
    //{	if((S[i]>=0x30)&&(S[i]<=0x4F)) S[i]-=20;
    //  else if(S[i]==0x51) S[i]=0x01;
		//}		
	}	

}


//-------------------------------------------------------------------------------------------
uint8_t KeyFind(char *S,const char **key)
{ uint8_t k,m;
	uint8_t B[20];
	
	if(S==NULL) return 0xFF;
	k=strlen(S);
	if(k==0) return 0xFF;
	if(k>20) k=20;
	memcpy(B,S,20);
	ConvertUp(B,k);	
	m=0;
	while(key[m])
	{   if(memcmp(B,key[m],k)==0)  return m;
		  m++;
	}		
// printf("key:%s(%d) [%d]\n",S,k,m);
	return 0xFF;

}
//---------------------------------------------------------------------------------------------------



uint8_t GetIn(In_TypeDef inn)
{
   if(HAL_GPIO_ReadPin(IN_PORT[inn],IN_PIN[inn])==GPIO_PIN_SET) return IO_SET;
	 else return IO_RESET;
}
//-----------------------------------------------------------------------------------																

uint8_t GetOutState(Out_TypeDef outn)
{  assert_param(IS_GPIO_PIN(OUT_PIN[outn])); 
  if ((OUT_PORT[outn]->ODR & OUT_PIN[outn]) != (uint32_t)GPIO_PIN_RESET) return IO_SET;
	else   return IO_RESET;
}
//-----------------------------------------------------------------------------------
void OUT_ON(Out_TypeDef outn)
{ HAL_GPIO_WritePin(OUT_PORT[outn],OUT_PIN[outn],GPIO_PIN_SET);
	//if(outn<MAX_OUTPUT) 
	//{	 if(!(storeouts&(1<<outn))) dbg_printf("Out%1d on [prev %02X]\n",outn+1,storeouts);
	//		storeouts|=(1<<outn);
	//}	
}
//------------------------------------------------------------------------------------
void OUT_OFF(Out_TypeDef outn)
{ HAL_GPIO_WritePin(OUT_PORT[outn],OUT_PIN[outn], GPIO_PIN_RESET);
	//if(outn<MAX_OUTPUT) 
	//{  
	//	 if(storeouts&(1<<outn)) dbg_printf("Out%1d off[prev %02X]\n",outn+1,storeouts);
	//	 storeouts&=~(1<<outn);
	//}	
}
//--------------------------------------------------------------------------------------	
void OUT_TGL(Out_TypeDef outn)
{ HAL_GPIO_TogglePin(OUT_PORT[outn],OUT_PIN[outn]);
	//if(outn<MAX_OUTPUT) 
	//{	if(storeouts&(1<<outn)){ dbg_printf("Out%1d off[prev %02X]\n",outn+1,storeouts);}
	//	else										{dbg_printf("Out%1d on[prev %02X]\n",outn+1,storeouts);};
	//	storeouts^=(1<<outn);
	
}
//----------------------------------------------------------------------------------------


