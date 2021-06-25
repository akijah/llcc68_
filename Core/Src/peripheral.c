
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

/*
uint8_t GetOutState(Out_TypeDef outn)
{  assert_param(IS_GPIO_PIN(OUT_PIN[outn])); 
  if ((OUT_PORT[outn]->ODR & OUT_PIN[outn]) != (uint32_t)GPIO_PIN_RESET) return IO_SET;
	else   return IO_RESET;
}*/
//-----------------------------------------------------------------------------------
