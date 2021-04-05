#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "parse.h"
#include "base.h"
#include "peripheral.h"
#include "LoraDrv.h"
#include "rev.h"
const char *engkey[] = {"VER","BAUD","TIME","RESET","HELP","OUT","ON","OFF",NULL};


uint16_t Parser(char **Buf)
{ uint8_t k;
	uint16_t res=0xFFFF;
	
	if(*Buf==NULL) return res;
	k=KeyFind(*Buf,engkey);
	if(k==KEY_NONE) return res;
	res=((uint16_t)k<<8);
	*Buf=strtok(NULL," \r");
	if(*Buf==NULL) return  res|KEY_NONE;
	k=KeyFind(*Buf,engkey);
	res|=k;
	if(k<KEY_NONE) 		*Buf=strtok(NULL," \r");
	return res;
}
//-----------------------------------------------------------------------------------------


uint8_t CmdPerform(char *Buf)
{		uint8_t result=0;//,res_fl,errp;
		uint16_t cmd;
	  char *StrPtr;//,*P,*P2;//,*V;
	 // char S[50], B[20];//,B[20];
		uint8_t u1;
		uint16_t URes;
	  //uint64_t Bb[MAX_SENS];  
	
		//printf("Convert: %s\n",Buf);

		StrPtr=strtok(Buf," \r");
  	    cmd=Parser(&StrPtr);
	
   
	u1=0;URes=0;
	switch (cmd)
		{ 	case U(KEY_VER,KEY_NONE):

					//printf("%s Ver.%s %s %s\n", T_USTR ,REV_NUM,REV_DATE, REV_TIME);
					printf("%s Ver.%s \n", T_USTR ,REV_NUM);


					break;
			 case U(KEY_BAUD,KEY_NONE):
				 	if(StrPtr)	u1=atoi(StrPtr);
				 	else u1=0;
					URes=SetBaud((u1>0)?(&u1):NULL);
			 	    printf("Baudrate= %d\n",URes);
					break;

   		    case 	U(KEY_TIME,KEY_NONE):
					printf("time is...\n");
					break;
			case 	U(KEY_RESET,KEY_NONE):
					printf("reset device...\n");
			 	 	NVIC_SystemReset();
					break;
			case 	U(KEY_HELP,KEY_NONE):
					printf("help,ver,baud,\n");
					break;
			case 	U(KEY_OUT,KEY_ON):
					if(StrPtr)	u1=atoi(StrPtr);
					if((u1>0)&&(u1<=OUTn)) OUT_ON(u1-1);
					break;
			case 	U(KEY_OUT,KEY_OFF):
					if(StrPtr)	u1=atoi(StrPtr);
					if((u1>0)&&(u1<=OUTn)) OUT_OFF(u1-1);
					break;




		};	
		return result;
}

