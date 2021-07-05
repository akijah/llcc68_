
#include <string.h>
#include <stdio.h>

#include "ssm.h"
#include "LoraDrv.h"
#include "peripheral.h"
#include "crc.h"
sensorStates state;
uint8_t serial[SERIAL_SIZE];
uint8_t tx_packet[PACKET_SIZE],rx_packet[PACKET_SIZE];
uint8_t myaddr;
uint16_t panelcs; //

//PushButTime

void SS_Init(void)
{
	memset (serial,0,SERIAL_SIZE);
	memset (tx_packet,0,PACKET_SIZE);
	memset (rx_packet,0,PACKET_SIZE);

	myaddr=0;
	panelcs=0;
	state=ssCheckSerial;

}
//------------------------------------------------------------------------------------------------------------
uint8_t SS_CheckSerial(void)
{
   return (serial[0]==SUF_WL);
}
//------------------------------------------------------------------------------------------------------------
uint8_t SS_Register(void)
{   uint8_t rxlen;
	rfm_result res;
	tx_packet[0]=16; //packet length
	tx_packet[1]=S_VERSION; //Protocol Version
	tx_packet[2]=S_TYPE;	 //Sensor Type
	tx_packet[3]=myaddr;	 //Sensor Address
	tx_packet[4]=0;		//LoCounter; 0 - Bind Packet
	tx_packet[5]=0;		//HiCounter;
	memcpy(&tx_packet[6],serial,SERIAL_SIZE); //Payload
	*(uint16_t*)(&tx_packet[14])=gen_crc16(tx_packet, 14,panelcs);
	//Пытаемся передать

	if(RFM_Transmit(tx_packet,16)==RR_OK)
	{
		res=RFM_Receive(rx_packet,&rxlen);


	}
	return 0;
}
//------------------------------------------------------------------------------------------------------------
void SS_Tick(void)
{	uint8_t result;
	if(PushButTime>0) state=ssCheckSerial;

	switch(state)
	{
	    case   ssCheckSerial: //Проверка серийного номера

	    	    if(PushButTime>=50)
	    		{
	    			{  //Генерировать серийный номер заново c crc
	    			   printf("Generate serial\n");
	    			   GetClientID(serial);
	    			   serial[0]=SUF_WL;
	    			   prnbuf(serial,8);

	    			}
	    		}
	    	    PushButTime=0;
	    		if (SS_CheckSerial()) state = ssBlinkHaveNumber;
	    	     else		    	       state = ssButtonWait;
	    	    break;

	    case   ssButtonWait:
	    	 	 printf("Go stendby\n"); //Просыпаемся только по кнопке
	    		 //ResetRTC();
	    	 	 AlarmOff();
	    		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	    		HAL_PWR_EnterSTANDBYMode();
	    		break;


	 //   case   ssGenerateSerial:		break;
	    case   ssBlinkHaveNumber:

	    		for(uint8_t i=0;i<5;i++)
	    		{ OUTOFF(LED);
	    		  HAL_Delay(50);
	    		  OUTON(LED);
	    		  HAL_Delay(250);
	    		}
	    		state=ssRegister;
	    		break;

	    case   ssRegister:

	    		result=SS_Register();
	    		printf("Register result %d\n",result);
	    		while(1){};

	    	   break;
	    case   ssBlinkRegistered:		break;
	    case   ssNormalMode:		break;
	    case   ssSeldomMode:		break;
	}
}







