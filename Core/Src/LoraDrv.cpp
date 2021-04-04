/*
 * LoraDrv.cpp
 *
 *  Created on: 4 апр. 2021 г.
 *      Author: AKI
 */
#include "LoraDrv.h"


static uint16_t Baudrate;

uint16_t SetBaud (uint16_t *val)
{
	if(val) Baudrate=*val;
	return Baudrate;
}

/*
uint8_t isTX()
{
	GPIO_PinState s = HAL_GPIO_ReadPin(MODE_IN_GPIO_Port, MODE_IN_Pin);
    if (s == GPIO_PIN_SET) return 1;
    return 0;
}

uint8_t SX1276_WriteSingle(uint8_t command, uint8_t value) 													{//WriteSingle

HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

uint8_t outdata[2];
uint8_t indata[2] = {};
outdata[0] = (WRITE_SINGLE | command);
outdata[1] = value;

HAL_StatusTypeDef hr;

hr = HAL_SPI_TransmitReceive(&hspi1, outdata, indata, 2, 1);

HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

if (hr == HAL_OK)
	return indata[1];
return 0;
}

uint8_t SX1276_ReadSingle(uint8_t command) 																					{//ReadSingle

HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
//while (GPIOA->IDR & MISO){};							//waiting until CC1101 ready

uint8_t outdata[1];
uint8_t indata[1] = {};
HAL_StatusTypeDef hr;

outdata[0] = (READ_SINGLE | command);
hr = HAL_SPI_TransmitReceive(&hspi1, outdata, indata, 1, 1);
if (hr != HAL_OK)
{
 hr = hr;
}
outdata[0] = 0;
hr = HAL_SPI_TransmitReceive(&hspi1, outdata, indata, 1, 1);
if (hr != HAL_OK)
{
 hr = hr;
}

HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

return indata[0];
}

void SX1276_WriteBurst( uint8_t addr, uint8_t *buff, uint8_t size )
{//WriteBurst

HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

uint8_t outdata[1];
uint8_t indata[1] = {};
outdata[0] = (WRITE_SINGLE | addr);
HAL_SPI_TransmitReceive(&hspi1, outdata, indata, 1, 1);

HAL_SPI_Transmit(&hspi1, buff, size, 1);

HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void SX1276_ReadBurst( uint8_t command, uint8_t *buff, uint8_t size )
{//ReadBurst

HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

uint8_t outdata[1];
uint8_t indata[1] = {};
outdata[0] = (READ_SINGLE | command);
HAL_SPI_TransmitReceive(&hspi1, outdata, indata, 1, 1);

HAL_SPI_Receive(&hspi1, buff, size, 1);

HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void SX1276_Init(void)
{//CC1101_Init
 uint8_t qnt, i_temp=0;
 qnt =0;// sizeof (LoRa_config);

HAL_GPIO_WritePin(RFM_RESET_GPIO_Port, RFM_RESET_Pin, GPIO_PIN_RESET);
HAL_Delay(10);
HAL_GPIO_WritePin(RFM_RESET_GPIO_Port, RFM_RESET_Pin, GPIO_PIN_SET);
HAL_Delay(10);

 while (i_temp < qnt)
 {
	//SX1276_WriteSingle(LoRa_config[i_temp], LoRa_config[i_temp+1]);
	i_temp += 2;
 }

  //SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
}

size_t WaitFor(volatile uint8_t& v, size_t waitcnt)
{
	size_t cur_time = HAL_GetTick();
	while(!v)
	{
		__WFE();
		size_t time = HAL_GetTick();
		while (time != cur_time)
		{
			cur_time++;
			waitcnt--;
			if (!waitcnt) return 0;
		}
	}
	return waitcnt;
}

void GoStandby()
{
	 // SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_STANDBY|0x80);
}
*/
void Transmit(uint8_t* data, size_t len)
{
/*	  SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_01);
	  SX1276_WriteSingle(REG_LR_SYNCWORD,0x12);
	  SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_STANDBY|0x80);
	  SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x80);
	  SX1276_WriteBurst( REG_LR_FIFO, data, len);
	  SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_TRANSMITTER|0x80);*/
}

void GoReceive()
{
/*	  SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_00);
	  SX1276_WriteSingle(REG_LR_SYNCWORD,0x12);
	  SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
	  SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x00);
	  SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER|0x80);*/
}

int Receive()
{
  /*	rx_bytes = SX1276_ReadSingle(REG_LR_RXNBBYTES);
  	rssi = SX1276_ReadSingle(REG_LR_PKTRSSIVALUE);
  	rssi = rssi;
  	snr= SX1276_ReadSingle(REG_LR_PKTSNRVALUE);
  	snr = snr;
  	SX1276_ReadBurst( REG_LR_FIFO, RX_BUF, rx_bytes);
  	return rx_bytes;*/
}


