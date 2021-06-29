/*
 * LoraDrv.cpp
 *
 *  Created on: 4 апр. 2021 г.
 *      Author: AKI
 */
#include "LoraDrv.h"
#include "llcc68_hal.h"
#include "llcc68.h"
#include "stdio.h"
#include "peripheral.h"
#include "string.h"
#include "cmsis_os2.h"


extern SPI_HandleTypeDef hspi1;
uint8_t  tx_buf[16],rx_buf[16] ;
int cnt;
LLCC68_Context_t llcc68;
llcc68_irq_mask_t irq_status;
//llcc68_mod_params_lora_t params;
llcc68_chip_status_t radio_status;
llcc68_rx_buffer_status_t rx_buffer_status;
llcc68_pkt_status_lora_t pkt_status;
bool isTX;
osEventFlagsId_t evt_id;                        // идентификатор очереди сообщений

static void Lora_Init(void);
static void Transmit(void);
static void Receive(void);

int Init_Events ( void )
{

  evt_id = osEventFlagsNew (NULL);


  if (evt_id == NULL)
  {
     // Объект Event Flags не создан, обработаем сбой
    return -1;
  }
  return 0;
}



void StartLoraTask(void const * argument)
{	uint32_t flags;
	Lora_Init();

	//Init_Events ();
	isTX=GETIN(MODE_MS);
	printf("Start Lora module in %s mode\n",isTX?"Tx":"Rx");
	for(;;)
	{
	  if(isTX)
	  {	flags=osEventFlagsWait(evt_id,0x000FU,osFlagsWaitAny|osFlagsNoClear,osWaitForever); //osFlagsWaitAny,osFlagsWaitAll,osFlagsNoClear
 		 printf("flags %08x \n",flags);

		Transmit();
		osDelay(1000);
		osEventFlagsClear (evt_id,0x000FU);
	  }
	  else Receive();
	  osDelay(100);

	}



}
//------------------------------------------------------------------------------------------------------------
void Lora_Init(void)
{
	    llcc68_pa_cfg_params_t pa_params;
	    llcc68_mod_params_lora_t mod_params;
	    llcc68_pkt_params_lora_t pkt_params;
	    uint32_t air_num,air_ms;

	    llcc68.hspi = &hspi1;
		llcc68.BUSY = BUSY_Pin;
		llcc68.NRST = NRST_Pin;
		llcc68.NSS = SPI1_NSS_Pin;
		llcc68.BUSY_GPIO = BUSY_GPIO_Port;
		llcc68.NRST_GPIO = NRST_GPIO_Port;
		llcc68.NSS_GPIO = SPI1_NSS_GPIO_Port;

		irq_status = 0;
		llcc68_reset(&llcc68);
		llcc68_hal_wakeup(&llcc68);

		llcc68_set_standby(&llcc68, llcc68_standby_cfgs_e::LLCC68_STANDBY_CFG_XOSC);

		llcc68_set_pkt_type(&llcc68, llcc68_pkt_types_e::LLCC68_PKT_TYPE_LORA);

		llcc68_set_rf_freq(&llcc68, 868000000L);



	   	pa_params.hp_max = 2;          // output power
	   	pa_params.pa_duty_cycle = 2;   // is +14 dBm
		//	pa_params.hp_max = 3;          // output power
		//	pa_params.pa_duty_cycle = 2;   // is +17 dBm
		//	pa_params.hp_max = 5;          // output power
		//	pa_params.pa_duty_cycle = 3;   // is +20 dBm
		//pa_params.hp_max = 7;          	   // output power
		//pa_params.pa_duty_cycle = 4;       // is +22 dBm
		pa_params.device_sel = 0;
		pa_params.pa_lut = 1;
		pa_llcc68_set_pa_cfg(&llcc68, &pa_params);

		//Мощность TX
		llcc68_set_tx_params(&llcc68, 14, llcc68_ramp_time_e::LLCC68_RAMP_3400_US);

		mod_params.bw = llcc68_lora_bw_e::LLCC68_LORA_BW_125;
		mod_params.sf = llcc68_lora_sf_e::LLCC68_LORA_SF5;
		mod_params.cr = llcc68_lora_cr_e::LLCC68_LORA_CR_4_5;
		mod_params.ldro = 0;
		llcc68_set_lora_mod_params(&llcc68, &mod_params);

		//llcc68_get_irq_status(&llcc68, &irq_status);
		pkt_params.crc_is_on = true;
		pkt_params.invert_iq_is_on = false;
		pkt_params.preamble_len_in_symb = 100;
		pkt_params.header_type = llcc68_lora_pkt_len_modes_e::LLCC68_LORA_PKT_EXPLICIT;
		pkt_params.pld_len_in_bytes = 16;
		llcc68_set_lora_pkt_params(&llcc68, &pkt_params);

		air_num=llcc68_get_lora_time_on_air_numerator( &pkt_params,&mod_params);
		air_ms=llcc68_get_lora_time_on_air_in_ms( &pkt_params,&mod_params);
		printf("On air = %d ms [%d]\n",air_ms,air_num);
		//llcc68_get_irq_status(&llcc68, &irq_status);

		int irq_mask = llcc68_irq_masks_e::LLCC68_IRQ_ALL;
		llcc68_get_irq_status(&llcc68, &irq_status);

		llcc68_set_dio_irq_params(&llcc68, irq_mask, irq_mask, 0, 0);

		llcc68_get_irq_status(&llcc68, &irq_status);


		llcc68_get_status(&llcc68, &radio_status);
		llcc68_get_irq_status(&llcc68, &irq_status);

		//adr0x741 0x3444 for Public Network
		//0x1424 for Private Network
		llcc68_set_lora_sync_word( &llcc68, 0x12); //0x34

		 llcc68_set_dio2_as_rf_sw_ctrl(&llcc68, true );
		cnt = 0;



}
//------------------------------------------------------------------------------------------------------------

//GETIN(MODE_MS)

/*
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
/*void Transmit(uint8_t* data, size_t len)
{
	  SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_01);
	  SX1276_WriteSingle(REG_LR_SYNCWORD,0x12);
	  SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_STANDBY|0x80);
	  SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x80);
	  SX1276_WriteBurst( REG_LR_FIFO, data, len);
	  SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_TRANSMITTER|0x80);
}*/


void Transmit(void)
{
	        tx_buf[0] = cnt;
			llcc68_write_buffer(&llcc68, 0, tx_buf, 16);
			OUTON(LED);
			OUTON(TXEN);
			llcc68_set_tx(&llcc68, 1000);

			while (1)
			{
				llcc68_get_status(&llcc68, &radio_status);
				llcc68_get_irq_status(&llcc68, &irq_status);
				if (irq_status != 0)	break;
				osDelay(100);
			}

			if (irq_status)
			{
				llcc68_clear_irq_status(&llcc68, irq_status);
			}

			llcc68_get_status(&llcc68, &radio_status);

			OUTOFF(LED);
			OUTOFF(TXEN);
}

//---------------------------------------------------------------------------------------------------
void Receive(void)
{			llcc68_get_irq_status(&llcc68, &irq_status);
			llcc68_get_status(&llcc68, &radio_status);

			OUTON(RXEN);
			llcc68_clear_irq_status(&llcc68, llcc68_irq_masks_e::LLCC68_IRQ_ALL);
			llcc68_get_irq_status(&llcc68, &irq_status);
			llcc68_clear_irq_status(&llcc68, llcc68_irq_masks_e::LLCC68_IRQ_RX_DONE);
			llcc68_get_irq_status(&llcc68, &irq_status);
			llcc68_set_rx(&llcc68, 1000);

			while (1)
			{
				llcc68_get_status(&llcc68, &radio_status);
				llcc68_get_irq_status(&llcc68, &irq_status);
				if (irq_status & (llcc68_irq_masks_e::LLCC68_IRQ_TIMEOUT))	break;
				if (irq_status & (llcc68_irq_masks_e::LLCC68_IRQ_RX_DONE))	break;
				if (radio_status.chip_mode != LLCC68_CHIP_MODE_RX) break;
				//if (radio_status.chip_mode);
				osDelay(100);
				cnt++;
			}

			if (irq_status & (llcc68_irq_masks_e::LLCC68_IRQ_RX_DONE))
			{

				OUTON(LED);

				llcc68_clear_irq_status(&llcc68, irq_status);

				llcc68_get_rx_buffer_status(&llcc68, &rx_buffer_status);

				llcc68_get_lora_pkt_status(&llcc68, &pkt_status);
				llcc68_read_buffer(&llcc68, 128, rx_buf, 16);
				printf("cnt = %d rssi=%d dBm SRSSI=%d dBm SNR=%d dB\n",rx_buf[0],
						pkt_status.rssi_pkt_in_dbm,pkt_status.signal_rssi_pkt_in_dbm,pkt_status.snr_pkt_in_db);
				osDelay(10);
				OUTOFF(LED);
			}
			llcc68_get_status(&llcc68, &radio_status);
			OUTOFF(RXEN);
}


void GoReceive()
{
/*	  SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_00);
	  SX1276_WriteSingle(REG_LR_SYNCWORD,0x12);
	  SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
	  SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x00);
	  SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER|0x80);*/
}

/*int Receive()
{
  	rx_bytes = SX1276_ReadSingle(REG_LR_RXNBBYTES);
  	rssi = SX1276_ReadSingle(REG_LR_PKTRSSIVALUE);
  	rssi = rssi;
  	snr= SX1276_ReadSingle(REG_LR_PKTSNRVALUE);
  	snr = snr;
  	SX1276_ReadBurst( REG_LR_FIFO, RX_BUF, rx_bytes);
  	return rx_bytes;
}*/


