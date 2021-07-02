/*
 * LoraDrv.cpp
 *
 *  Created on: 4 апр. 2021 г.
 *      Author: AKI
 */
#include <string.h>
#include <stdlib.h>

#include "LoraDrv.h"
#include "llcc68_hal.h"
#include "llcc68.h"
#include "stdio.h"
#include "peripheral.h"
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
uint8_t butcnt1;
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




void StartLoraTask(void *argument)
{	uint32_t flags;
	Lora_Init();
	uint32_t TimeDown,TimeUp;
		 //ticks = portMAX_DELAY;
	 //TimeNow=xTaskGetTickCount();
	 //if(timewait==0) TimeEnd=TimeNow+1;
	 //else	 TimeEnd=TimeNow+timewait*1000/ portTICK_PERIOD_MS;



	//Init_Events ();
	isTX=GETIN(MODE_MS);
	//btncnt1=0;
	printf("Start Lora module in %s mode\n",isTX?"Tx":"Rx");
	if(!isTX)
	{	llcc68_get_status(&llcc68, &radio_status);
		if(radio_status.chip_mode ==LLCC68_CHIP_MODE_STBY_RC) //LLCC68_CHIP_MODE_STBY_RC LLCC68_CHIP_MODE_RX
		{ printf("Go rx\n");
		  Receive();
		}

	}

	for(;;)
	{    flags=osEventFlagsWait(evt_id,EV_ALL,osFlagsWaitAny|osFlagsNoClear,osWaitForever); //osFlagsWaitAny,osFlagsWaitAll,osFlagsNoClear
 		 printf("flags %08lx \n",flags);

		if(flags&EV_PUSHBUT1)
 		{

		  if(!GETIN(EXT6_BTN))
		  {	 TimeDown=osKernelGetTickCount();


			  //Transmit();
 		  	  // OUTOFF(LED);
 		  	  // OUTON(TEST1);
		  }
		  else
		  { TimeUp=osKernelGetTickCount();
			printf("tim: %lu %lu %lu\n",TimeUp,TimeDown,TimeUp-TimeDown);

		  }

		  //osDelay(1000);
 		}

		if(flags&EV_LLCC68)
		{
			//llcc68_get_status(&llcc68, &radio_status);
		    llcc68_get_irq_status(&llcc68, &irq_status);
		    printf("irq_status: %04X\n",irq_status);
		    if (irq_status & (llcc68_irq_masks_e::LLCC68_IRQ_TIMEOUT))
		    {
		    	OUTON(LED);
		    	OUTOFF(TEST1);
		    	printf("irq timeout\n");
		    	if(!isTX) Receive();
		    }
		    if (irq_status & (llcc68_irq_masks_e::LLCC68_IRQ_TX_DONE))
		    {	OUTOFF(TXEN);
			    OUTON(LED);
			    OUTOFF(TEST1);
		    	printf("irq tx done\n");
		    }
		    if (irq_status & (llcc68_irq_masks_e::LLCC68_IRQ_RX_DONE))
		    {
			   llcc68_get_rx_buffer_status(&llcc68, &rx_buffer_status);
               printf("rx available:%d start:%d\n",rx_buffer_status.pld_len_in_bytes, rx_buffer_status.buffer_start_pointer);


			   llcc68_get_lora_pkt_status(&llcc68, &pkt_status);
			   llcc68_read_buffer(&llcc68, 128, rx_buf, 16);
			   prnbuf(rx_buf,16);
			   printf("cnt = %d rssi=%d dBm SRSSI=%d dBm SNR=%d dB\n",rx_buf[0],
					pkt_status.rssi_pkt_in_dbm,pkt_status.signal_rssi_pkt_in_dbm,pkt_status.snr_pkt_in_db);
			   OUTON(LED);
			   OUTOFF(TEST1);
			   OUTOFF(RXEN);
			   if(!isTX) Receive();
		    }


		    llcc68_clear_irq_status(&llcc68, irq_status);

		}
		osEventFlagsClear (evt_id,flags);
	  }


		  //osDelay(100);
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

		llcc68_set_standby(&llcc68, llcc68_standby_cfgs_e::LLCC68_STANDBY_CFG_RC);//LLCC68_STANDBY_CFG_XOSC);

		llcc68_set_pkt_type(&llcc68, llcc68_pkt_types_e::LLCC68_PKT_TYPE_LORA);

		llcc68_set_rf_freq(&llcc68, 868000000L);

		llcc68_cal_img(&llcc68 ,  868000000L);

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
		llcc68_set_pa_cfg(&llcc68, &pa_params);

		//Мощность TX
		llcc68_set_tx_params(&llcc68, 14, llcc68_ramp_time_e::LLCC68_RAMP_3400_US);

		mod_params.bw = llcc68_lora_bw_e::LLCC68_LORA_BW_125;
		mod_params.sf = llcc68_lora_sf_e::LLCC68_LORA_SF5;//5
		mod_params.cr = llcc68_lora_cr_e::LLCC68_LORA_CR_4_6;//Если
		mod_params.ldro = 0;
		llcc68_set_lora_mod_params(&llcc68, &mod_params);

		//llcc68_get_irq_status(&llcc68, &irq_status);
		pkt_params.crc_is_on = true;
		pkt_params.invert_iq_is_on = false;
		pkt_params.preamble_len_in_symb = 100;
		pkt_params.header_type = llcc68_lora_pkt_len_modes_e::LLCC68_LORA_PKT_EXPLICIT; //LLCC68_LORA_PKT_IMPLICIT;
		pkt_params.pld_len_in_bytes = 16;
		llcc68_set_lora_pkt_params(&llcc68, &pkt_params);

		air_num=llcc68_get_lora_time_on_air_numerator( &pkt_params,&mod_params);
		air_ms=llcc68_get_lora_time_on_air_in_ms( &pkt_params,&mod_params);
		printf("On air = %ld ms [%ld]\n",air_ms,air_num);
		//llcc68_get_irq_status(&llcc68, &irq_status);

		int irq_mask = llcc68_irq_masks_e::LLCC68_IRQ_ALL;
		//llcc68_get_irq_status(&llcc68, &irq_status);

		llcc68_set_dio_irq_params(&llcc68, irq_mask, irq_mask, 0, 0);

		//llcc68_get_irq_status(&llcc68, &irq_status);


		//llcc68_get_status(&llcc68, &radio_status);
		//llcc68_get_irq_status(&llcc68, &irq_status);

		//adr0x741 0x3444 for Public Network
		//0x1424 for Private Network
		llcc68_set_buffer_base_address( &llcc68,  0, 128 );
		llcc68_set_lora_sync_word( &llcc68, 0x12); //0x34

		 llcc68_set_dio2_as_rf_sw_ctrl(&llcc68, true );
		cnt = 0;



}
//------------------------------------------------------------------------------------------------------------
void Transmit(void)
{			cnt++;
	        tx_buf[0] = cnt;
			llcc68_write_buffer(&llcc68, 0, tx_buf, 16);
			//OUTON(LED);
			OUTON(TXEN);
			llcc68_set_tx(&llcc68, 1000);
			printf("TX:");
			prnbuf(tx_buf,16);
	/*		while (1)
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

			//OUTOFF(LED);
			OUTOFF(TXEN);*/
}

//---------------------------------------------------------------------------------------------------
void Receive(void)
{
	llcc68_status_t res;
		//	llcc68_get_irq_status(&llcc68, &irq_status);
		//	llcc68_get_status(&llcc68, &radio_status);
			OUTOFF(LED);
			OUTON(TEST1);
			OUTON(RXEN);
			//llcc68_clear_irq_status(&llcc68, llcc68_irq_masks_e::LLCC68_IRQ_ALL);
			//llcc68_get_irq_status(&llcc68, &irq_status);
			//llcc68_clear_irq_status(&llcc68, llcc68_irq_masks_e::LLCC68_IRQ_RX_DONE);
			//llcc68_get_irq_status(&llcc68, &irq_status);
			res=llcc68_set_rx(&llcc68, LLCC68_RX_SINGLE_MODE); //Всегда в RX
			/*
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

				//OUTON(LED);

				llcc68_clear_irq_status(&llcc68, irq_status);

				llcc68_get_rx_buffer_status(&llcc68, &rx_buffer_status);

				llcc68_get_lora_pkt_status(&llcc68, &pkt_status);
				llcc68_read_buffer(&llcc68, 128, rx_buf, 16);
				printf("cnt = %d rssi=%d dBm SRSSI=%d dBm SNR=%d dB\n",rx_buf[0],
						pkt_status.rssi_pkt_in_dbm,pkt_status.signal_rssi_pkt_in_dbm,pkt_status.snr_pkt_in_db);
				osDelay(10);
				//OUTOFF(LED);
			}
			llcc68_get_status(&llcc68, &radio_status);
			OUTOFF(RXEN);*/
}




