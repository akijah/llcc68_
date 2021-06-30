#ifndef __LORADRV_H
#define __LORADRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
/*
 		llcc68_reset(&llcc68);
		llcc68_hal_wakeup(&llcc68);

		1.При переходе в sleep 500mks не доступно
		2. При warm режиме можно сохр настройки в регистре хранения

		llcc68_set_sleep( &llcc68,, const llcc68_sleep_cfgs_e::LLCC68_SLEEP_CFG_COLD_START);
		llcc68_set_sleep( &llcc68,, const llcc68_sleep_cfgs_e::LLCC68_SLEEP_CFG_WARM_START);

		llcc68_set_standby(&llcc68, llcc68_standby_cfgs_e::LLCC68_STANDBY_CFG_RC);
		llcc68_set_standby(&llcc68, llcc68_standby_cfgs_e::LLCC68_STANDBY_CFG_XOSC);

		llcc68_set_fs(&llcc68);//for test


		llcc68_set_reg_mode(&llcc68, llcc68_reg_mods_e::LLCC68_REG_MODE_LDO);
		llcc68_set_reg_mode(&llcc68, llcc68_reg_mods_e::LLCC68_REG_MODE_DCDC);

		llcc68_cal( &llcc68, llcc68_cal_mask_e::... ) //калибровка полная 3,5мс
		llcc68_cal_img(&llcc68 , const uint32_t freq_in_hz )

		llcc68_set_rx_tx_fallback_mode( &llcc68 , llcc68_fallback_modes_e::LLCC68_FALLBACK_STDBY_RC); //Default
        llcc68_set_rx_tx_fallback_mode( &llcc68 , llcc68_fallback_modes_e::LLCC68_FALLBACK_STDBY_XOSC);
        llcc68_set_rx_tx_fallback_mode( &llcc68 , llcc68_fallback_modes_e::LLCC68_FALLBACK_FS);

        llcc68_set_rf_freq( &llcc68, 868000000L);
        llcc68_set_pkt_type(&llcc68, llcc68_pkt_types_e::LLCC68_PKT_TYPE_LORA);//LLCC68_PKT_TYPE_GFSK
        llcc68_get_pkt_type( &llcc68, llcc68_pkt_type_t* pkt_type )



		llcc68_set_buffer_base_address( &llcc68,  tx_base_address, rx_base_address );

//---------------------------------------------------------------------------------------
		llcc68_write_register( &llcc68 , address, buffer,size );
		llcc68_read_register ( &llcc68 , address, buffer,size );
		llcc68_write_buffer( &llcc68 , offset, buffer,size ); //256byte
		llcc68_read_buffer ( &llcc68 , offset, buffer,size );
//----------------------------------------------------------------------------------------------
DIO&IRQ
		llcc68_set_dio_irq_params( &llcc68, irq_mask, dio1_mask,dio2_mask,dio3_mask ); //llcc68_irq_masks_e
		llcc68_get_irq_status(&llcc68, &irq_status);
		llcc68_clear_irq_status(&llcc68, irq_status);
		llcc68_get_and_clear_irq_status( &llcc68, &irq_status )
		llcc68_set_dio2_as_rf_sw_ctrl(&llcc68, true );
		llcc68_set_dio3_as_tcxo_ctrl( &llcc68, const llcc68_tcxo_ctrl_voltages_t tcxo_voltage,const uint32_t timeout )
//-----------------------------------------------------------------------------------
TX		llcc68_set_tx(&llcc68, timeout);
		Timeout duration = Timeout * 15.625 μs
		24bit 262sec max
		0 - no timeout


			llcc68_pa_cfg_params_t params;
			params.device_sel = 0;
	//		params.hp_max = 2;          // output power
	//		params.pa_duty_cycle = 2;   // is +14 dBm
			params.hp_max = 7;          // output power
			params.pa_duty_cycle = 4;   // is +22 dBm
			params.pa_lut = 1;
			llcc68_set_pa_cfg(&llcc68, &params);
			llcc68_set_pa_cfg(&llcc68 , const llcc68_pa_cfg_params_t* params )

		llcc68_set_tx_params(&llcc68, 14, llcc68_ramp_time_e::LLCC68_RAMP_3400_US); -9-+22 dBm
		RampTime- 10,20,40,80,200,800,1700,3400

		125 SF=5,6,7,8,9
		250 SF=5,6,7,8,9,10
		500 SF=5,6,7,8,9,10,11
		params.bw = llcc68_lora_bw_e::LLCC68_LORA_BW_125;
		params.sf = llcc68_lora_sf_e::LLCC68_LORA_SF5;
		params.cr = llcc68_lora_cr_e::LLCC68_LORA_CR_4_5;
		params.ldro = 0;  //Low DataRate Optimization 0или 1
		llcc68_set_lora_mod_params(&llcc68, &params);

		llcc68_pkt_params_lora_t params;
		params.crc_is_on = true;
		params.invert_iq_is_on = false; //??
		params.preamble_len_in_symb = 100; //1-65535
		params.header_type = llcc68_lora_pkt_len_modes_e::LLCC68_LORA_PKT_EXPLICIT; //переменная длина
		params.pld_len_in_bytes = 16; //макс длина данных
		llcc68_set_lora_pkt_params(&llcc68, &params);

		llcc68_set_lora_sync_word( &llcc68, const uint8_t sync_word)


	    llcc68_set_tx_cw(  &llcc68 )  SetTxContinuousWave тестовая генерация RF Tone
		llcc68_set_tx_infinite_preamble(&llcc68 ) тестовая генерация преамбулы



//-----------------------------------------------------------------------------------
RX		llcc68_set_rx(&llcc68, timeout);
		0 - no timeout
		0xFFFFFF -Rx Continuous mode
		 llcc68_stop_timer_on_preamble(&llcc68 ,true ); //true-останавливать таймер timeout если обнаружена преамбула (может застрять в RX mode)
		 	 	 	 	 	 	 	 	 	 	 	 	 //false -Sync Word or Header detection (default)
		llcc68_set_rx_duty_cycle( &llcc68 , rx_time_in_ms,sleep_time_in_ms ); //24bit -time
		//Tpreamble + Theader ≤ 2 * rxPeriod + sleepPeriod


		llcc68_set_cad(  &llcc68 )  - режим поиска Lora преамбулы


		llcc68_cad_param_s params;
        params.cad_symb_nb=llcc68_cad_symbs_e::LLCC68_CAD_01_SYMB; //!< CAD number of symbols 1,2,4,8,16
        params.cad_detect_peak=uint8;  //!< CAD peak detection
        params.cad_detect_min=uint8;   //!< CAD minimum detection
        params.cad_exit_mode=llcc68_cad_exit_modes_e::LLCC68_CAD_ONLY;    // LLCC68_CAD_RX LLCC68_CAD_LBT  !< CAD exit mode
        params.cad_timeout=uint32;      //!< CAD timeout value

		llcc68_set_cad_params( &llcc68, &params )

		llcc68_cfg_rx_boosted( &llcc68, true);

		llcc68_set_lora_symb_nb_timeout( &llcc68, nb_of_symbs ) количества символов, которые будут использоваться для проверки правильности
прием пакета.

//------------------------------------------------------------------------------------------
status
llcc68_chip_status_t radio_status;
llcc68_get_status(&llcc68, &radio_status);

 radio_status.cmd_status= llcc68_cmd_status_e::LLCC68_CMD_STATUS_DATA_AVAILABLE
                          llcc68_cmd_status_e::LLCC68_CMD_STATUS_CMD_TIMEOUT
                          llcc68_cmd_status_e::LLCC68_CMD_STATUS_CMD_PROCESS_ERROR
                          llcc68_cmd_status_e::LLCC68_CMD_STATUS_CMD_EXEC_FAILURE
                          llcc68_cmd_status_e::LLCC68_CMD_STATUS_CMD_TX_DONE
 radio_status.chip_mode=  llcc68_chip_modes_e::LLCC68_CHIP_MODE_UNUSED
                          llcc68_chip_modes_e::LLCC68_CHIP_MODE_RFU
                          llcc68_chip_modes_e::LLCC68_CHIP_MODE_STBY_RC
                          llcc68_chip_modes_e::LLCC68_CHIP_MODE_STBY_XOSC
                          llcc68_chip_modes_e::LLCC68_CHIP_MODE_FS
                          llcc68_chip_modes_e::LLCC68_CHIP_MODE_RX
                          llcc68_chip_modes_e::LLCC68_CHIP_MODE_TX

  llcc68_rx_buffer_status_t rx_buffer_status;
  llcc68_get_rx_buffer_status(&llcc68, &rx_buffer_status);

   rx_buffer_status.pld_len_in_bytes;      //!< Number of bytes available in the buffer
   rx_buffer_status.buffer_start_pointer;  //!< Position of the first byte in the buffer

   llcc68_pkt_status_lora_t pkt_status;
   llcc68_get_lora_pkt_status(&llcc68, &pkt_status);
   pkt_status.rssi_pkt_in_dbm;         //!< RSSI of the last packet Среднее значение RSSI
   pkt_status.snr_pkt_in_db;           //!< SNR of the last packet
   pkt_status.signal_rssi_pkt_in_dbm;  //!< Estimation of RSSI (after despreading)

   llcc68_get_rssi_inst( &llcc68, int16_t* rssi_in_dbm ) мгновенное значение во время приема пакета




    llcc68_stats_lora_t stats;
    llcc68_get_lora_stats( &llcc68, &stats );
    stats.nb_pkt_received;
    stats.nb_pkt_crc_error;
    stats.nb_pkt_header_error;

	llcc68_reset_stats( &llcc68);
	llcc68_errors_mask_t OpErr;
	llcc68_get_device_errors(&llcc68 , &OpErr );
	llcc68_clear_device_errors(&llcc68)


	air_num=llcc68_get_lora_time_on_air_numerator( &pkt_params,&mod_params)
   	air_ms=llcc68_get_lora_time_on_air_in_ms( &pkt_params,&mod_params);



   	 llcc68_get_random_numbers( &llcc68, uint32_t* numbers, unsigned int n )//перез вызовом отключите прерывания



 */

enum event_sig
{
    EV_NONE 	= 0x0000U,
	EV_PUSHBUT1 = 0x0001U,
	EV_PUSHBUT2 = 0x0002U,
	EV_LLCC68	= 0x0004U,
	EV_TEST1	= 0x0008U,
	EV_TEST2	= 0x0010U,
	EV_ALL		= 0x001FU
};




void StartLoraTask(void *argument);
int Init_Events ( void );
#ifdef __cplusplus
}
#endif

#endif
