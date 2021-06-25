/*
 * llcc68_hal.c
 *
 *  Created on: May 19, 2021
 *      Author: User
 */

#include "llcc68_hal.h"

/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
llcc68_hal_status_t llcc68_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
	if ((!command) && (!data)) return LLCC68_HAL_STATUS_ERROR;
	HAL_StatusTypeDef HR = HAL_OK;
	LLCC68_Context_t* ctx = (LLCC68_Context_t*) context;

	HAL_GPIO_WritePin(ctx->NSS_GPIO, ctx->NSS, GPIO_PIN_RESET);

	int timeout = 10000;
	while (HAL_GPIO_ReadPin(ctx->BUSY_GPIO, ctx->BUSY) == GPIO_PIN_SET)
	{
		//HAL_Delay(1);
		timeout--;
		if (!timeout)
		{
			HR = HAL_TIMEOUT;
			break;
		}
	}

	if (HR == HAL_OK)
		if (command)
		{
			HR = HAL_SPI_Transmit(ctx->hspi, (uint8_t*) command, command_length, 10);
		}

	if (HR == HAL_OK)
	{
		if (data)
		{
			HR = HAL_SPI_Transmit(ctx->hspi, (uint8_t*) data, data_length, 10);
		}
	}

	//__disable_irq();
	HAL_GPIO_WritePin(ctx->NSS_GPIO, ctx->NSS, GPIO_PIN_SET);


	timeout = 10000;
	/*
	if (HR != HAL_TIMEOUT)
	{
	while (HAL_GPIO_ReadPin(ctx->BUSY_GPIO, ctx->BUSY) == GPIO_PIN_RESET)
		{
			timeout--;
			if (!timeout)
			{
				HR = HAL_TIMEOUT;
				break;
			}
		}
	}*/
	//__enable_irq();

	if (HAL_OK != HR)
		return LLCC68_HAL_STATUS_ERROR;
	return LLCC68_HAL_STATUS_OK;
}

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
llcc68_hal_status_t llcc68_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
	if ((!command) || (!data)) return LLCC68_HAL_STATUS_ERROR;
	HAL_StatusTypeDef HR = HAL_OK;
	LLCC68_Context_t* ctx = (LLCC68_Context_t*) context;

	HAL_GPIO_WritePin(ctx->NSS_GPIO, ctx->NSS, GPIO_PIN_RESET);

	int timeout = 10000;
	while (HAL_GPIO_ReadPin(ctx->BUSY_GPIO, ctx->BUSY) == GPIO_PIN_SET)
	{
		//HAL_Delay(1);
		timeout--;
		if (!timeout)
		{
			HR = HAL_TIMEOUT;
			break;
		}
	}

	if (HR == HAL_OK)
	{
		HR = HAL_SPI_Transmit(ctx->hspi, (uint8_t*) command, command_length, 10);
	}
/*
	if (HR == HAL_OK)
	{
		uint8_t tmp = 0;
		HR = HAL_SPI_Transmit(ctx->hspi, &tmp, 1, 10);
	}*/

	size_t len = data_length;
	while (HR == HAL_OK)
	{
		uint8_t tmp = 0;
		HR = HAL_SPI_TransmitReceive(ctx->hspi, &tmp, data, 1, 10);
		data++;
		len--;
		if (!len)
			break;
	}

	HAL_GPIO_WritePin(ctx->NSS_GPIO, ctx->NSS, GPIO_PIN_SET);

	//HAL_Delay(1);

	/*
	if (HR != HAL_TIMEOUT)
	{
		timeout = 100000;
		while (HAL_GPIO_ReadPin(ctx->BUSY_GPIO, ctx->BUSY) == GPIO_PIN_RESET)
		{
			timeout--;
			if (!timeout)
			{
				HR = HAL_TIMEOUT;
				break;
			}
		}
	}*/

	if (HAL_OK != HR)
		return LLCC68_HAL_STATUS_ERROR;
	return LLCC68_HAL_STATUS_OK;
}

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
llcc68_hal_status_t llcc68_hal_reset( const void* context )
{
	LLCC68_Context_t* ctx = (LLCC68_Context_t*) context;
	HAL_GPIO_WritePin(ctx->NRST_GPIO, ctx->NRST, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(ctx->NRST_GPIO, ctx->NRST, GPIO_PIN_SET);
	HAL_Delay(20);
	return LLCC68_HAL_STATUS_OK;
}

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
llcc68_hal_status_t llcc68_hal_wakeup( const void* context )
{
	LLCC68_Context_t* ctx = (LLCC68_Context_t*) context;
	HAL_StatusTypeDef HR = HAL_OK;
	HAL_GPIO_WritePin(ctx->NSS_GPIO, ctx->NSS, GPIO_PIN_RESET);

	int timeout = 100;
	while (HAL_GPIO_ReadPin(ctx->BUSY_GPIO, ctx->BUSY) == GPIO_PIN_SET)
	{
		HAL_Delay(1);
		timeout--;
		if (!timeout)
		{
			HR = HAL_TIMEOUT;
			break;
		}
	}
	HAL_GPIO_WritePin(ctx->NSS_GPIO, ctx->NSS, GPIO_PIN_SET);

	if (HAL_OK == HR)
		return LLCC68_HAL_STATUS_OK;
	return LLCC68_HAL_STATUS_ERROR;
}
