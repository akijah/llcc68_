/*
 * maincpp.h
 *
 *  Created on: Mar 14, 2021
 *      Author: Eugene
 */

#ifndef INC_MAINCPP_H_
#define INC_MAINCPP_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define 	WRITE_SINGLE     				0x80
#define 	READ_SINGLE     				0x00

#define REG_LR_PKTSNRVALUE                          0x19
#define REG_LR_PKTRSSIVALUE                         0x1A

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

void maincpp();

#ifdef __cplusplus
}
#endif


#endif /* INC_MAINCPP_H_ */
