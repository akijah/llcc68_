#ifndef __CRC_H
#define __CRC_H
#include "stm32f1xx_hal.h"
#ifdef __cplusplus
 extern "C" {
#endif 

uint16_t gen_crc16(const uint8_t *data, uint16_t size,uint16_t crc);
#ifdef __cplusplus
 }
#endif 

#endif

