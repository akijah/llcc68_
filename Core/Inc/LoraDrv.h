#ifndef __LORADRV_H
#define __LORADRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"


uint16_t SetBaud (uint16_t *val);
void StartLoraTask(void const * argument);
int Init_Events ( void );
#ifdef __cplusplus
}
#endif

#endif
