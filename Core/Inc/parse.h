#ifndef __PARSER_H
#define __PARSER_H 
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


#define	KEY_VER			0x00
#define	KEY_BAUD		0x01
#define	KEY_TIME		0x02
#define KEY_RESET		0x03
#define	KEY_HELP		0x04
#define	KEY_OUT			0x05
#define	KEY_ON			0x06
#define	KEY_OFF			0x07

#define KEY_NONE		0xFF




#define U(X,Y)  ((X<<8)|Y)

//Lexem result												
#define F_NONE	0x00
#define F_KEY	0x01
#define F_VAL	0x02
#define F_JSON	0x04
												

uint8_t CmdPerform(char *Buf);
uint16_t Parser(char **Buf);

#ifdef __cplusplus
}
#endif
#endif

