//#include <stdint.h>
#include "crc.h"
#define CRC16 0x8005
//------------------------------------------------------------------------------------
uint16_t gen_crc16(const uint8_t *data, uint16_t size,uint16_t crc)
{
    uint16_t out = crc;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> (7 - bits_read)) & 1;

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;

    }
    return out;
}
//---------------------------------------------------------------------------------------
