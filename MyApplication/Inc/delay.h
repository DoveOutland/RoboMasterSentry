#ifndef _DELAY_H_
#define _DELAY_H_

#include "main.h"
#include "struct_typedef.h"


extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);

#endif

