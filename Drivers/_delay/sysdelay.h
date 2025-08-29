#ifndef DELAY__SYSDELAY_H_
#define DELAY__SYSDELAY_H_

#include "main.h"
#include <stdint.h>

void SysTick_Init_1us(void);
void delay_us(uint32_t us);
void delay_1s(void);

void SysTick_Init_100us(void);

#endif /* DELAY__SYSDELAY_H_ */
