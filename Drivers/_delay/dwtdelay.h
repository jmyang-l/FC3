#ifndef DELAY__DWTDELAY_H_
#define DELAY__DWTDELAY_H_

#include "main.h"
#include <stdint.h>

void DWT_Init(void);
void measure_my_func(void (*f)(void));
void measure_my_func_1x(void (*f)(int),int a);



#endif /* DELAY__DWTDELAY_H_ */
