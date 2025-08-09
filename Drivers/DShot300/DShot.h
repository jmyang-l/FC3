#ifndef DSHOT300_DSHOT_H_
#define DSHOT300_DSHOT_H_

#include "main.h"
#include <stdint.h>

void DSHOT_Update(uint8_t motor, uint16_t throttle);
void DSHOT_Start(void);



#endif /* DSHOT300_DSHOT_H_ */
