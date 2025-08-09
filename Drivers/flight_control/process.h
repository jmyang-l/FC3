#ifndef FLIGHT_CONTROL_PROCESS_H_
#define FLIGHT_CONTROL_PROCESS_H_
#include "main.h"
#include "position_control.h"
#include "position_estimator.h"
#include "attitude_control.h"
#include "attitude_estimator.h"
//#include "attitude_ekf.h"
//外设驱动
#include "BMI088.h"
#include "baro.h"
#include "light_flow.h"
#include "DShot.h"
#include "flash.h"



// 声明 process_main 函数
void process_main(void);





#endif /* FLIGHT_CONTROL_PROCESS_H_ */
