#include <elog.h>
#include "usart.h"
/**
 * EasyLogger port initialize
 *
 * @return result
 */
ElogErrCode elog_port_init(void) {
    ElogErrCode result = ELOG_NO_ERR;

    /* add your code here */
    
    return result;
}

/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void) {

    /* add your code here */

}

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char *log, size_t size) {
    
    /* add your code here */
	// 1. 串口输出
	HAL_UART_Transmit(&huart4, (uint8_t *)log, size, HAL_MAX_DELAY);
	// 2. 同时写入 Flash
	elog_flash_port_output(log, size);
}


//以下的中断开关为了在 EasyLogger 往串口输出一条完整日志的“开始–结束”之间，临时屏蔽所有中断，防止输出过程被抢占，从而避免日志内容被打断或出现乱序。
/**
 * output lock
 */
void elog_port_output_lock(void) {
    
    /* add your code here */
	__disable_irq();
}

/**
 * output unlock
 */
void elog_port_output_unlock(void) {
    
    /* add your code here */
	__enable_irq();
}

/**
 * get current time interface
 *
 * @return current time
 */
const char *elog_port_get_time(void) {
    
    /* add your code here */
	return "time";
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void) {
    
    /* add your code here */
	return "";
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void) {
    
    /* add your code here */
	return "";
}
