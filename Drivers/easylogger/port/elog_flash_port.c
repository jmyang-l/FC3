#include "elog.h"
#include "flash.h"          // 你的 FM25V20A 驱动
#include <string.h>

/* 日志起始地址（最后 64 KiB） */
#define ELOG_FRAM_BASE  0x20000
static uint32_t log_addr = ELOG_FRAM_BASE;

/* 1. 初始化：SPI 已在 Init_FM25Vx() 里完成，这里空实现即可 */
ElogErrCode elog_flash_port_init(void) { return ELOG_NO_ERR; }

/* 2. 写日志：简单追加即可，F-RAM 无需擦除 */
void elog_flash_port_output(const char *log, size_t size)
{
#ifdef ELOG_FLASH_ENABLE  // 当定义 ELOG_FLASH_ENABLE 时才执行写入
    BSP_FM25V20A_Write((uint8_t *)log, log_addr, size);
    log_addr += size;
#else  // 未定义时可以空实现或做其他处理（如仅打印调试信息）
    // 可选：添加调试提示，或直接留空
    // printf("ELOG_FLASH_DISABLED: %.*s\n", (int)size, log);
#endif
}

/* 3. 加解锁：关中断即可 */
void elog_flash_port_lock(void)   { __disable_irq(); }
void elog_flash_port_unlock(void) { __enable_irq();  }
