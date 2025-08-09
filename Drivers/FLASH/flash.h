
#ifndef FLASH_FLASH_H_
#define FLASH_FLASH_H_

#include "main.h"



/*------------------------------  片选宏 ------------------------------*/
/* 假设使用 HAL 库，CS 引脚名为 CS_FRAM_Pin，端口为 CS_FRAM_GPIO_Port */
#define FM25Vx_Enable()          HAL_GPIO_WritePin(CS_FRAM_GPIO_Port, CS_FRAM_Pin, GPIO_PIN_RESET)  /* 拉低选中 */
#define FM25Vx_Disable()         HAL_GPIO_WritePin(CS_FRAM_GPIO_Port, CS_FRAM_Pin, GPIO_PIN_SET)    /* 拉高释放 */

/*------------------------------ 15. 函数返回值 ------------------------------*/
#define W25Qx_OK                ((uint8_t)0x00)
#define W25Qx_ERROR             ((uint8_t)0x01)
#define W25Qx_BUSY              ((uint8_t)0x02)
#define W25Qx_TIMEOUT           ((uint8_t)0x03)

#define FM25V20A_OK   0
#define FM25V20A_ERR  1

#define FM25Qx_TIMEOUT_VALUE 1000

#define FM25V20A_READ_CMD   0x03  // READ 命令码
#define FM25V20A_DUMMY_BYTE 0xFF  // 用于读取时的哑字节

uint8_t BSP_FM25V20A_Init(void);
uint8_t BSP_FM25V20A_WriteEnable(void);
void BSP_FM25V20A_Read_ID(uint8_t *ID);
void FM25V20A_Read(uint32_t address, uint8_t *buffer, uint32_t length);
uint8_t BSP_FM25V20A_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);

void Init_FM25Vx(void);

void FM25Vx_text(void);//测试函数





#endif /* FLASH_FLASH_H_ */
