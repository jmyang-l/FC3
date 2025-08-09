
#include "flash.h"
#include "spi.h"
#include <stdio.h>


/*--------------------------------------------------
 * 函数：BSP_FM25V20A_Init
 * 作用：初始化 FM25V20A F-RAM，软复位后等待就绪
 * 返回：FM25V20A_OK   初始化成功
 *       FM25V20A_ERR  初始化失败（可选）
 *-------------------------------------------------*/
uint8_t BSP_FM25V20A_Init(void)
{
    /* 1. 软复位：连续发送 0x66 0x99 */
    uint8_t cmd[2] = {0x66, 0x99};
    FM25Vx_Enable();
    HAL_SPI_Transmit(&hspi2, cmd, 2, FM25Qx_TIMEOUT_VALUE);
    FM25Vx_Disable();

    /* 2. FM25V20A 复位完成需要约 100 µs */
    HAL_Delay(1);  // 1 ms 足够覆盖 100 µs

    /* F-RAM 没有 BUSY 位，直接返回成功 */
    return FM25V20A_OK;
}

/*--------------------------------------------------
 * 函数：BSP_FM25V20A_WriteEnable
 * 作用：发送 0x06 置位 WEL（FM25V20A 的写使能）
 * 返回：FM25V20A_OK  成功
 * 备注：FM25V20A 无 BUSY 位，无需等待
 *-------------------------------------------------*/
uint8_t BSP_FM25V20A_WriteEnable(void)
{
    uint8_t cmd = 0x06;              // WREN 指令

    FM25Vx_Enable();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, FM25Qx_TIMEOUT_VALUE);
    FM25Vx_Disable();

    /* F-RAM 写使能立即生效，无 BUSY 位 */
    return FM25V20A_OK;
}


/**
 * @brief 读取 FM25V20A（2-Mbit F-RAM）的 9 字节 JEDEC 设备 ID
 * @param[out] ID 缓冲区，长度必须 ≥ 9 字节
 *        返回值排列：
 *        ID[0..5]：0x7F 0x7F 0x7F 0x7F 0x7F 0x7F（JEDEC 连续码）
 *        ID[6]    ：0xC2（Cypress 厂商 ID）
 *        ID[7]    ：0x25（产品系列码）
 *        ID[8]    ：0x08（容量码：2-Mbit）
 *
 * @note  1. 必须使用 0x9F（RDID）指令，且一次读取 9 字节
 *        2. 如果只读 3 字节，会被误认为其他芯片
 */
void BSP_FM25V20A_Read_ID(uint8_t *ID)
{
    uint8_t cmd = 0x9F;            // JEDEC ID 读取指令

    FM25Vx_Enable();                // 拉低 CS（片选有效）
    /* 发送指令 */
    HAL_SPI_Transmit(&hspi2, &cmd, 1, FM25Qx_TIMEOUT_VALUE);
    /* 接收 9 字节 ID */
    HAL_SPI_Receive(&hspi2, ID, 9, FM25Qx_TIMEOUT_VALUE);
    FM25Vx_Disable();               // 拉高 CS（片选失效）
}


/**
  * @brief  从 FM25V20A 连续读取数据
  * @param  address : 起始读取地址（18 位有效，范围 0x00000 ~ 0x3FFFF）
  * @param  buffer  : 数据接收缓冲区首地址
  * @param  length  : 要读取的字节数（任意长度）
  * @retval 无
  * @note   1. FM25V20A 支持 40 MHz SPI，无需等待 Busy；
  *         2. 地址 18 位，高 6 位可填 0，兼容更大容量；
  *         3. 函数内部完成 CS 拉低/拉高，调用前无需额外操作。
  */
void FM25V20A_Read(uint32_t address, uint8_t *buffer, uint32_t length)
{
    uint8_t cmd[4];

    // FM25V20A 地址为 18 位，需填充 3 字节地址（高位补0）
    cmd[0] = FM25V20A_READ_CMD;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FM25Vx_Enable();  // 拉低 CS 选中芯片

    // 发送命令 + 地址
    HAL_SPI_Transmit(&hspi2, cmd, 4, FM25Qx_TIMEOUT_VALUE);

    // 接收数据
    HAL_SPI_Receive(&hspi2, buffer, length, FM25Qx_TIMEOUT_VALUE);

    FM25Vx_Disable();  // 拉高 CS 结束通信
}


/**
 * @brief  向 FM25V20A 连续写入数据（无页限制）
 * @param  pData     : 待写入数据缓冲区首地址
 * @param  WriteAddr : 起始写入地址（0x00000 ~ 0x3FFFF）
 * @param  Size      : 要写入的字节数（任意长度）
 * @retval 
 *         FM25V20A_OK   0
           FM25V20A_ERR  1
 */
uint8_t BSP_FM25V20A_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
    uint8_t cmd[4];

    /* 构造写命令：0x02 + 3 字节地址（18 位地址，高 6 位忽略） */
    cmd[0] = 0x02;                          // WRITE command
    cmd[1] = (uint8_t)(WriteAddr >> 16) & 0x03; // A17~A16，仅低 2 位有效
    cmd[2] = (uint8_t)(WriteAddr >> 8);
    cmd[3] = (uint8_t)(WriteAddr);

    /* FM25V20A 写前必须发送 WREN（0x06） */
    BSP_FM25V20A_WriteEnable();  // 你自己封装的发送 0x06 的函数

    /* 开始 SPI 传输 */
    FM25Vx_Enable();  // 拉低 CS
    if (HAL_SPI_Transmit(&hspi2, cmd, 4, FM25Qx_TIMEOUT_VALUE) != HAL_OK)
    {
        FM25Vx_Disable();
        return FM25V20A_ERR;
    }

    if (HAL_SPI_Transmit(&hspi2, pData, Size, FM25Qx_TIMEOUT_VALUE) != HAL_OK)
    {
        FM25Vx_Disable();
        return FM25V20A_ERR;
    }
    FM25Vx_Disable();  // 拉高 CS，写入立即完成

    /* FM25V20A 没有 busy 位，无需等待 */
    return FM25V20A_OK;
}


uint8_t ID[9] , a;
static uint32_t i;

void Init_FM25Vx(void)
{
//	printf("error!\r\n");
  while(a != FM25V20A_OK)
  {
	  a = BSP_FM25V20A_Init();//初始化FM25V20A
	  HAL_Delay(5);
	  printf("%d\r\n",a);
  }

    BSP_FM25V20A_Read_ID(ID);//读取ID

	if (ID[2] == 0xC2 && ID[1] == 0x25 && ID[0] == 0x08)
	{
		printf("FM25V20A ID : \r");
		for(i=0;i<9;i++)
		{
		printf("0x%02X \r",ID[i]);//ID正确，输出ID
		}
		printf("\r\n\r\n");//间隔一行
	}
	else//ID错误
	{
		while(1)
		{
			printf("ID错误 \r\n",ID[i]);
		}

	}

}

void FM25Vx_text(void)
{
  	uint8_t  writeBuf[16] = {0};
	uint8_t  readBuf [16] = {0};
	uint32_t i;
    Init_FM25Vx();    //初始化flash：FM25V20A

    /* 2. 准备测试数据 */
	for (i = 0; i < sizeof(writeBuf); i++)
		writeBuf[i] = i + 0xA0;   // 填点非 0xFF 的值

	/* 3. 写入 16 字节到地址 0x12345 */
	if (BSP_FM25V20A_Write(writeBuf, 0x12345, sizeof(writeBuf)) != HAL_OK)
	{
		printf("Write Error!\r\n");
	}
	else
	{
		printf("Write Done\r\n");
	}

	/* 4. 读回验证 */
	FM25V20A_Read(0x12345, readBuf, sizeof(readBuf));

	/* 5. 打印比对 */
	printf("Read back:\r\n");
	for (i = 0; i < sizeof(readBuf); i++)
	{
		printf("0x%02X ", readBuf[i]);
	}
	printf("\r\n");

	/* 6. 简单校验 */
	if (memcmp(writeBuf, readBuf, sizeof(writeBuf)) == 0)
		printf("Verify OK\r\n");
	else
		printf("Verify FAIL\r\n");
}
