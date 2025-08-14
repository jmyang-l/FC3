/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置BMI088陀螺仪 加速度计的各项参数并读取数据
 * @author:
 * @date:2022/05/31
 * @note:
 ****************************************************/
#include "BMI088.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "baro.h"
#include "low_pass_filter.h"
#include <math.h>

#define SPI1_ACC_Enable                                                    \
    {                                                                      \
        HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);   \
        HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(CS2_ACC_GPIO_Port, CS2_ACC_Pin, GPIO_PIN_RESET); \
        HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_SET); \
    } // PA2,PC13

#define SPI1_ACC_Disable                                                   \
    {                                                                      \
        HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);     \
        HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(CS2_ACC_GPIO_Port, CS2_ACC_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_SET); \
    }

#define SPI1_GYRO_Enable                                                     \
    {                                                                        \
        HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);       \
        HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_RESET);   \
        HAL_GPIO_WritePin(CS2_ACC_GPIO_Port, CS2_ACC_Pin, GPIO_PIN_SET);     \
        HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_RESET); \
    } // PA3,PC2

#define SPI1_GYRO_Disable                                                  \
    {                                                                      \
        HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);     \
        HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(CS2_ACC_GPIO_Port, CS2_ACC_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_SET); \
    }

// #define SPI4_ACC_Enable       {  HAL_GPIO_WritePin(CS2_ACC_GPIO_Port,CS2_ACC_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port,CS2_GYRO_Pin, GPIO_PIN_SET);}//PC13
// #define SPI4_ACC_Disable       {  HAL_GPIO_WritePin(CS2_ACC_GPIO_Port,CS2_ACC_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port,CS2_GYRO_Pin, GPIO_PIN_SET);}
// #define SPI4_GYRO_Enable     {  HAL_GPIO_WritePin(CS2_ACC_GPIO_Port,CS2_ACC_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port,CS2_GYRO_Pin, GPIO_PIN_RESET);}//PC2
// #define SPI4_GYRO_Disable     {  HAL_GPIO_WritePin(CS2_ACC_GPIO_Port,CS2_ACC_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port,CS2_GYRO_Pin, GPIO_PIN_SET);}

#define CS_GYRO 0
#define CS_ACC 1

#define BOARD_IMU 0
#define FLOAT_IMU 1

float accel_offset[3] = {0.0f, 0.0f, 0.0f}; // 加速度计三轴零偏
float gyro_offset[3] = {0.0f, 0.0f, 0.0f};  // 陀螺仪三轴零偏

low_pass_filter_t accel_filter[3]; // 加速度计滤波
low_pass_filter_t gyro_filter[3];  // 陀螺仪滤波

/* 用于读取BMI088温度数据 */
// spi通信在bmi里面，bit（0）位为1表示读，0表示写，这里因为是小端是或0x80
static void BMI088_accel_read_muli_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(hspi, (reg) | 0x80);
    BMI088_read_muli_reg(hspi, reg, buf, len);
    BMI088_ACCEL_NS_H();
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于BMI088传感器中的软件延时
 * @author:
 * @date:2022/05/31
 * @note:
 ****************************************************/
static void BMI088_Delay(int times)
{
    HAL_Delay(times);
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 通过SPI通讯总线读取和发送传感器数据
 * @author:
 * @date:2022/05/31
 * @note:
 ****************************************************/
static uint8_t BMI088_SPIReadSend(SPI_HandleTypeDef *hspi, uint8_t data)
{
    uint8_t ret = 0xff;
    HAL_SPI_TransmitReceive(hspi, &data, &ret, 1, 0xffff);
    return ret;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪的数据
 * @author:
 * @date:2022/05/31
 * @note:	已经知道读取陀螺仪数据的函数，如果想使用数据，需要知道滤波函数的用法
 ****************************************************/
static uint8_t BMI088_Read_GYRO(SPI_HandleTypeDef *hspi, uint8_t Addr, int BOARD_OR_FLOAT)
{
    uint8_t val;
    SPI1_GYRO_Enable

        // 步骤1：发送读命令（Addr | 0x80）
        // 此时传感器正在接收地址，返回值无效，所以丢弃
        BMI088_SPIReadSend(hspi, Addr | 0x80); // spi通信在bmi里面，bit（0）位为1表示读，0表示写，这里因为是小端是或0x80

    // 步骤2：发送虚拟字节（0x00），触发传感器返回数据
    // 此时传感器返回的才是目标寄存器的值
    val = (uint8_t)(BMI088_SPIReadSend(hspi, 0x00) & 0xFF); // 提取数据的低 8 位。
    SPI1_GYRO_Disable;
    return val;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取加速度计的数据
 * @author:
 * @date:2022/05/31
 * @note:	已经知道读取加速度计数据的函数，如果想使用数据，需要知道滤波函数的用法
 ****************************************************/
static uint8_t BMI088_Read_ACC(SPI_HandleTypeDef *hspi, uint8_t Addr, int BOARD_OR_FLOAT)
{
    uint8_t val;
    SPI1_ACC_Enable
        //	HAL_GPIO_WritePin(CS2_ACC_GPIO_Port,CS2_ACC_Pin, GPIO_PIN_RESET);
        //    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port,CS2_GYRO_Pin, GPIO_PIN_SET);
        BMI088_SPIReadSend(hspi, Addr | 0x80);

    // 在读取加速度计数据时，传感器不会直接返回请求的寄存器数据
    // 而是先发送一个不可预测的虚拟字节（dummy byte），之后才发送实际的寄存器内容。
    BMI088_SPIReadSend(hspi, 0x00);

    val = (uint8_t)(BMI088_SPIReadSend(hspi, 0x00) & 0xFF);

    SPI1_ACC_Disable
        //    HAL_GPIO_WritePin(CS2_ACC_GPIO_Port,CS2_ACC_Pin, GPIO_PIN_SET);
        return val;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
 * @author:
 * @date:2022/05/31
 * @note:
 ****************************************************/
static void BMI088_Write_Reg(SPI_HandleTypeDef *hspi, uint8_t Addr, uint8_t Val, uint8_t ACC_OR_GYRO)
{
    if (ACC_OR_GYRO == CS_ACC)
    {
        SPI1_ACC_Enable
    }
    else if (ACC_OR_GYRO == CS_GYRO)
    {
        SPI1_GYRO_Enable
    }
    BMI088_SPIReadSend(hspi, Addr & 0x7f); // 0x7F的二进制为0111 1111，置零写操作
    BMI088_SPIReadSend(hspi, Val);         // 先发地址再发数据
    if (ACC_OR_GYRO == CS_ACC)
    {
        SPI1_ACC_Disable
    }
    else if (ACC_OR_GYRO == CS_GYRO)
    {
        SPI1_GYRO_Disable
    }
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的加速度计
 * @author:
 * @date:2022/05/31
 * @note:	配置加速度计产生中断的IO口的输出方式，数据输出频率1600hz
                加速度量程是 ±24g 有点大，目前的车无需这么大的量程
 ****************************************************/
/**
 * @brief
 * @retval  尝试降低采样率是否能提高数据稳定性
 * @date: 2021/10/17
 */
int ret = 0;
static uint8_t BMI088_ACC_Congfig(SPI_HandleTypeDef *hspi, int BOARD_OR_FLOAT)
{
    SPI1_ACC_Disable; // 全cs失能，在下面配置的函数会cs使能

    BMI088_Write_Reg(hspi, 0x7e, 0xb6, CS_ACC); // Soft Reset

    if (hspi == &hspi1)
    {
        while (BMI088.acc_id != ACC_CHIP_ID) // Rising edge ,turn to spi
        {

            BMI088_Delay(100);
            BMI088.acc_id = BMI088_Read_ACC(hspi, 0x00, BOARD_OR_FLOAT); // id:1E
        }
    }

    if (hspi == &hspi4)
    {
        while (BMI088_2.acc_id != ACC_CHIP_ID) // Rising edge ,turn to spi
        {
            BMI088_Delay(100);
            BMI088_2.acc_id = BMI088_Read_ACC(hspi, 0x00, BOARD_OR_FLOAT); // id:1E

            //        BMI088_Delay(100);
            //        BMI088_2.acc_id = 0;
            //        BMI088_2.acc_id = BMI088_Read_GYRO(&hspi1, 0x00, BOARD_OR_FLOAT); //id:1E
            //
            //        HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088_2.acc_id, sizeof(BMI088_2.acc_id), 100); //发送数据给串口
            //
            //        BMI088_2.acc_id = BMI088_Read_ACC(&hspi1, 0x00, BOARD_OR_FLOAT);
            //
            //        HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088_2.acc_id, sizeof(BMI088_2.acc_id), 100); //发送数据给串口
        }
    }

    BMI088_Delay(50); //> 1 ms ;
    ret = 0;
    while (ret != ACC_ON) // 等待启动完成
    {
        ret = BMI088_Read_ACC(hspi, ACC_PWR_CTRL, BOARD_OR_FLOAT);
        BMI088_Write_Reg(hspi, ACC_PWR_CTRL, ACC_ON, CS_ACC);
        BMI088_Delay(5);
    }
    /* 可根据实际使用和加速度寄存器表修改加速度计测量范围 */
    while (BMI088_Read_ACC(hspi, ACC_RANG, BOARD_OR_FLOAT) != Plus_Minus_6G) // 等待配置完成
    {
        BMI088_Write_Reg(hspi, ACC_RANG, Plus_Minus_6G, CS_ACC); // ACC Rang +- 24g;//
        BMI088_Delay(5);
    }
    while (BMI088_Read_ACC(hspi, 0x40, BOARD_OR_FLOAT) != 0xBA)
    {
        /* 可根据实际使用和加速度寄存器表修改加速度计数据读取频率 */
        BMI088_Write_Reg(hspi, 0x40, 0xBA, CS_ACC); //
        BMI088_Delay(5);
    }
    while (BMI088_Read_ACC(hspi, 0X53, BOARD_OR_FLOAT) != 0X08)
    {
        BMI088_Write_Reg(hspi, 0X53, 0X08, CS_ACC); // 0000 1000 INT1 OUTPUT PUSH-PULL  Active low
        BMI088_Delay(5);
    }
    while (BMI088_Read_ACC(hspi, 0X54, BOARD_OR_FLOAT) != 0X08)
    {
        BMI088_Write_Reg(hspi, 0X54, 0X08, CS_ACC); // 0000 1000 INT2 OUTPUT PUSH-PULL  Active low
        BMI088_Delay(5);
    }
    while (BMI088_Read_ACC(hspi, 0X58, BOARD_OR_FLOAT) != 0x44)
    {
        BMI088_Write_Reg(hspi, 0X58, 0x44, CS_ACC); // 0100 0100 data ready interrupt to INT1 and INT2，使能int1/2
        BMI088_Delay(5);
    }

    BMI088_Delay(20); //>1ms
    while (BMI088_Read_ACC(hspi, ACC_PWR_CONF, BOARD_OR_FLOAT) != ACC_ACTIVE)
    {
        BMI088_Write_Reg(hspi, ACC_PWR_CONF, ACC_ACTIVE, CS_ACC); // ACtive mode
        BMI088_Delay(600);                                        //>50ms
    }
    return 0;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪
 * @author:
 * @date:2022/05/31
 * @note:	配置陀螺仪产生中断的IO口的输出方式，数据输出频率2000hz
 ****************************************************/
static uint8_t BMI088_GYRO_Congfig(SPI_HandleTypeDef *hspi, int BOARD_OR_FLOAT)
{
    SPI1_ACC_Disable; // 全cs失能，在下面配置的函数会cs使能

    BMI088_Write_Reg(hspi, 0x14, 0xb6, CS_GYRO); // Soft Reset

    if (hspi == &hspi1)
    {
        while (BMI088.gyro_id != GYRO_CHIP_ID)
        {
            BMI088.gyro_id = BMI088_Read_GYRO(hspi, 0x00, BOARD_OR_FLOAT); // 0x0f
            BMI088_Delay(5);
        }
    }
    if (hspi == &hspi4)
    {
        while (BMI088_2.gyro_id != GYRO_CHIP_ID)
        {

            //        uint8_t TEXTACC[1] = {0x05};
            //            HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

            BMI088_2.gyro_id = BMI088_Read_GYRO(hspi, 0x00, BOARD_OR_FLOAT); // 0x0f
            BMI088_Delay(5);
        }
    }

    while (BMI088_Read_GYRO(hspi, GYRO_RANG, BOARD_OR_FLOAT) != Plus_Minus_1000)
    {
        //        uint8_t TEXTACC[1] = {0x06};
        //            HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

        BMI088_Write_Reg(hspi, GYRO_RANG, Plus_Minus_1000, CS_GYRO); // rang +-2000
        BMI088_Delay(5);
    }
    // bit #7 is Read Only

    /* 可根据陀螺仪寄存器对照表陀螺仪采样率是2000hz以及相对应的滤波器宽度 */
    BMI088_Write_Reg(hspi, GYRO_BANDWIDTH, ODR_400_FD_47, CS_GYRO);

    while (BMI088_Read_GYRO(hspi, 0X11, BOARD_OR_FLOAT) != 0x00)
    {
        //        uint8_t TEXTACC[1] = {0x07};
        //            HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

        BMI088_Write_Reg(hspi, 0X11, 0x00, CS_GYRO); // normal模式
        BMI088_Delay(5);
    }
    while (BMI088_Read_GYRO(hspi, 0X15, BOARD_OR_FLOAT) != 0X80)
    {
        //        uint8_t TEXTACC[1] = {0x08};
        //            HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

        BMI088_Write_Reg(hspi, 0X15, 0X80, CS_GYRO); // //interrupt使能中断，在bmi088里面加速度的中断自动使能，但是**陀螺仪要手动使能**
        BMI088_Delay(5);
    }
    while (BMI088_Read_GYRO(hspi, 0X16, BOARD_OR_FLOAT) != 0X00) // int3/4配置，只能输出模式
    {
        //        uint8_t TEXTACC[1] = {0x09};
        //            HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

        BMI088_Write_Reg(hspi, 0X16, 0X00, CS_GYRO); // OUTPUT PUSH-PULL  Active low
        BMI088_Delay(5);
    }
    while (BMI088_Read_GYRO(hspi, 0X18, BOARD_OR_FLOAT) != 0X81)
    {
        //        uint8_t TEXTACC[1] = {0x10};
        //            HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

        BMI088_Write_Reg(hspi, 0X18, 0X81, CS_GYRO); // data ready interrupt to INT3 and INT4，使能int3/4
        BMI088_Delay(5);
    }
    return 0;
}

/*****************************以下几个函数用于读取BMI088传感器的温度数据****************************/
/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪加速度传感器高八位数据
 * @author:
 * @date:2022/05/31
 * @note:	参考的大疆源码BMI088传感器的温度补偿控制
            温度传感器应该是挂载在加速度传感器上，所以读取温度
            需要使用加速度传感器
 ****************************************************/
static void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET); // 没写完的，先这样吧
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪加速度传感器低八位数据
 * @author:
 * @date:2022/05/31
 * @note:	参考的大疆源码BMI088传感器的温度补偿控制
            温度传感器应该是挂载在加速度传感器上，所以读取温度
            需要使用加速度传感器
 ****************************************************/
static void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET); // 没写完的，先这样吧
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
 * @author:
 * @date:2022/05/31
 * @note:	该函数与 BMI088_Write_Reg 函数不同，使用的时候需要注意
 ****************************************************/
static uint8_t BMI088_read_write_byte(SPI_HandleTypeDef *hspi, uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(hspi, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取BMI088传感器中多个寄存器的数据
 * @author:
 * @date:2022/05/31
 * @note:	参考的大疆源码BMI088传感器的温度补偿控制
 ****************************************************/
static void BMI088_read_muli_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(hspi, reg | 0x80);
    while (len != 0)
    {
        *buf = BMI088_read_write_byte(hspi, 0x55); // 虚拟字节
        buf++;
        len--;
    }
}
/***************************************************************************************************/

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪和加速度计
 * @author:
 * @date:2022/05/31
 * @note:
 ****************************************************/
uint8_t BMI088_FLOAT_ACC_GYRO_Init(SPI_HandleTypeDef *hspi)
{

    BMI088_ACC_Congfig(hspi, FLOAT_IMU);
    BMI088_GYRO_Congfig(hspi, FLOAT_IMU);

    for (int i = 0; i < 3; i++) // 初始化滤波器
    {
        lpf_set_cutoff_frequency(&accel_filter[i], 800, 20);
        lpf_set_cutoff_frequency(&gyro_filter[i], 800, 20);
    }

    return 1;
}

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于处理传感器的温度
 * @author:
 * @date: 2021/10/18
 * @note:	读取出来的温度进行温度补偿
 ****************************************************/
void BMI088_Read_TMP(float *temperate)
{
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((BMI088.temp_originalbuff[0] << 3) | (BMI088.temp_originalbuff[1] >> 5));
    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* 按键外部中断定义变量 */
extern uint8_t exit_flag;
extern uint8_t rising_falling_flag;

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置中断服务函数
 * @author:
 * @date:2022/05/31
 * @note:	I/O外部中断回调函数，用于接收陀螺仪和加速度数据
 ****************************************************/
/* 定义变量用于读取传输频率（中断触发） */
int16_t Gyro_Cnt = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //    if( mtime.Init_OK!=1)
    //        return;

    switch (GPIO_Pin)
    {
    case INT_ACC_Pin: // ACC中断

        //     //测试bmi088ACC的中断输出
        //     uint8_t TEXTACC[1] = {0x01};
        //         HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

        BMI088.ACC.buff[0] = BMI088_Read_ACC(&hspi1, 0X12, FLOAT_IMU);
        BMI088.ACC.buff[1] = BMI088_Read_ACC(&hspi1, 0X13, FLOAT_IMU);
        BMI088.ACC.buff[2] = BMI088_Read_ACC(&hspi1, 0X14, FLOAT_IMU);
        BMI088.ACC.buff[3] = BMI088_Read_ACC(&hspi1, 0X15, FLOAT_IMU);
        BMI088.ACC.buff[4] = BMI088_Read_ACC(&hspi1, 0X16, FLOAT_IMU);
        BMI088.ACC.buff[5] = BMI088_Read_ACC(&hspi1, 0X17, FLOAT_IMU); // 12--17为加速度计xyz信息

        /* 用于读取传感器的温度 */
        BMI088_accel_read_muli_reg(&hspi1, BMI088_TEMP_M, BMI088.temp_originalbuff, 2); // 读取raw温度数据
        break;
    case INT_GYRO_Pin: // GYRO中断

        //     //测试bmi088GYRO的中断输出
        //     uint8_t TEXTGYRO[1] = {0x02};
        //         HAL_UART_Transmit(&huart4, TEXTGYRO, 1, 100); //发送数据给串口

        BMI088.GYRO.buff[0] = BMI088_Read_GYRO(&hspi1, 0X02, FLOAT_IMU);
        BMI088.GYRO.buff[1] = BMI088_Read_GYRO(&hspi1, 0X03, FLOAT_IMU);
        BMI088.GYRO.buff[2] = BMI088_Read_GYRO(&hspi1, 0X04, FLOAT_IMU);
        BMI088.GYRO.buff[3] = BMI088_Read_GYRO(&hspi1, 0X05, FLOAT_IMU);
        BMI088.GYRO.buff[4] = BMI088_Read_GYRO(&hspi1, 0X06, FLOAT_IMU);
        BMI088.GYRO.buff[5] = BMI088_Read_GYRO(&hspi1, 0X07, FLOAT_IMU); // 2--7为陀螺仪xyz信息
        /* 陀螺仪的引脚触发中断一次，计数+1，用于确定陀螺仪传感器实际的传输频率*/
        break;

    case INT_ACC2_Pin: // ACC2中断

        //    //测试bmi088ACC的中断输出
        //    uint8_t TEXTACC2[1] = {0x03};
        //        HAL_UART_Transmit(&huart4, TEXTACC2, 1, 100); //发送数据给串口

        BMI088_2.ACC.buff[0] = BMI088_Read_ACC(&hspi4, 0X12, FLOAT_IMU);
        BMI088_2.ACC.buff[1] = BMI088_Read_ACC(&hspi4, 0X13, FLOAT_IMU);
        BMI088_2.ACC.buff[2] = BMI088_Read_ACC(&hspi4, 0X14, FLOAT_IMU);
        BMI088_2.ACC.buff[3] = BMI088_Read_ACC(&hspi4, 0X15, FLOAT_IMU);
        BMI088_2.ACC.buff[4] = BMI088_Read_ACC(&hspi4, 0X16, FLOAT_IMU);
        BMI088_2.ACC.buff[5] = BMI088_Read_ACC(&hspi4, 0X17, FLOAT_IMU); // 12--17为加速度计xyz信息

        /* 用于读取传感器的温度 */
        BMI088_accel_read_muli_reg(&hspi4, BMI088_TEMP_M, BMI088.temp_originalbuff, 2); // 读取raw温度数据
        break;
    case INT_GYRO2_Pin: // GYRO2中断

        //    //测试bmi088GYRO的中断输出
        //    uint8_t TEXTGYRO2[1] = {0x04};
        //        HAL_UART_Transmit(&huart4, TEXTGYRO2, 1, 100); //发送数据给串口

        BMI088_2.GYRO.buff[0] = BMI088_Read_GYRO(&hspi4, 0X02, FLOAT_IMU);
        BMI088_2.GYRO.buff[1] = BMI088_Read_GYRO(&hspi4, 0X03, FLOAT_IMU);
        BMI088_2.GYRO.buff[2] = BMI088_Read_GYRO(&hspi4, 0X04, FLOAT_IMU);
        BMI088_2.GYRO.buff[3] = BMI088_Read_GYRO(&hspi4, 0X05, FLOAT_IMU);
        BMI088_2.GYRO.buff[4] = BMI088_Read_GYRO(&hspi4, 0X06, FLOAT_IMU);
        BMI088_2.GYRO.buff[5] = BMI088_Read_GYRO(&hspi4, 0X07, FLOAT_IMU); // 2--7为陀螺仪xyz信息
        /* 陀螺仪的引脚触发中断一次，计数+1，用于确定陀螺仪传感器实际的传输频率*/
        break;

    default:
        break;
    }

    BARO_GPIO_EXTI_Callback(GPIO_Pin);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————

/*!***************************************************
 * @brief: 用于配置对陀螺仪加速度计进行单位换算
 * @note:
 ****************************************************/

// 陀螺仪零偏矫正
mpu BMI088;
struct _mpu BMI088_2;

/**
 * @brief 	读取陀螺仪的数据
 * @retval  将读取出来的数据赋值给原始数
 *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
 * @attention 从这开始实际的频率编程 2000hz
 */
void BMI088_Read_Gyro_Data(bool a)
{
    if (!a)
    {
        BMI088.gyro.origin[xx] = BMI088.GYRO.data[xx]; // union联合体地址共享
        BMI088.gyro.origin[yy] = BMI088.GYRO.data[yy];
        BMI088.gyro.origin[zz] = -BMI088.GYRO.data[zz];
    }
    else
    {
        BMI088_2.gyro.origin[xx] = BMI088_2.GYRO.data[xx]; // union联合体地址共享
        BMI088_2.gyro.origin[yy] = BMI088_2.GYRO.data[yy];
        BMI088_2.gyro.origin[zz] = -BMI088_2.GYRO.data[zz];
    }
}

/**
 * @brief 	读取加速度计的数据
 * @retval  将读取出来的数据赋值给原始数
 *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
 * @attention 从这开始实际的频率编程 1600hz
 */
void BMI088_Read_Acc_Data(bool a)
{
    if (!a)
    {
        BMI088.acc.origin[xx] = BMI088.ACC.data[yy];
        BMI088.acc.origin[yy] = -BMI088.ACC.data[xx];
        BMI088.acc.origin[zz] = BMI088.ACC.data[zz];
    }
    else
    {
        BMI088_2.acc.origin[xx] = BMI088_2.ACC.data[yy];
        BMI088_2.acc.origin[yy] = -BMI088_2.ACC.data[xx];
        BMI088_2.acc.origin[zz] = BMI088_2.ACC.data[zz];
    }
}

/**
 * @brief 	温度数据处理函数
 * @retval  将读取出来的数据赋值给原始数
 *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
 * @attention 从这开始实际的频率编程 1600hz
 */
void BMI088_Read_Tmp_Data(bool a)
{
    if (!a)
    {
        BMI088_Read_TMP(&BMI088.Temperature);
    }
    else
    {
        BMI088_Read_TMP(&BMI088_2.Temperature);
    }
}

/*!***************************************************
 * @file: MYGYROData.c
 * @brief:读取加速度/陀螺仪数据并进行单位转换，并转换为 m/(s^2)和°/s
 * @note:这个函数是对外提供的接口函数，用户调用这个函数可以获取到加速度/陀螺仪数据
 ****************************************************/
void IMU_Read(bool a)
{
    if (!a)
    {
        mpu *mympu = &BMI088; // 使用指针去操作BMI088的结构体

        BMI088_Read_Gyro_Data(a); /* 读取陀螺仪数据 */
        BMI088_Read_Acc_Data(a);  /* 读取加速度数据 */

        for (int i = 0; i < 3; i++)
        {
            mympu->gyro.dps[i] = mympu->gyro.origin[i] * MPU_GYRO_TO_DPS*0.250f*0.175f;
            mympu->gyro.dps[i] = lpf_allpy(&gyro_filter[i], mympu->gyro.dps[i]);

            mympu->acc.m_s_2[i] = mympu->acc.origin[i] * MPU_ACCE_M_S_2;
            mympu->acc.m_s_2[i] = lpf_allpy(&accel_filter[i], mympu->acc.m_s_2[i]);
        }
    }
    else
    {
        mpu *mympu = &BMI088_2; // 使用指针去操作BMI088_2的结构体

        BMI088_Read_Gyro_Data(a); /* 读取陀螺仪数据 */
        BMI088_Read_Acc_Data(a);  /* 读取加速度数据 */

        for (int i = 0; i < 3; i++)
        {
            mympu->gyro.dps[i] = mympu->gyro.origin[i] * MPU_GYRO_TO_DPS*0.125f - gyro_offset[i];
//            mympu->gyro.dps[i] = lpf_allpy(&gyro_filter[i], mympu->gyro.dps[i]);

            mympu->acc.m_s_2[i] = mympu->acc.origin[i] * MPU_ACCE_M_S_2 - accel_offset[i];
//            mympu->acc.m_s_2[i] = lpf_allpy(&accel_filter[i], mympu->acc.origin[i]);
        }
    }
}
