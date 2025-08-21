#ifndef __BMI088_H__
#define __BMI088_H__

#include "main.h"
#include <stdbool.h> // 包含 bool 类型定义
//加速度计id
#define ACC_CHIP_ID 0X1E

//快速启动bmi088
#define	ACC_PWR_CTRL  0X7D
#define	ACC_OFF  0X00
#define	ACC_ON  0X04

//加速度计模式配置，03睡眠模式，00正常模式
#define	ACC_PWR_CONF  0X7C
#define	ACC_ACTIVE  0X00
#define	ACC_SUSPEND  0X03

/* 加速度量程范围设定 */
#define	ACC_RANG  0X41
#define	Plus_Minus_3G  0X00
#define	Plus_Minus_6G  0X01
#define	Plus_Minus_12G  0X02
#define	Plus_Minus_24G  0X03

/* 加速度信号输出频率设定 */
#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

//陀螺仪id
#define GYRO_CHIP_ID 0X0F

//陀螺仪模式配置，表示测试范围
#define	GYRO_RANG  0X0F
#define	Plus_Minus_2000  0X00
#define	Plus_Minus_1000  0X01
#define	Plus_Minus_500   0X02
#define	Plus_Minus_250   0X03
#define	Plus_Minus_125   0X04


#define	GYRO_BANDWIDTH  0X10
/* 2000代表采样频率，532代表对应的滤波器带宽：
带宽越高，响应越快，但可能引入更多高频噪声；
带宽越低，噪声越少，但动态响应会变慢。 */
#define	ODR_2000_FD_532  0X00
#define	ODR_2000_FD_230  0X01
#define	ODR_1000_FD_116  0X02
#define	ODR_400_FD_47  0X03
#define	ODR_200_FD_23  0X04
#define	ODR_100_FD_12 0X05
#define	ODR_200_FD_64 0X06
#define	ODR_100_FD_32 0X07

/* 温度读取 */
#define BMI088_TEMP_M 0x22

#define BMI088_TEMP_L 0x23
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于BMI088传感器中的软件延时
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static void BMI088_Delay(int times);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 通过SPI通讯总线读取和发送传感器数据
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static uint8_t BMI088_SPIReadSend(SPI_HandleTypeDef *hspi, uint8_t data);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取加速度计的数据
 * @author:
 * @date: 2022/05/31
 * @note:	已经知道读取加速度计数据的函数，如果想使用数据，需要知道滤波函数的用法
 ****************************************************/
static uint8_t BMI088_Read_ACC(SPI_HandleTypeDef *hspi, uint8_t Addr, int BOARD_OR_FLOAT);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪的数据
 * @author:
 * @date: 2022/05/31
 * @note:	已经知道读取陀螺仪数据的函数，如果想使用数据，需要知道滤波函数的用法
 ****************************************************/
static uint8_t BMI088_Read_GYRO(SPI_HandleTypeDef *hspi, uint8_t Addr, int BOARD_OR_FLOAT);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static void BMI088_Write_Reg(SPI_HandleTypeDef *hspi, uint8_t Addr, uint8_t Val, uint8_t ACC_OR_GYRO);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的加速度计
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static uint8_t BMI088_ACC_Congfig(SPI_HandleTypeDef *hspi, int BOARD_OR_FLOAT);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static uint8_t BMI088_GYRO_Congfig(SPI_HandleTypeDef *hspi, int BOARD_OR_FLOAT);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪加速度传感器高八位数据
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static void BMI088_ACCEL_NS_H(void);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取陀螺仪加速度传感器低八位数据
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static void BMI088_ACCEL_NS_L(void);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于写传感器配置，如果需要改变传感器参数使用该方法
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static uint8_t BMI088_read_write_byte(SPI_HandleTypeDef *hspi, uint8_t txdata);

/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于读取BMI088传感器中多个寄存器的数据
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
static void BMI088_read_muli_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *buf, uint8_t len);


/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于配置初始化传感器中的陀螺仪和加速度计
 * @author:
 * @date: 2022/05/31
 * @note:
 ****************************************************/
uint8_t BMI088_FLOAT_ACC_GYRO_Init(SPI_HandleTypeDef *hspi);



//——————————————————————————————————————————————————————————————————————————————————————————————
/*!***************************************************
 * @file: BMI088.c
 * @brief: 用于处理换算传感器的温度数据
 * @author:
 * @date: 2021/10/18
 * @note:	读取出来温度进行温度补偿
 ****************************************************/
void BMI088_Read_TMP(float *temperate);
#endif

/*!***************************************************
 * @file: MYGYROData.c
 * @brief: 用于配置对陀螺仪加速度计进行数据单位换算
 * @note:
 ****************************************************/
#define BMI088 BMI088
#define BMI088_2 BMI088_2
#define xx 0
#define yy 1
#define zz 2

#define G_Z 0

/*陀螺仪加速度计去零漂方式选择*/

#define MPU_ACC_CALOFFSET_AUTO
#define  MPU_GROY_CALOFFSET_AUTO
/*---------------------------------------------*/

#define XAISN   (3)  //3个坐标
#define ITEMS  6
#define MPU9250_FILTER_NUM   4
#define MPU_CORRECTION_FLASH     0x0800F000        //存储校正数据的FLASH地址，SIZE=6*3*4字节
#define GG 9.8															// 重力加速?
/* 现在量程设定为±6g */
#define MPU_ACCE_M_S_2 (6.0 * GG / 32768.0) //单位换算，将LSB化为m/(s^2)，GG前面的"X"需根据量程修改
#define MPU_GYRO_TO_DPS (500 / 32768.0)    //单位换算，将LSB化为 gegree/s
#define MPU_MAGN_TO_GS (4800 / 16384.0 / 100.0)            //单位换算，将LSB化为Gs
#define MPU_MAGN_TO_UT (4800 / 16384.0)            //单位换算，将GS化为UT
#define MPU_TEMP_K (0.002995177763f)            //degC/LSB

typedef struct _accdata
{
    short origin[XAISN];  //原始值
    float offset[XAISN];      //零偏值
    float offset_max[XAISN];  //零偏值最大值
    float offset_min[XAISN];  //零偏值最小值
    float calibration[XAISN]; //校准值
    float filter[XAISN];      //滑动平均滤波值
    float m_s_2[XAISN];      //米每二次方秒
} accdata;

typedef struct _gyrodata
{
    short origin[XAISN];  //原始值
    float offset_max[XAISN];  //零偏值最大值
    float offset_min[XAISN];  //零偏值最小值
    float offset[XAISN];      //零偏值
    float calibration[XAISN]; //校准值
    float filter[XAISN];      //滑动平均滤波值
    float dps[XAISN];         //度每秒
    float radps[XAISN];       //弧度每秒
		/* 2022-01-11 加入滑动滤波 */
		float last_filter[XAISN];
} gyrodata;

struct _mpu
{
    accdata acc;
    gyrodata gyro;

    float Temperature;
    uint8_t acc_id, gyro_id;
    union
    {
        int16_t data[3];
        uint8_t buff[6];
    } ACC, GYRO;
    uint8_t temp_originalbuff[2];
    uint8_t gyro_times;
    uint8_t acc_times;
    enum
    {
        ReadingACC,
        ReadingGYRO,
        IDLE,
    } state;

    float pitch;
    float roll;
    float yaw;
    float lastyaw;
		/* 记录陀螺仪累计旋转多少度 */
		float yawsum;
		/* 陀螺仪动态数据矫正 */
		uint8_t DynamicOffsetEnable;	// 是否进行动态校准标志位
		int32_t DynamicTmp[3];				// 动态数据累加
		uint32_t DynamicTimes;				// 校准时间计数
		uint16_t OffsetCnt;						// 校准周期
		uint16_t OffsetErrorRange;		// 校准误差范围
		float gyro_z;
		int16_t yaw_turns;
};

typedef struct _mpu mpu;

extern mpu BMI088;
extern struct _mpu BMI088_2;
/**
  * @brief 	读取陀螺仪的数据
  * @retval  将读取出来的数据赋值给原始数
  *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
  * @attention 从这开始实际的频率编程 2000hz
  */
void BMI088_Read_Gyro_Data(bool a);

/**
  * @brief 	读取加速度计的数据
  * @retval  将读取出来的数据赋值给原始数
  *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
  * @attention 从这开始实际的频率编程 1600hz
  */
void BMI088_Read_Acc_Data(bool a);

/**
  * @brief 	读取温度
  * @retval  将读取出来的数据赋值给原始数
  *	@note	 data数据长度是3  原始数据长度是3  采集出来的数据就是原始数据
  * @attention 从这开始实际的频率编程 1600hz
  */
void BMI088_Read_Tmp_Data(bool a);

/*!***************************************************
 * @file: MYGYROData.c
 * @brief:读取加速度，并转换为 m/(s^2)
 * @note:
 ****************************************************/
void IMU_Read(bool a);
