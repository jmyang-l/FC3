
#ifndef BARO_BARO_H_
#define BARO_BARO_H_


#include "main.h"
 
#define SPL06_Write 0XED
#define SPL06_Read 0xEC
 
#define k_SPS1 524288.0
#define k_SPS2 1572864.0
#define k_SPS4 3670016.0
#define k_SPS8 7864320.0
#define k_SPS16 253952.0
#define k_SPS32 516096.0
#define k_SPS64 1040384.0
#define k_SPS128 2088960.0
 
#define PSR_B2_Addr 0x00
#define PSR_B1_Addr 0x01
#define PSR_B0_Addr 0x02
#define TMP_B2_Addr 0x03
#define TMP_B1_Addr 0x04
#define TMP_B0_Addr 0x05
#define PRS_CFG_Addr 0x06
#define TMP_CFG_Addr 0x07
#define MEAS_CFG_Addr 0x08
#define CFG_REG_Addr 0x09
#define INT_STS_Addr 0x0A//中断状态寄存器
#define RESET_Addr 0x0C
#define ID_Addr 0x0D
 
#define Temp_c0_Addr 0x10
#define Temp_c1_Addr  0x11
#define Temp_c2_Addr  0x12
 
#define Press_c0_Addr  0x13
#define Press_c1_Addr  0x14
#define Press_c2_Addr  0x15
#define Press_c3_Addr  0x16
#define Press_c4_Addr  0x17
#define Press_c5_Addr  0x18
#define Press_c6_Addr  0x19
#define Press_c7_Addr  0x1A
#define Press_c8_Addr  0x1B
#define Press_c9_Addr  0x1C
#define Press_c10_Addr  0x1D
#define Press_c11_Addr  0x1E
#define Press_c12_Addr  0x1F
#define Press_c13_Addr  0x20
#define Press_c14_Addr  0x21
 
#define Total_Number_24 16777216.0
#define Total_Number_20 1048576.0
#define Total_Number_16 65536.0
#define Total_Number_12 4096.0
 
uint8_t SPL06_Init(void);
// 初始化SPL06传感器
 
uint8_t SPL06_Read_Byte(uint8_t addr);
// 从SPL06传感器指定地址读取一个字节的数据
 
void SPL06_Write_Byte(uint8_t addr,uint8_t data);
// 向SPL06传感器指定地址写入一个字节的数据
 
void Parameter_Reading(int *Pressure_Para,int *Temperature_Para);
// 读取压力和温度相关的参数
 
float Temperature_conversion(uint32_t Temp_Data,float k);
// 将读取到的温度数据转换为实际温度值
 
float Pressure_conversion(uint32_t Pressure_Data,float k);
// 将读取到的压力数据转换为实际压力值
 
float Scale_factor(uint8_t Config_k);
// 根据配置计算比例因子，方便转换为实际数值
 
float Correcting_Pressure(int *Pressure_Para,float Pressure,float Temperature);
// 根据压力参数和温度修正压力值
 
float Correcting_Temperature(int *Temperature_Para,float Temperature);
// 根据温度参数修正温度值

#define Average_Times 4
#define Standard_atmospheric_pressure 101325.0
#define Offest_Pressure 0

// BARO传感器数据结构体
typedef struct {
    float Actual_temperature;     // 实际温度
    float Actual_pressure;        // 实际气压
    float Actual_altitude;        // 实际高度
    float temp_correction; // 温度校正值
    float press_correction; // 气压校正值
} BARO_Data;
extern	BARO_Data BARO_Data_Now;
// extern	float Actual_Temperature, Actual_Pressure, Actual_Altitude;
// extern	float Correcting_Temp, Correcting_Press;
void INIT_BARO(void);//气压计初始化函数

void  BARO_GPIO_EXTI_Callback(uint16_t GPIO_Pin);//BARO中断回调函数

#endif /* BARO_BARO_H_ */
