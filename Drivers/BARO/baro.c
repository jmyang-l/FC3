#include "baro.h"
#include "i2c.h"
#include "usart.h"
// void SPL06_Write_Byte(uint8_t addr,uint8_t data)  //软件I2C写数据
// {
//   IIC_Start();
// 	IIC_Send_Byte(SPL06_Write);
// 	IIC_Wait_Ack();
// 	IIC_Send_Byte(addr);
// 	IIC_Wait_Ack();
// 	IIC_Send_Byte(data);
// 	IIC_Wait_Ack();
// 	IIC_Stop();
// }

// uint8_t SPL06_Read_Byte(uint8_t addr)
// {
// 	uint8_t SPL06_Data;
	
// 	IIC_Start();
// 	IIC_Send_Byte(SPL06_Write);
// 	IIC_Wait_Ack();
// 	IIC_Send_Byte(addr);
// 	IIC_Wait_Ack();
 
// 	IIC_Start();// start again
// 	IIC_Send_Byte(SPL06_Read);
// 	IIC_Wait_Ack();
// 	SPL06_Data = IIC_Read_Byte(0);
// 	IIC_Stop();
// 	return SPL06_Data;
// }
void delay_ms(int times)
{
HAL_Delay(times);
}
// 向 SPL06 传感器指定地址写入一个字节数据，使用硬件 I2C
void SPL06_Write_Byte(uint8_t addr, uint8_t data)
{
  HAL_StatusTypeDef status;
  
  // 使用 HAL_I2C_Mem_Write 函数，参数分别是：
  //I2C 句柄、设备写地址、寄存器地址、寄存器地址长度、要写入的数据、数据长度、超时时间
  status = HAL_I2C_Mem_Write(&hi2c1, SPL06_Write, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
//   if (status != HAL_OK)
//   {
//     // 这里可以添加错误处理，比如打印错误信息、进入错误处理函数等
//     Error_Handler();
//   }
}
 


/**
 * 从SPL06-001指定寄存器地址读取1字节数据（硬件I2C实现）
 * @param addr：寄存器地址（如0x00-0x27，见规格书第7章寄存器映射）
 * @return 读取到的寄存器数据
 */
uint8_t SPL06_Read_Byte(uint8_t addr)
{
    uint8_t SPL06_Data;
    HAL_StatusTypeDef status;

    // 调用HAL库函数读取数据，符合规格书I2C读时序：
    // 自动完成"写寄存器地址→重复起始→读数据"流程
    status = HAL_I2C_Mem_Read(
        &hi2c1,              // I2C句柄（需匹配硬件I2C外设）
        SPL06_Read,          // 设备读地址（符合规格书I2C地址定义）
        addr,                // 目标寄存器地址（如压力数据寄存器0x00-0x02）
        I2C_MEMADD_SIZE_8BIT,// 寄存器地址为8位（规格书寄存器地址为8位）
        &SPL06_Data,         // 接收数据缓冲区
        1,                   // 读取1字节（单寄存器数据）
        100                  // 超时时间（避免死等，单位ms）
    );

    // // 检查通信状态（规格书要求需处理通信异常）
    // if (status != HAL_OK)
    // {
    //     // 通信失败处理（如返回错误值或进入错误函数）
    //     // 可参考规格书第5.3.1节I2C错误处理建议
    //     return 0xFF;  // 错误时返回无效值（根据实际需求调整）
    // }
    return SPL06_Data;
}


uint8_t SPL06_Init(void)//初始化SPL06-001
{
	uint8_t SPL06_ID;
	SPL06_Write_Byte(RESET_Addr,0x89);//Reset
	delay_ms(100);
	SPL06_ID = SPL06_Read_Byte(ID_Addr);//Read the SPL06's ID
	SPL06_Write_Byte(MEAS_CFG_Addr,0x07);//Set Working mode and state of sensor，连续测量压力+温度mode
	SPL06_Write_Byte(PRS_CFG_Addr,0x26);//Set the PM-RATE and PM-PRC，压力测量速率和精度，可查表对应
	SPL06_Write_Byte(TMP_CFG_Addr,0xA0);//Set the TMPI-RATE and TMP-PRC，温度测量速率和精度，可查表对应

    //开压力、温度就绪中断（SDO引脚），无FIFO
	//SDO接低时要高电平触发，接高时要低电平触发
	SPL06_Write_Byte(CFG_REG_Addr,0xB4);//移位（精度>8时要移位）和中断（压力or温度、单次or批次）配置
	return SPL06_ID;
}

//温度数据处理函数
float Temperature_conversion(uint32_t Temp_Data,float k)
{
	float Temperature;
	int Temp;
	// if(Temp_Data&0x800000)
	// {
	// 	Temp = Temp_Data-Total_Number_24;
	// }
	// else
	// {
	 	 Temp = Temp_Data;
	// }
	Temperature = Temp/k;
	return Temperature;
}

//压力数据处理函数
float Pressure_conversion(uint32_t Pressure_Data,float k)
{
	float Pressure;
	int Press;
	// if(Pressure_Data&0x800000)
	// {
	// 	Press = Pressure_Data-Total_Number_24;
	// }
	// else
	// {
	 	Press = Pressure_Data;
	// }
	Pressure = Press/k;
	return Pressure;
}

//匹配过采样率配置与预设系数，确保上两个转换函数能将原始数据准确换算为实际物理量（温度值和压力值）
float Scale_factor(uint8_t Config_k)
{
	float k;
	switch(Config_k)
	{
		case 0: k = k_SPS1;break;
		case 1: k = k_SPS2;break;
    case 2: k = k_SPS4;break;
		case 3: k = k_SPS8;break;
		case 4: k = k_SPS16;break;
		case 5: k = k_SPS32;break;
    case 6: k = k_SPS64;break;
    case 7:	k = k_SPS128;break;	 
	default: k = 0;break;	
	}
	return k;
}

//读取校准参数
void Parameter_Reading(int *Pressure_Para,int *Temperature_Para)
{
	uint8_t Temp_Config0,Temp_Config1,Temp_Config2;
	uint8_t Press_Config0,Press_Config1,Press_Config2,Press_Config3,Press_Config4;
	uint8_t Press_Config5,Press_Config6,Press_Config7,Press_Config8,Press_Config9;
	uint8_t Press_Config10,Press_Config11,Press_Config12,Press_Config13,Press_Config14;	 
	//Temperature
	Temp_Config0 = SPL06_Read_Byte(Temp_c0_Addr);
	Temp_Config1 = SPL06_Read_Byte(Temp_c1_Addr);
	Temp_Config2 = SPL06_Read_Byte(Temp_c2_Addr);
	Temperature_Para[0] = ((int32_t)Temp_Config0<<4)+(((int32_t)Temp_Config1&0xF0)>>4);
	if(Temperature_Para[0]&0x0800) Temperature_Para[0] = Temperature_Para[0]-Total_Number_12;
	Temperature_Para[1] = (((int32_t)Temp_Config1&0x0F)<<8)+(int32_t)Temp_Config2;
	if(Temperature_Para[1]&0x0800) Temperature_Para[1] = Temperature_Para[1]-Total_Number_12;
	//Pressure
	Press_Config0 = SPL06_Read_Byte(Press_c0_Addr);
	Press_Config1 = SPL06_Read_Byte(Press_c1_Addr);
	Press_Config2 = SPL06_Read_Byte(Press_c2_Addr);
	Press_Config3 = SPL06_Read_Byte(Press_c3_Addr);
	Press_Config4 = SPL06_Read_Byte(Press_c4_Addr);
	Press_Config5 = SPL06_Read_Byte(Press_c5_Addr);
	Press_Config6 = SPL06_Read_Byte(Press_c6_Addr);
	Press_Config7 = SPL06_Read_Byte(Press_c7_Addr);
	Press_Config8 = SPL06_Read_Byte(Press_c8_Addr);
	Press_Config9 = SPL06_Read_Byte(Press_c9_Addr);
	Press_Config10 = SPL06_Read_Byte(Press_c10_Addr);
	Press_Config11 = SPL06_Read_Byte(Press_c11_Addr);
	Press_Config12 = SPL06_Read_Byte(Press_c12_Addr);
	Press_Config13 = SPL06_Read_Byte(Press_c13_Addr);
	Press_Config14 = SPL06_Read_Byte(Press_c14_Addr);

	
// 修复后代码（避免溢出）
Pressure_Para[0] = ((int32_t)Press_Config0 << 12) + 
                  ((int32_t)Press_Config1 << 4) + 
                  (((int32_t)Press_Config2 & 0xF0) >> 4); // c00（20位）
if (Pressure_Para[0] & 0x80000)  // 判断20位符号位（第19位）
    Pressure_Para[0] -= Total_Number_20;  // 转为补码负数

Pressure_Para[1] = (((int32_t)Press_Config2 & 0x0F) << 16) + 
                  ((int32_t)Press_Config3 << 8) + 
                  (int32_t)Press_Config4;  // c10（20位）
if (Pressure_Para[1] & 0x80000)  // 判断20位符号位
    Pressure_Para[1] -= Total_Number_20;

Pressure_Para[2] = ((int32_t)Press_Config5 << 8) + (int32_t)Press_Config6;  // c01（16位）
if (Pressure_Para[2] & 0x8000)  // 判断16位符号位（第15位）
    Pressure_Para[2] -= Total_Number_16;

Pressure_Para[3] = ((int32_t)Press_Config7 << 8) + (int32_t)Press_Config8;  // c11（16位）
if (Pressure_Para[3] & 0x8000)
    Pressure_Para[3] -= Total_Number_16;

Pressure_Para[4] = ((int32_t)Press_Config9 << 8) + (int32_t)Press_Config10;  // c20（16位）
if (Pressure_Para[4] & 0x8000)
    Pressure_Para[4] -= Total_Number_16;

Pressure_Para[5] = ((int32_t)Press_Config11 << 8) + (int32_t)Press_Config12;  // c21（16位）
if (Pressure_Para[5] & 0x8000)
    Pressure_Para[5] -= Total_Number_16;

Pressure_Para[6] = ((int32_t)Press_Config13 << 8) + (int32_t)Press_Config14;  // c30（16位）
if (Pressure_Para[6] & 0x8000)
    Pressure_Para[6] -= Total_Number_16;


	// Pressure_Para[0] = (Press_Config0<<12)+(Press_Config1<<4)+((Press_Config2&0xF0)>>4);//c00
	// if(Pressure_Para[0]&0x80000) Pressure_Para[0] = Pressure_Para[0] - Total_Number_20;//c00
	// Pressure_Para[1] = ((Press_Config2&0x0F)<<16)+ (Press_Config3<<8)+ Press_Config4;//c10
	// if(Pressure_Para[1]&0x80000) Pressure_Para[1] = Pressure_Para[1] - Total_Number_20;//c10
	// Pressure_Para[2] = (Press_Config5<<8)+Press_Config6;//c01
	// if(Pressure_Para[2]&0x8000) Pressure_Para[2] = Pressure_Para[2] - Total_Number_16;//c01
	// Pressure_Para[3] = (Press_Config7<<8)+Press_Config8;//c11
	// if(Pressure_Para[3]&0x8000) Pressure_Para[3] = Pressure_Para[3] - Total_Number_16;//c11
	// Pressure_Para[4] = (Press_Config9<<8)+Press_Config10;//c20
	// if(Pressure_Para[4]&0x8000) Pressure_Para[4] = Pressure_Para[4] - Total_Number_16;//c20
	// Pressure_Para[5] = (Press_Config11<<8)+Press_Config12;//c21
	// if(Pressure_Para[5]&0x8000) Pressure_Para[5] = Pressure_Para[5] - Total_Number_16;//c21
	// Pressure_Para[6] = (Press_Config13<<8)+Press_Config14;//c30
	// if(Pressure_Para[6]&0x8000) Pressure_Para[6] = Pressure_Para[6] - Total_Number_16;//c30
}

//压力校正函数
float Correcting_Pressure(int *Pressure_Para,float Pressure,float Temperature)
{
		float	Corr_Pressure;
	  Corr_Pressure = Pressure_Para[0]+ Pressure*(Pressure_Para[1]+Pressure*(Pressure_Para[4]+Pressure*Pressure_Para[6]))+Temperature*Pressure_Para[2]+Temperature*Pressure*(Pressure_Para[3]+Pressure*Pressure_Para[5]);
		return Corr_Pressure;
}
 
//温度校正函数
float Correcting_Temperature(int *Temperature_Para,float Temperature)
{	
	float Corr_Temperature;
	Corr_Temperature = Temperature_Para[0]*0.5+Temperature_Para[1]*Temperature;
	return Corr_Temperature;
}


	uint8_t i;
	uint8_t Config, Config_Press, Config_Temp;
	uint8_t Pressure_MSB, Pressure_CSB, Pressure_LSB;
	uint8_t Temp_MSB, Temp_CSB, Temp_LSB;
	uint8_t SPL06_ID;
	uint8_t meas_cfg; //初始化数据就绪
	
	int Temperature, Pressure;
	int Pressure_Para[7], Temperature_Para[2];
	float k_Press, k_Temp;
    
    BARO_Data BARO_Data_Now;
    // float Actual_Temperature, Actual_Pressure, Actual_Altitude;
	// float Correcting_Temp, Correcting_Press;

void INIT_BARO(void)
{

	while(SPL06_ID != 0x10){
		// 初始化SPL06传感器
		SPL06_ID = SPL06_Init();
//		//串口测试
//		uint8_t rxbuff0[1];
//		rxbuff0[0] = SPL06_ID;
//		HAL_UART_Transmit(&huart4,rxbuff0 , 1, 100);
	}

	//  必须在读取校准系数和配置寄存器之前，等待传感器初始化完成和校准系数就绪。
    while ((meas_cfg & 0xC0) != 0xC0 ) // 0xC0 = 11000000（COEF_RDY=1且SENSOR_RDY=1）
	{
        meas_cfg = SPL06_Read_Byte(MEAS_CFG_Addr); // 读取MEAS_CFG寄存器（0x08）
		delay_ms(100);
    }
	delay_ms(100);
	// 读取校准系数（来自COEF寄存器0x10-0x21，用于补偿计算）
	Parameter_Reading(Pressure_Para, Temperature_Para);
	// 读取压力配置寄存器（PRS_CFG，0x06）和温度配置寄存器（TMP_CFG，0x07）
	Config_Press = SPL06_Read_Byte(PRS_CFG_Addr);
	Config_Temp = SPL06_Read_Byte(TMP_CFG_Addr);


	// 根据过采样率确定补偿比例因子（对应规格书5.6.3节表4）
	k_Press = Scale_factor((Config_Press) & 0x0F);//排除无效位后带入匹配比例因子函数
	k_Temp = Scale_factor((Config_Temp) & 0x07);

		// float rx[1];
		// rx[0] = k_Press;
		// HAL_UART_Transmit(&huart4, (uint8_t*)rx, sizeof(rx), 100); //发送数据给串口

		
		// float rx1[1];
		// rx1[0] = k_Temp;
		// HAL_UART_Transmit(&huart4, (uint8_t*)rx1, sizeof(rx1), 100); //发送数据给串口
    delay_ms(300);


}

 void  BARO_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    switch (GPIO_Pin)
    {
    case I2C1_DR_Pin:	
    {
//    	//测试中断
//    	     uint8_t TEXTACC[1] = {0x01};
//    	         HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

		Pressure = 0;
		Temperature = 0;

        uint8_t int_sts;
        int_sts = SPL06_Read_Byte(INT_STS_Addr);

    //检查压力就绪中断（bit0）    
    if ((int_sts & 0x01) == 0x01)
    {
//		     	     uint8_t TEXTACC[1] = {0x07};
//   	          HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口
		
        // 读取压力原始数据（PSR_B2-B0，0x00-0x02）
        Pressure_MSB = SPL06_Read_Byte(PSR_B2_Addr);
        Pressure_CSB = SPL06_Read_Byte(PSR_B1_Addr);
        Pressure_LSB = SPL06_Read_Byte(PSR_B0_Addr);
        // Pressure = (Pressure_MSB << 16) + (Pressure_CSB << 8) + Pressure_LSB;
		// 拼接为24位数据
        uint32_t raw_24bit = (uint32_t)Pressure_MSB << 16 | (uint32_t)Pressure_CSB << 8 | Pressure_LSB;

        // 符号位扩展为32位有符号整数
  		if (raw_24bit & 0x800000) {  // 若24位最高位为1（负数）
    	Pressure = (int)raw_24bit -Total_Number_24; // 扩展高8位为1
		} else {
    	Pressure = (int)raw_24bit;  // 正数直接转换
		}
		// float rx[1];
		// rx[0] = Pressure;
		// HAL_UART_Transmit(&huart4, (uint8_t*)rx, sizeof(rx), 100); //发送数据给串口
		
        // 压力原始值缩放（对应规格书5.6.1节Praw_SC = Praw / kP）
		BARO_Data_Now.Actual_pressure = Pressure_conversion(Pressure, k_Press);
		// HAL_UART_Transmit(&huart4, (uint8_t*)&BARO_Data_Now.Actual_altitude, sizeof(BARO_Data_Now.Actual_altitude), 100);
        
		// 压力补偿计算（代入c00等7个系数，包含温度交叉补偿项）//那刚刚上电时的气压是不准的，要等温度读取更新后才准
		BARO_Data_Now.press_correction = Correcting_Pressure(Pressure_Para, BARO_Data_Now.Actual_pressure, BARO_Data_Now.Actual_temperature) + Offest_Pressure;

        // 计算海拔高度（对应规格书6.3节国际气压公式）
		BARO_Data_Now.Actual_altitude = 44330 * (1 - pow(BARO_Data_Now.press_correction / Standard_atmospheric_pressure, 1.0 / 5.255));
    }

    // 检查温度数据就绪中断（INT_TMP，bit1）
    if ((int_sts & 0x02) == 0x02)
    {
//		     	     uint8_t TEXTACC[1] = {0x08};
//   	          HAL_UART_Transmit(&huart4, TEXTACC, 1, 100); //发送数据给串口

        // 读取温度原始数据（TMP_B2-B0，0x03-0x05）
        Temp_MSB = SPL06_Read_Byte(TMP_B2_Addr);
        Temp_CSB = SPL06_Read_Byte(TMP_B1_Addr);
        Temp_LSB = SPL06_Read_Byte(TMP_B0_Addr);
        // Temperature = (Temp_MSB << 16) + (Temp_CSB << 8) + Temp_LSB;
		// 拼接为24位数据
  		uint32_t raw_24bit = (uint32_t)Temp_MSB << 16 | (uint32_t)Temp_CSB << 8 | Temp_LSB;

		// 符号位扩展为32位有符号整数
		if (raw_24bit & 0x800000) {  // 若24位最高位为1（负数）
   		 Temperature = (int)raw_24bit -Total_Number_24;  // 扩展高8位为1
		} else {
    	Temperature = (int)raw_24bit;  // 正数直接转换
		}
        
		// 温度原始值缩放（对应规格书5.6.2节Traw_SC = Traw / kT）
		BARO_Data_Now.Actual_temperature = Temperature_conversion(Temperature, k_Temp) ;
		// 温度补偿计算（代入c0、c1系数，公式：Tcomp = c0*0.5 + c1*Traw_SC）
		BARO_Data_Now.temp_correction = Correcting_Temperature(Temperature_Para, BARO_Data_Now.Actual_temperature) ;
    }

    break;
    }

    default:
        break;
    }
}

