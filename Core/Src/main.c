/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "light_flow.h"
#include "flow_lowpass.h"
//#include "BMI088.h"
#include <stdbool.h> // 包含 bool 类型定义
#include "baro.h"
#include "flash.h"
#include <string.h>
#include <stdio.h> // 包含标准输入输出函数原型
#include "position_estimator.h"
#include "process.h"
#include "imu_selector.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF); //  huart4串口重定向，将c语言的printf函数重定向到串口4（要包含stdio.h）
    return ch;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern DMA_HandleTypeDef hdma_tim1_ch2;
extern DMA_HandleTypeDef hdma_tim1_ch3;
extern DMA_HandleTypeDef hdma_tim1_ch4;

extern MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;

extern DMA_HandleTypeDef hdma_adc1;  // ADC1的DMA句柄
extern DMA_HandleTypeDef hdma_adc2;  // ADC2的DMA句柄

uint8_t rx_buffer[1];
bool BMIx = false;//false对应spi1的bmi088，true对应spi4的bmi088_2

//BMIx为0时对应spi1的bmi088，为1时对应spi4的bmi088_2

// 全局变量：油门
 volatile uint16_t throttle_value = 200;
 uint16_t throttle_motor[4] = {0,0,0,0};
 uint8_t slow_down = 0;
 uint8_t start_fly = 0;
 uint8_t q = 0;
 int maxmotor = 0;
// int maxmotor = 1420;
 int xxx = 1;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口中断调试
{
  if (huart == &huart4) {
	  printf("%d \r\n",rx_buffer[0]);
    switch (rx_buffer[0])
    {
    case 49://1
    	/*----------------BMI088陀螺仪/加速度计数据读取----------------*/
        //读取IMU数据
        IMU_Read(BMIx);
        printf("gyro_x: %2f \r\n",BMI088.gyro.dps[xx]);
        printf("gyro_y: %2f \r\n",BMI088.gyro.dps[yy]);
        printf("gyro_z: %2f \r\n",BMI088.gyro.dps[zz]);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088.gyro.dps[xx], sizeof(BMI088.gyro.dps[xx]), 100);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088.gyro.dps[yy], sizeof(BMI088.gyro.dps[yy]), 100);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088.gyro.dps[zz], sizeof(BMI088.gyro.dps[zz]), 100);

      break;

    case 50://2
        //读取IMU数据
        IMU_Read(BMIx);
        printf("acc_x: %2f \r\n",BMI088.acc.m_s_2[xx]);
        printf("acc_y: %2f \r\n",BMI088.acc.m_s_2[yy]);
        printf("acc_z: %2f \r\n",BMI088.acc.m_s_2[zz]);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088.acc.m_s_2[xx], sizeof(BMI088.acc.m_s_2[xx]), 100);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088.acc.m_s_2[yy], sizeof(BMI088.acc.m_s_2[yy]), 100);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088.acc.m_s_2[zz], sizeof(BMI088.acc.m_s_2[zz]), 100);

      break;

    case 51://3
        //读取IMU数据
        IMU_Read(!BMIx);
    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088_2.gyro.dps[xx], sizeof(BMI088_2.gyro.dps[xx]), 100);
    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088_2.gyro.dps[yy], sizeof(BMI088_2.gyro.dps[yy]), 100);
    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088_2.gyro.dps[zz], sizeof(BMI088_2.gyro.dps[zz]), 100);
//    printf("距离 =  %d cm \r\n",payload.distance/10);
//    printf("light_flow_x：%d ,speed_x= %d \r\n",payload.flow_vel_x, (int64_t)payload.flow_vel_x * payload.distance/1000);
//    printf("light_flow_y：%d ,speed_y= %d \r\n",payload.flow_vel_y, (int64_t)payload.flow_vel_y * payload.distance/1000);

      break;

    case 52://4
        //读取IMU数据
        IMU_Read(!BMIx);
    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088_2.acc.m_s_2[xx], sizeof(BMI088_2.acc.m_s_2[xx]), 100);
    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088_2.acc.m_s_2[yy], sizeof(BMI088_2.acc.m_s_2[yy]), 100);
    HAL_UART_Transmit(&huart4, (uint8_t*)&BMI088_2.acc.m_s_2[zz], sizeof(BMI088_2.acc.m_s_2[zz]), 100);

      break;

      /*----------------气压计数据读取----------------*/
    case 53://5
    	maxmotor = 300;
    	printf("\r\n\r\n\r\nthrottle_value: %d \r\n\r\n\r\n",maxmotor);
        //读取BARO数据
//    	printf("气压：%2f \r\n",BARO_Data_Now.press_correction);
//    	printf("温度：%2f \r\n",BARO_Data_Now.temp_correction);
//    	printf("海拔：%2f \r\n",BARO_Data_Now.Actual_altitude);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BARO_Data_Now.press_correction, sizeof(BARO_Data_Now.Actual_pressure), 100);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BARO_Data_Now.temp_correction, sizeof(BARO_Data_Now.Actual_temperature), 100);
//    HAL_UART_Transmit(&huart4, (uint8_t*)&BARO_Data_Now.Actual_altitude, sizeof(BARO_Data_Now.Actual_altitude), 100);

      break;

      /*----------------油门调整----------------*/
      case 54://6
          // 实现油门急停
    	  start_fly = 0;
    	  slow_down = 0;
          throttle_value = 0;  // 将油门值设置为0
          for (uint8_t m = 0; m < 4; ++m) {
              DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
          }
          printf("\r\n Stop! \r\n\r\n");
          break;

      case 55://7
    	  if(xxx==0)
    	  {
    		  throttle_value = 1100;
				for (uint8_t m = 0; m < 4; ++m) {
					DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
				}
        	  xxx = 2;
        	  xxx = 2;
        	  xxx = 2;
    	  }


    	  if(xxx==1)
    	  {
    		  throttle_value = 750;
				for (uint8_t m = 0; m < 4; ++m) {
					DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
				}
        	  xxx = 0;
        	  xxx = 0;
        	  xxx = 0;
    	  }



          // 实现缓加油门，每次增加50，但不超过1000
//
//        	  if (throttle_value < 1200)
//        	  {  // 确保油门值不超过1000
//               throttle_value += 100;
//        	  }  // 增加油门值100
//
//              if (throttle_value == 1000)
//              {
//               throttle_value = 1200;  // 如果超过1000，则设置为1000
//              }

//              for (uint8_t m = 0; m < 4; ++m) {
//                  DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
//              }
//              DSHOT_Update(0, throttle_value);
//              DSHOT_Update(q, throttle_value);
//              q++;
//              if(q==4)q=0;

          printf("throttle_value: %d \r\n\r\n",throttle_value);
//    	  if(xxx==0)
//    	  {
//        	  maxmotor += 20;
//        	  printf("\r\n\r\n\r\nthrottle_value: %d \r\n\r\n\r\n",maxmotor);
//    	  }

          break;

      case 56://8
//    	  if (throttle_value > 100)
//    	  {
//    		  throttle_value -= 50;
//              if (throttle_value < 100) {
//                  throttle_value = 50;  // 如果超过1000，则设置为1000
//              }
//              for (uint8_t m = 0; m < 4; ++m) {
//              DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
//              }
//          }
//    	  printf("throttle_value: %d \r\n\r\n",throttle_value);
    	  maxmotor -= 50;
    	  printf("\r\n\r\n\r\nthrottle_value: %d \r\n\r\n\r\n",maxmotor);
          break;

      case 57://9,缓降
    	  start_fly = 0;

    	  slow_down = 1;
    	  break;

      case 48://0，一键起飞
    	  start_fly = 1;
    	  break;

    default:
      break;
    
    }
    
}
  //重启串口4接收中断
  HAL_UART_Receive_IT(&huart4, rx_buffer, 1);

}

// TIM1\CH1\DMA传输完成中断
uint8_t r = 0;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
//    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//    {
//    	//TIM1\CH1\DMA传输完成中断
//    	r++;
//    	if(r=2)
//    	{
//            for (uint8_t m = 0; m < 4; ++m) {
//                DSHOT_Update(m, 0, 1);  // 更新所有四个电机的油门值
//            }
//            r = 0;
//    	}
//
//    }
}

// ADC采样完成中断
  uint16_t bat=0;
  double curr=0;
  float BAT_V = 0.0;
  double Dc_Motor_Current = 0.0;

  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
  {
      if (hadc == &hadc1)
      {
    	  bat = HAL_ADC_GetValue(&hadc1);
          BAT_V = bat * 3.3f / 65535.0f;      //电池电压
      }
      else if (hadc == &hadc2)
      {
    	  curr = HAL_ADC_GetValue(&hadc2);
          Dc_Motor_Current = (curr * 3.3f / 65535.0f) / 0.01275f; //电机电流,12.75mV/A
      }

  }

  uint8_t rx_buffer2[64]; // usart2接收光流数据缓冲区
  extern DMA_HandleTypeDef hdma_usart2_rx;
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size) {
      if (huart == &huart2) {

    	  micolink_decode(rx_buffer2[0]);//micolink光流数据处理
    	  micolink_decode(rx_buffer2[1]);
    	  micolink_decode(rx_buffer2[2]);
    	  micolink_decode(rx_buffer2[3]);
    	  micolink_decode(rx_buffer2[4]);
    	  micolink_decode(rx_buffer2[5]);
    	  micolink_decode(rx_buffer2[6]);
    	  micolink_decode(rx_buffer2[7]);
    	  micolink_decode(rx_buffer2[8]);
    	  micolink_decode(rx_buffer2[9]);
    	  micolink_decode(rx_buffer2[10]);
    	  micolink_decode(rx_buffer2[11]);
    	  micolink_decode(rx_buffer2[12]);
    	  micolink_decode(rx_buffer2[13]);
    	  micolink_decode(rx_buffer2[14]);
    	  micolink_decode(rx_buffer2[15]);
    	  micolink_decode(rx_buffer2[16]);
    	  micolink_decode(rx_buffer2[17]);
    	  micolink_decode(rx_buffer2[18]);
    	  micolink_decode(rx_buffer2[19]);
    	  micolink_decode(rx_buffer2[20]);
  		  micolink_decode(rx_buffer2[21]);
  		  micolink_decode(rx_buffer2[22]);
  		  micolink_decode(rx_buffer2[23]);
  		  micolink_decode(rx_buffer2[24]);
  		  micolink_decode(rx_buffer2[25]);
  		  micolink_decode(rx_buffer2[26]);

      }
  }

  uint16_t t=0;
  uint16_t tt=0;
  uint8_t bat_cur=0;
  uint8_t circle=0;
  uint8_t select_imu=0;
  bool use_imu = false;//选择使用哪个imu

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_SPI4_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  
  /*--------光流传感器--------*/
  // 启用串口2接收DMA+idle（非阻塞模式）
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer2, sizeof(rx_buffer2));
//  //关闭dma过半中断
//  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);


  /*--------气压计--------*/
//   INIT_BARO();

  //开定时器6中断
  HAL_TIM_Base_Start_IT(&htim6);

  //开启串口4接收中断
  HAL_UART_Receive_IT(&huart4, rx_buffer, 1);

  /*-----------初始化并使能BMI088-----------*/
   BMI088_FLOAT_ACC_GYRO_Init(&hspi1);
   BMI088_FLOAT_ACC_GYRO_Init(&hspi4);

  /*--------FLASH--------*/
//    Init_FM25Vx();    //初始化flash：FM25V20A
//
//    FM25Vx_text();    //测试函数

  /*-----------DMA+PWM输出DShot300信号-----------*/
  /* 先关 HT 中断，只留 TC */
   __HAL_DMA_DISABLE_IT(&hdma_tim1_ch1, DMA_IT_HT);
   __HAL_DMA_DISABLE_IT(&hdma_tim1_ch2, DMA_IT_HT);
   __HAL_DMA_DISABLE_IT(&hdma_tim1_ch3, DMA_IT_HT);
   __HAL_DMA_DISABLE_IT(&hdma_tim1_ch4, DMA_IT_HT);
  DSHOT_Start();    // 启动 4 路循环发送
  DSHOT_Update(0, 0);  // 电机 0
  DSHOT_Update(1, 0);  // 电机 1
  DSHOT_Update(2, 0);  // 电机 2
  DSHOT_Update(3, 0);  // 电机 3

  /*-----------ADC采样BAT电压与电调电流-----------*/
  //ADC校准参考电压
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  // 启动 ADC 并使能中断
  HAL_ADC_Start_IT(&hadc1);  // 关键：用 _IT 函数启动，自动使能中断
  HAL_ADC_Start_IT(&hadc2);


//  float a_x,a_y,b_x,b_y;
// extern float m_x;
// extern float m_y;
//  float c_x = 0;
//  float c_y = 0;
//  float d_x, d_y;


  extern motor_out_t motor_out; // 电机输出值，由 torque_to_motor 函数计算得到

  uint8_t fly_cnt = 1;
  uint16_t cnt = 0;

  HAL_TIM_Base_Start(&htim7);   // 放在 main 里或任务初始化里一次即可

  int del=0;
  bool del_flag=false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if(select_imu==1)//0.5s打分一次，选择使用哪个imu
	  {
		  select_imu=0;
//		  imu_selector_get_active(&use_imu);
//		  use_imu = use_imu ? false : true;  //false代表使用第一个imu，ture代表使用第二个imu
		  use_imu = true;

	if(del<4)//添加适当的延时，3s后再开始计算零偏
	{
		del++;
		if(del==3)
		{
			del_flag=true;
		}
	}

//      if(use_imu){
//    	  printf("_2\r\n");
//      }
//      else{
//    	  printf("_1\r\n");
//      }

	  }

//	  uint32_t start = __HAL_TIM_GET_COUNTER(&htim7); 测量时间代码段

//    需要测的代码

//	  uint32_t end = __HAL_TIM_GET_COUNTER(&htim7);
//
//	  uint32_t us = (end >= start) ? (end - start) : (0xFFFF - start + end);
//	  printf("耗时: %lu us\r\n", us);

	  if(circle == 1 && del_flag==true)
	  {
//		  uint32_t start = __HAL_TIM_GET_COUNTER(&htim7);

		    circle = 0;

	  	IMU_Read(use_imu);//读取第一个imu
	  	process_main();


//	  	uint32_t end = __HAL_TIM_GET_COUNTER(&htim7);
//	  	uint32_t us = (end >= start) ? (end - start) : (0xFFFF - start + end);
//	  	printf("耗时: %lu us\r\n", us);
//	  	  HAL_Delay(1);
	  }


//		 throttle_motor[0] = (motor_out.m[1]/10) *maxmotor;
//		 throttle_motor[1] = (motor_out.m[2]/10) *maxmotor;
//		 throttle_motor[2] = (motor_out.m[0]/10) *maxmotor;
//		 throttle_motor[3] = (motor_out.m[3]/10) *maxmotor;

////	  // 1. 读温度（把 raw 字节转成 ℃）
////	      BMI088_Read_Tmp_Data(false);   // 读取 BMI088 的温度
//	      float current_temp = BMI088.Temperature;
////
////	      // 2. 现在可以用 current_temp 做温度补偿或打印
//	      printf("IMU temp ℃\r\n\r\n");
//////	      HAL_Delay(1000);
//
//		 if(bat_cur == 1 ){
//			 bat_cur =0;
//			 printf("BAT_V=%.2f V\r\n",BAT_V*10);
//			 printf("CURR =%.2f A\r\n\r\n",Dc_Motor_Current);
//		 }

	  while (start_fly == 1)
	  {
			 IMU_Read(BMIx);
			 process_main();
			 HAL_Delay(0);


			 if(motor_out.m[0]!=0 )//校准完成
			 {
				 if(fly_cnt == 3)
				 {
					 maxmotor=1400;
					 fly_cnt = 0;

				 }
				 if(fly_cnt == 2)
				 {
					 maxmotor=1100;
					 cnt++;
					 if(cnt == 4000)
					 {
						 fly_cnt = 3;
						 cnt = 0;
					 }
				 }
				 if(fly_cnt == 1)
				 {
					 maxmotor=700;
					 cnt++;
					 if(cnt == 4000)
					 {
						 fly_cnt = 2;
						 cnt = 0;
					 }

				 }



//				 throttle_motor[0] = (motor_out.m[0]/10) *maxmotor;
//				 throttle_motor[1] = (motor_out.m[1]/10) *maxmotor;
//				 throttle_motor[2] = (motor_out.m[3]/10) *maxmotor;
//				 throttle_motor[3] = (motor_out.m[2]/10) *maxmotor;


				 throttle_motor[0] = (motor_out.m[2]/10) *maxmotor*1.096;
//				 throttle_motor[0] = (motor_out.m[2]/10) *maxmotor;
				 throttle_motor[1] = (motor_out.m[3]/10) *maxmotor;
				 throttle_motor[2] = (motor_out.m[0]/10) *maxmotor;
				 throttle_motor[3] = (motor_out.m[1]/10) *maxmotor;
				 for (uint8_t m = 0; m < 4; ++m)
				 {
					 DSHOT_Update(m, throttle_motor[m]);
				 }
			 }
	   }

		 if(slow_down==1)
		 {
			 slow_down=0;
			 start_fly == 0;
			 throttle_value = 800 ;
			  for (uint8_t m = 0; m < 4; ++m)
			  {
				  DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
			  }
			  HAL_Delay(800);
			 throttle_value = 600 ;
			  for (uint8_t m = 0; m < 4; ++m)
			  {
				  DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
			  }
			  HAL_Delay(500);
			 throttle_value = 400 ;
			  for (uint8_t m = 0; m < 4; ++m)
			  {
				  DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
			  }
			  HAL_Delay(300);
			 throttle_value = 200 ;
			  for (uint8_t m = 0; m < 4; ++m)
			  {
				  DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
			  }
			  HAL_Delay(300);
			 throttle_value = 0 ;
			  for (uint8_t m = 0; m < 4; ++m)
			  {
				  DSHOT_Update(m, throttle_value);  // 更新所有四个电机的油门值
			  }
              //停止
		 }
		 slow_down=0;


//	  //处理光流数据
//	  a_x= -payload.flow_vel_x;
//	  d_x=payload.distance*0.001;//距离m
//	  b_x=a_x*d_x;//cm/s
//	  b_x=flow_lpf(b_x, 0.5f);
//	  m_x=b_x*0.01;//m/s
//	  c_x+=m_x*0.05*2.0f;//m
////
//	  a_y=payload.flow_vel_y;
//	  d_y=payload.distance*0.001;//距离m
//	  b_y=a_y*d_y;//cm/s
//	  b_y=flow_lpf(b_y, 0.2f);
//	  m_y=b_y*0.01;//m/s
//	  c_y+=m_y*0.05*2.0f;//m
//      IMU_Read(BMIx);
//      printf("加速度_x：%2f \r\n",BMI088.acc.m_s_2[xx]);
//      printf("加速度_y：%2f \r\n",BMI088.acc.m_s_2[yy]);
//      printf("加速度_z：%2f \r\n",-BMI088.acc.m_s_2[zz]);
//

//   printf("BAT_V=%.2f\r\n\r\n",BAT_V);
////   printf("电机电流=%.2f\r\n",Dc_Motor_Current);
//

//   HAL_Delay(50);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 10;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//处理函数，重启dma+idle
void handler01(void) {

HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer2, sizeof(rx_buffer2));

}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if (htim->Instance == TIM6)
    {
      //定时器1ms中断服务
//	  void check_time_stability_simple(handler01);

	  t++;
	  tt++;
	  if(t==10000)
	  {
		  t=0;
		  select_imu = 1;
	  }
	  if(tt==25)
	  {
		  tt=0;
		  circle = 1;
	  }


//		  bat_cur = 1;
//		  uint8_t r[1];
//		  r[0]=1;
//		  HAL_UART_Transmit(&huart2, (uint8_t*)&r, sizeof(r), 100);
//		 printf("BAT_V=%.2f V\r\n",BAT_V*10);
//		 printf("CURR =%.2f A\r\n\r\n",Dc_Motor_Current);
//	        IMU_Read(BMIx);
//	        printf(" %2f %2f %2f \r\n",BMI088.acc.m_s_2[xx],BMI088.acc.m_s_2[yy],BMI088.acc.m_s_2[zz]);
	  }
//		 IMU_Read(BMIx);
//		 process_main();
}



  /* USER CODE END Callback 1 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
