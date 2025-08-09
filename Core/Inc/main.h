/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS2_ACC_Pin GPIO_PIN_13
#define CS2_ACC_GPIO_Port GPIOC
#define INT_GYRO2_Pin GPIO_PIN_3
#define INT_GYRO2_GPIO_Port GPIOE
#define INT_GYRO2_EXTI_IRQn EXTI3_IRQn
#define INT_ACC2_Pin GPIO_PIN_4
#define INT_ACC2_GPIO_Port GPIOE
#define INT_ACC2_EXTI_IRQn EXTI4_IRQn
#define I2C1_DR_Pin GPIO_PIN_5
#define I2C1_DR_GPIO_Port GPIOB
#define I2C1_DR_EXTI_IRQn EXTI9_5_IRQn
#define CS_FRAM_Pin GPIO_PIN_4
#define CS_FRAM_GPIO_Port GPIOD
#define CS2_GYRO_Pin GPIO_PIN_2
#define CS2_GYRO_GPIO_Port GPIOC
#define INT_ACC_Pin GPIO_PIN_0
#define INT_ACC_GPIO_Port GPIOA
#define INT_ACC_EXTI_IRQn EXTI0_IRQn
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOB
#define INT_GYRO_Pin GPIO_PIN_1
#define INT_GYRO_GPIO_Port GPIOA
#define INT_GYRO_EXTI_IRQn EXTI1_IRQn
#define CS_ACC_Pin GPIO_PIN_2
#define CS_ACC_GPIO_Port GPIOA
#define CS_GYRO_Pin GPIO_PIN_3
#define CS_GYRO_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
