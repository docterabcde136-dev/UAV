/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define OLED_SCL_Pin GPIO_PIN_13
#define OLED_SCL_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_14
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_RES_Pin GPIO_PIN_15
#define OLED_RES_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOC
#define Lidar_TX_Pin GPIO_PIN_2
#define Lidar_TX_GPIO_Port GPIOA
#define Lidar_RX_Pin GPIO_PIN_3
#define Lidar_RX_GPIO_Port GPIOA
#define User_BUZZER_Pin GPIO_PIN_4
#define User_BUZZER_GPIO_Port GPIOA
#define Battery_Ch_Pin GPIO_PIN_7
#define Battery_Ch_GPIO_Port GPIOA
#define cur_ch_Pin GPIO_PIN_1
#define cur_ch_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_12
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI15_10_IRQn
#define User_KEY_Pin GPIO_PIN_13
#define User_KEY_GPIO_Port GPIOB
#define User_LED1_Pin GPIO_PIN_14
#define User_LED1_GPIO_Port GPIOB
#define User_LED2_Pin GPIO_PIN_15
#define User_LED2_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_10
#define BT_TX_GPIO_Port GPIOC
#define BT_RX_Pin GPIO_PIN_11
#define BT_RX_GPIO_Port GPIOC
#define STP23L_TX_Pin GPIO_PIN_12
#define STP23L_TX_GPIO_Port GPIOC
#define STP23L_RX_Pin GPIO_PIN_2
#define STP23L_RX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

extern uint8_t uart1_recv,uart4_recv;

typedef struct{
	float f1;
	float f2;
	float f3;
	float f4;
	float f5;
	float f6;
	float f7;
	float f8;
	
	short s1;
	short s2;
	short s3;
	short s4;
	short s5;
	short s6;
	short s7;
	short s8;
	
}DebugShowVal_t;

extern DebugShowVal_t g_debugVal;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
