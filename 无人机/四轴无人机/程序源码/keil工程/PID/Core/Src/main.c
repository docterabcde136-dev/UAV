/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//C Lib
#include <stdio.h>

//FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"

//HAL

//BSP
#include "bsp_led.h"
#include "bsp_oled.h"
#include "bsp_adc.h"
#include "bsp_imu.h"
#include "bsp_LC307.h"
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
uint8_t uart1_recv,uart4_recv;
float g_userparam_pitchzero = 0 , g_userparam_rollzero = 0;
DebugShowVal_t g_debugVal = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	pOLEDInterface_t oled = &UserOLED;
	oled->init();
	
	//蓝牙、调试串口
	HAL_UART_Receive_IT(&huart4,&uart4_recv,1);
	HAL_UART_Receive_IT(&huart1,&uart1_recv,1);
	
	//用户调试定时器
	HAL_TIM_Base_Start(&htim6);

	//ADC初始化
	pADCInterface_t adc1 = &UserADC1;
	adc1->init();

	//使用硬件IIC时需要
	//检查硬件IIC是否出现BUSY异常
	if( (I2C1->SR2>>1)&0x01 )
	{
		HAL_I2C_MspDeInit(&hi2c1);

		//将IIC总线强制释放
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);

		//重新初始化IIC
		I2C1->CR1 &= ~(1<<0);//禁止IIC外设
		HAL_Delay(50);
		I2C1->CR1 |= (1<<0);//重新使能
		I2C1->CR1 |= (1<<15);//进行软件复位
		HAL_Delay(50);
		I2C1->CR1 &= ~(1<<15);
		MX_I2C1_Init();
	}

  //icm20948初始化与状态提示
  pLedInterface_t imu_initled = &UserLed1;
  pIMUInterface_t imu = &UserICM20948;

  imu_initled -> off();
  while( imu->Init() )
  {
	  imu_initled -> toggle();//初始化过程出错,闪烁
	  HAL_Delay(60);
	  NVIC_SystemReset();
  }
  imu_initled -> on();//初始化完成,亮灯提示

   //光流模块初始化
  Opf_LC307_Init();
  
  //读取用户设定的零点数据
  extern void User_Flash_ReadParam(uint32_t* p,uint16_t datalen);
  int32_t tmp[2] = { 0 } ;
  User_Flash_ReadParam((uint32_t*)tmp,2);
  if( tmp[0]!=0xffffffff )
	  g_userparam_pitchzero = *((float*)&tmp[0]);
  if( tmp[1]!=0xffffffff )
	  g_userparam_rollzero = *((float*)&tmp[1]);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

UART_HandleTypeDef *DebugSerial = &huart1;

//实现printf、scanf函数
int fputc(int ch,FILE* stream)
{
	while( HAL_OK != HAL_UART_Transmit(DebugSerial,(const uint8_t *)&ch,1,100));
	return ch;
}

static xSemaphoreHandle debug_1_HandleMutex;
static char g_last_char;
static char g_backspace;

int fgetc(FILE* f)
{
	//没有初始化则调用初始化函数
	static uint8_t init = 0;
	if( init == 0 )
	{
		init = 1;
		debug_1_HandleMutex = xSemaphoreCreateMutex();
	}
	
	int ch;
	if( g_backspace )
	{
		xSemaphoreTake(debug_1_HandleMutex,portMAX_DELAY);
		g_backspace = 0 ;
		xSemaphoreGive(debug_1_HandleMutex);
		
		return g_last_char;
	}
	
	while( HAL_OK!= HAL_UART_Receive(DebugSerial,(uint8_t *)&ch,1,HAL_MAX_DELAY) );
	g_last_char = ch;
	return ch;
}

int __backspace(FILE *stream)
{
	xSemaphoreTake(debug_1_HandleMutex,portMAX_DELAY);
	g_backspace = 1 ;
	xSemaphoreGive(debug_1_HandleMutex);
	return 0;
}

//定时器8更新中断回调函数
__weak void User_TIM8_UpdateCallback(void)
{
	
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else if(TIM8 == htim->Instance)
	{
		User_TIM8_UpdateCallback();
	}
	
  /* USER CODE END Callback 1 */
}

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
