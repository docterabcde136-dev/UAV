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
/* C 标准库 ------------------------------------------------------------------*/
#include <stdio.h>              /* 标准输入输出，提供 printf/scanf 重定向支持 */

/* FreeRTOS ------------------------------------------------------------------*/
#include "FreeRTOS.h"           /* FreeRTOS 内核接口 */
#include "semphr.h"             /* 信号量/互斥锁接口 */

/* HAL 库 --------------------------------------------------------------------*/
/* （HAL 外设头文件已在 main.h 中通过 stm32f4xx_hal.h 统一引入） */

/* BSP 板级支持包 -------------------------------------------------------------*/
#include "bsp_led.h"            /* LED 指示灯驱动 */
#include "bsp_oled.h"           /* OLED 显示屏驱动 */
#include "bsp_adc.h"            /* ADC 采样驱动（电池电压/电流检测） */
#include "bsp_imu.h"            /* IMU 惯性测量单元驱动（ICM20948） */
#include "bsp_LC307.h"          /* LC307 激光测距模块驱动 */
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
/* 串口中断接收缓存 -----------------------------------------------------------*/
uint8_t uart1_recv;             /* USART1 单字节中断接收缓存（蓝牙模块） */
uint8_t uart4_recv;             /* UART4  单字节中断接收缓存（调试串口） */

/* 用户校准参数 ---------------------------------------------------------------*/
float g_userparam_pitchzero = 0; /* 用户设定的俯仰角零点偏移（从 Flash 读取） */
float g_userparam_rollzero  = 0; /* 用户设定的横滚角零点偏移（从 Flash 读取） */

/* 调试数据显示结构体 ---------------------------------------------------------*/
DebugShowVal_t g_debugVal = { 0 }; /* 调试用全局变量，可通过串口/OLED 查看各传感器数据 */
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

	/* OLED 显示屏初始化 ----------------------------------------------------*/
	/* 使用 I2C1 接口驱动 OLED 显示模块 */
	pOLEDInterface_t oled = &UserOLED;
	oled->init();

	/* 启动串口中断接收 ------------------------------------------------------*/
	/* UART4: 调试串口（printf 重定向），UART1: 蓝牙模块，均使能单字节中断接收 */
	HAL_UART_Receive_IT(&huart4, &uart4_recv, 1);
	HAL_UART_Receive_IT(&huart1, &uart1_recv, 1);

	/* 启动用户基础定时器 ----------------------------------------------------*/
	/* TIM6 用作系统时基或周期任务调度时钟源 */
	HAL_TIM_Base_Start(&htim6);

	/* ADC 初始化 ------------------------------------------------------------*/
	/* ADC1 用于采集电池电压 (Battery_Ch) 和电流检测通道 (cur_ch) */
	pADCInterface_t adc1 = &UserADC1;
	adc1->init();

	/* 硬件 I2C 总线忙异常恢复处理 -------------------------------------------*/
	/* 上电或异常复位后，I2C 从设备可能将 SDA 拉低导致 BUSY 标志置位 */
	/* 此处检测并强制释放 I2C 总线，然后重新初始化外设 */
	if ((I2C1->SR2 >> 1) & 0x01)       /* 检查 I2C1 SR2 寄存器的 BUSY 标志位 */
	{
		HAL_I2C_MspDeInit(&hi2c1);      /* 反初始化 I2C MSP（释放引脚复用回 GPIO） */

		/* 将 I2C 引脚 (PB6=SCL, PB7=SDA) 配置为 GPIO 推挽输出，模拟时钟释放总线 */
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull  = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); /* SCL 拉高 */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); /* SDA 拉高 */

		/* 重新初始化 I2C1 外设，清除 BUSY 状态 */
		I2C1->CR1 &= ~(1 << 0);         /* PE=0: 关闭 I2C 使能 */
		HAL_Delay(50);                  /* 等待总线稳定 */
		I2C1->CR1 |= (1 << 0);          /* PE=1: 重新使能 I2C */
		I2C1->CR1 |= (1 << 15);         /* SWRST=1: 软件复位 I2C */
		HAL_Delay(50);
		I2C1->CR1 &= ~(1 << 15);        /* 清除复位位 */
		MX_I2C1_Init();                 /* 重新调用 CubeMX 生成的 I2C 初始化函数 */
	}

	/* ICM20948 IMU 初始化与状态指示 -----------------------------------------*/
	/* 使用 User_LED1 指示 IMU 初始化状态：闪烁=正在初始化，常亮=初始化成功 */
	pLedInterface_t  imu_initled = &UserLed1;
	pIMUInterface_t  imu         = &UserICM20948;

	imu_initled->off();
	while (imu->Init())                   /* 循环初始化 IMU 直到成功 */
	{
		imu_initled->toggle();            /* LED 闪烁表示初始化过程持续中 */
		HAL_Delay(60);
		NVIC_SystemReset();               /* 初始化超时则执行系统复位重试 */
	}
	imu_initled->on();                    /* LED 常亮指示 IMU 初始化成功完成 */

	/* 激光测距模块初始化 ----------------------------------------------------*/
	/* LC307 / STP23L 激光测距模块，用于对地高度测量 */
	Opf_LC307_Init();

	/* 从 Flash 读取用户校准参数 ---------------------------------------------*/
	/* 读取存储的俯仰角和横滚角零点偏移值（出厂校准或用户设定值） */
	extern void User_Flash_ReadParam(uint32_t* p, uint16_t datalen);
	int32_t tmp[2] = { 0 };
	User_Flash_ReadParam((uint32_t*)tmp, 2);
	if (tmp[0] != 0xffffffff)             /* 0xFFFFFFFF 表示 Flash 该地址未被写入过（擦除态） */
		g_userparam_pitchzero = *((float*)&tmp[0]);
	if (tmp[1] != 0xffffffff)
		g_userparam_rollzero  = *((float*)&tmp[1]);

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
    /* FreeRTOS 任务调度运行中，此循环体由 osKernelStart() 内部的调度器控制 */
    /* 用户任务代码在各 APP 任务文件中实现（balance_task, lidar_task 等） */
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

/* 调试串口句柄 ---------------------------------------------------------------*/
/* 将 huart4 重命名为 DebugSerial，用于 printf/scanf 重定向 */
UART_HandleTypeDef *DebugSerial = &huart4;

/**
  * @brief  printf 重定向底层函数：将字符通过串口发送
  * @param  ch:     要发送的字符
  * @param  stream: 文件流指针（未使用）
  * @retval 发送的字符
  * @note   重写 C 库的 fputc 实现 printf 串口输出
  */
int fputc(int ch, FILE* stream)
{
	while (HAL_OK != HAL_UART_Transmit(DebugSerial, (const uint8_t *)&ch, 1, 100));
	return ch;
}

/* scanf 重定向所需的互斥锁和状态变量 -----------------------------------------*/
static xSemaphoreHandle debug_1_HandleMutex;  /* 保护 scanf 回退缓冲区的互斥锁 */
static char g_last_char;                       /* 上次接收的字符（用于 backspace 回退） */
static char g_backspace;                       /* backspace 回退标志（非零表示下次读取返回缓存字符） */

/**
  * @brief  scanf 重定向底层函数：从串口接收字符
  * @param  f: 文件流指针（未使用）
  * @retval 接收到的字符
  * @note   重写 C 库的 fgetc 实现 scanf 串口输入，支持 backspace 功能
  */
int fgetc(FILE* f)
{
	/* 首次调用时创建互斥锁（延迟初始化） */
	static uint8_t init = 0;
	if (init == 0)
	{
		init = 1;
		debug_1_HandleMutex = xSemaphoreCreateMutex();
	}

	int ch;
	if (g_backspace)
	{
		/* 处理 backspace：返回上次缓存的字符而不是从串口读取 */
		xSemaphoreTake(debug_1_HandleMutex, portMAX_DELAY);
		g_backspace = 0;
		xSemaphoreGive(debug_1_HandleMutex);

		return g_last_char;
	}

	/* 阻塞等待串口接收一个字符 */
	while (HAL_OK != HAL_UART_Receive(DebugSerial, (uint8_t *)&ch, 1, HAL_MAX_DELAY));
	g_last_char = ch;
	return ch;
}

/**
  * @brief  scanf backspace 支持函数：将上次读取的字符回退到输入流
  * @param  stream: 文件流指针（未使用）
  * @retval 0
  * @note   配合 fgetc 实现 scanf 的退格编辑功能
  */
int __backspace(FILE *stream)
{
	xSemaphoreTake(debug_1_HandleMutex, portMAX_DELAY);
	g_backspace = 1;
	xSemaphoreGive(debug_1_HandleMutex);
	return 0;
}

/**
  * @brief  TIM8 定时器更新中断回调函数（弱定义）
  * @note   用户可重写此函数实现 TIM8 周期中断自定义处理
  *         当前由 HAL_TIM_PeriodElapsedCallback 中调用
  */
__weak void User_TIM8_UpdateCallback(void)
{
	/* 用户在此添加 TIM8 中断处理代码 */
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
	else if (TIM8 == htim->Instance)
	{
		/* TIM8 周期中断：调用用户自定义回调 */
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
