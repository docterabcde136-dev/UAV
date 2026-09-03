/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
/**
 * ============================================================
 * ADC1 驱动模块 (MX_ADC1_Init 自动生成)
 * ============================================================
 * 硬件映射:
 *   PA7 (ADC1_IN7) → 电池电压检测 (Battery_Ch)
 *   PB1 (ADC1_IN9) → 电流检测       (cur_ch)
 *
 * 工作模式:
 *   - 12位分辨率, 扫描模式, 连续转换
 *   - DMA2_Stream0 循环搬运数据, 无需CPU干预
 *   - 采样时间 480 周期, 保证低速信号精度
 *
 * 数据流: ADC → DMA → 内存数组 → 使用者直接读取
 * ============================================================
 */
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /*
   * 在HAL外设初始化之前, 可在此处添加用户的预配置代码
   * 例如: 校准数据恢复、外部参考电压设置等
   */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /*
   * ADC句柄参数已配置完毕, 尚未写入硬件寄存器
   * 如有动态调整需求, 在此修改 hadc1.Init 结构体成员即可
   */
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* ADC通道7: 电池电压检测 (PA7)
   * Rank=1 表示在扫描序列中第一个被转换
   */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* ADC通道9: 电流检测 (PB1)
   * Rank=2 表示在扫描序列中第二个被转换
   */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /*
   * ADC初始化完成, 两路通道已配置
   * 后续可在此添加: 校准命令、用户自定义通道等
   */
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */
  /*
   * MSP (MCU Support Package) 初始化入口
   * CubeMX已生成: GPIO模拟输入配置 + DMA2_Stream0循环模式
   * 用户可在此添加额外的引脚配置、电源管理
   */
  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA7     ------> ADC1_IN7
    PB1     ------> ADC1_IN9
    */
    GPIO_InitStruct.Pin = Battery_Ch_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Battery_Ch_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = cur_ch_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(cur_ch_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */
  /*
   * MSP初始化后处理
   * DMA已链接到ADC句柄, 可在此启动首次转换或使能ADC中断
   */
  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */
  /*
   * MSP去初始化入口
   * 在低功耗模式或外设重配置时调用, 释放GPIO/DMA资源
   */
  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA7     ------> ADC1_IN7
    PB1     ------> ADC1_IN9
    */
    HAL_GPIO_DeInit(Battery_Ch_GPIO_Port, Battery_Ch_Pin);

    HAL_GPIO_DeInit(cur_ch_GPIO_Port, cur_ch_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */
  /*
   * MSP去初始化后处理
   * 可在恢复时重新配置, 或在彻底关闭前保存校准数据
   */
  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*
 * ============================================================
 * 用户自定义 ADC 函数区
 * 可在此添加: 均值滤波、电压换算、电池电量估算等
 * ============================================================
 */
/* USER CODE END 1 */
