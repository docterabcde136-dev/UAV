/**
 * @file    bsp_adc.c
 * @brief   板级 ADC 采样驱动实现
 * @note    使用 ADC1 + DMA 双通道连续采样
 *          Channel 0: 电池电压检测 (Battery_Ch, PA7)
 *          Channel 1: 电流检测通道 (cur_ch, PB1)
 *          DMA 缓冲区长度可配置 (userconfig_ADCDMA_BUF_LEN)
 */

#include "bsp_adc.h"
#include "adc.h"

/* 全局电池电压值（单位：V）----------------------------------------------------*/
float g_robotVOL = 12.0f;             /* 电池电压，初始默认 12V */

/* ADC DMA 缓冲区配置 ---------------------------------------------------------*/
#define userconfig_ADCDMA_BUF_LEN 80   /* DMA 缓存深度：每个通道采集 80 个样本用于均值滤波 */

static uint16_t g_Adc1Buf[userconfig_ADCDMA_BUF_LEN][2] = { 0 }; /* ADC DMA 双通道原始数据缓存 */

/**
 * @brief  ADC 用户配置初始化
 * @note   启动 ADC1 的 DMA 连续采样模式，数据自动填充到 g_Adc1Buf
 *         通道 0: 电池电压, 通道 1: 电流检测
 */
void ADC_Userconfig_Init(void)
{
    /* 启动 ADC1 DMA 采样，数据持续写入双通道缓存数组 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_Adc1Buf,
                      sizeof(g_Adc1Buf) / sizeof(g_Adc1Buf[0][0]));
}

/**
 * @brief  获取指定 ADC 通道的 DMA 采样平均值
 * @param  channel: ADC 通道号（0=电池电压, 1=电流检测）
 * @retval 该通道 80 次采样的算术平均值（去噪滤波）
 * @note   若 channel > 2 则返回 0（无效通道保护）
 */
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel)
{
    uint32_t tmp = 0;

    /* 通道号范围保护 */
    if (channel > 2) return 0;

    /* 对缓冲区中该通道的所有样本求和 */
    for (uint8_t i = 0; i < userconfig_ADCDMA_BUF_LEN; i++)
    {
        tmp += g_Adc1Buf[i][channel];
    }

    /* 返回算术平均值 */
    return tmp / userconfig_ADCDMA_BUF_LEN;
}

/* ADC 接口结构体实例 ---------------------------------------------------------*/
/* 通过函数指针向上层 APP 暴露统一的 ADC 操作接口 */

ADCInterface_t UserADC1 = {
    .init     = ADC_Userconfig_Init,       /* ADC DMA 采样初始化 */
    .getValue = USER_ADC_Get_AdcBufValue,  /* 获取指定通道采样均值 */
};
