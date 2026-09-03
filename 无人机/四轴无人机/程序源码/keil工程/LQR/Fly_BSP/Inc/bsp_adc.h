/**
 * @file    bsp_adc.h
 * @brief   板级 ADC 采样驱动接口定义（电池电压 + 电流检测）
 */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>

/* ADC 通道编号 */
#define userconfigADC_VBAT_CHANNEL  0   /* 电池电压检测通道 (PA7) */
#define userconfigADC_Curr_CHANNEL  1   /* 电流检测通道 (PB1) */

/* ADC 接口结构体 */
typedef struct {
	void     (*init)(void);                 /* ADC DMA 采样初始化 */
	uint16_t (*getValue)(uint8_t channel);  /* 获取指定通道的 ADC 均值 */
} ADCInterface_t, *pADCInterface_t;

extern ADCInterface_t UserADC1;             /* ADC1 接口实例 */

/* 公开接口函数 */
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel);
void     ADC_Userconfig_Init(void);
extern float g_robotVOL;                    /* 电池电压值 (V) */

#endif /* __BSP_ADC_H */
