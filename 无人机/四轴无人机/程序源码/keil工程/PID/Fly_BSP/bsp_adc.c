/**
 * ============================================================
 * bsp_adc.c — ADC驱动 (电池电压+电流采样)
 * ============================================================
 * 功能:
 *   - ADC1 DMA双通道连续采样 (CH7电池电压, CH9电流)
 *   - 滑动窗口均值滤波
 * 接口: ADCInterface_t { getValue }
 * 实例: UserADC1
 * ============================================================
 */

#include "bsp_adc.h"
#include "adc.h"

float g_robotVOL = 12.0f;

#define userconfig_ADCDMA_BUF_LEN 80 //设置dma搬运某个通道的个数

static uint16_t g_Adc1Buf[userconfig_ADCDMA_BUF_LEN][2] = { 0 };//存放ADC数据的数组

void ADC_Userconfig_Init(void)
{
	//启动ADC采集并设置DMA传输
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)g_Adc1Buf,sizeof(g_Adc1Buf)/sizeof(g_Adc1Buf[0][0]));
}

//获取DMA搬运后的ADC值
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel)
{
	uint32_t tmp = 0;
	
	//防止索引溢出
	if( channel > 2 ) return 0;
	
	for(uint8_t i=0;i<userconfig_ADCDMA_BUF_LEN;i++)
	{
		tmp += g_Adc1Buf[i][channel];
	}
	
	return tmp/userconfig_ADCDMA_BUF_LEN;
}

ADCInterface_t UserADC1 = {
	.init = ADC_Userconfig_Init,
	.getValue = USER_ADC_Get_AdcBufValue,
};

