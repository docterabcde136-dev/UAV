#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>

//ADC存放在缓冲区的索引值
#define userconfigADC_VBAT_CHANNEL  0
#define userconfigADC_Curr_CHANNEL  1

typedef struct {
	void (*init)(void);
    uint16_t (*getValue)(uint8_t channel);
}ADCInterface_t,*pADCInterface_t;

extern ADCInterface_t UserADC1;

//对外接口
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel);
void ADC_Userconfig_Init(void);
extern float g_robotVOL;

#endif /* __BSP_ADC_H */

