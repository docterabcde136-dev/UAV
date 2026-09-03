#ifndef __POWER_DETECTION_H_
#define __POWER_DETECTION_H_

void AD_Init(void);
uint16_t Get_ADC(void);
void GetBattery_Init(void);
float GetBattery(void);

void GetCur_Power(void);


#endif
