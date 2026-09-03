#ifndef __BSP_STP23L_H
#define __BSP_STP23L_H

#include <stdint.h>

//stp23L传感器 DMA数据缓冲区分配
#define userconfig_STP23L_DMABUF_LEN 200

//stp23L sensor的原始数据
typedef struct {
	uint8_t Buf[userconfig_STP23L_DMABUF_LEN];
	uint16_t DataLen;
}OriData_STP23L_t;

//STP23L数据处理函数、
extern float g_readonly_distance;
void stp23L_getdistance_callback(uint8_t recv,float* dis);

#endif /* __BSP_STP23L_H */


