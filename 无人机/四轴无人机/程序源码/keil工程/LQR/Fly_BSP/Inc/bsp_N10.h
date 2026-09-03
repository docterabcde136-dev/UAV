#ifndef __BSP_N10_H
#define __BSP_N10_H

#include <stdint.h>

//N10雷达 DMA数据缓冲区内存分配
#define userconfig_N10_DMABUF_LEN 70

//N10 sensor的原始数据
typedef struct {
	uint8_t Buf[userconfig_N10_DMABUF_LEN];
	uint16_t DataLen;
}OriData_N10_t;

//N10雷达数据处理函数
void N10_DataHandle(uint8_t* oribuf);

#endif /* __BSP_N10_H */


