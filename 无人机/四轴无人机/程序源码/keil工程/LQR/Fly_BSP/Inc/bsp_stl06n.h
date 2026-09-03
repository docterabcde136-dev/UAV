#ifndef __BSP_STL06N_H
#define __BSP_STL06N_H

#include <stdint.h>

//角度分辨率、雷达数据长度
#define Stl06n_AngleRatio 72 //单位0.01度
#define Stl06n_LidarDistanceBufferLen 500 // 360/分辨率

//1个雷达点云信息
#pragma pack(1)
typedef struct {
	uint16_t distance;
	uint8_t peak;
} Stl06NPointStructDef;
#pragma pack()

//1帧雷达数据
#pragma pack(1)
typedef struct {
	uint8_t header;
	uint8_t ver_len;
	uint16_t speed;
	uint16_t start_angle;
	Stl06NPointStructDef point[12];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
}Stl06NFrameTypeDef;
#pragma pack()

//STL06N雷达DMA接收长度配置
#define userconfig_STL06N_DMABUF_LEN 60

//N10 sensor的原始数据
typedef struct {
	uint8_t Buf[userconfig_STL06N_DMABUF_LEN];
	uint16_t DataLen;
}OriData_STL06N_t;

#pragma pack(1)
typedef struct {
	Stl06NPointStructDef buffer[Stl06n_LidarDistanceBufferLen];
} Stl06NAngleBuffer_t;
#pragma pack()

void Stl06NLidar_Start(void);
void Stl06NLidar_Stop(void);
void Stl06NLidar_Toggle(void);

#endif /* __BSP_STL06N_H */
