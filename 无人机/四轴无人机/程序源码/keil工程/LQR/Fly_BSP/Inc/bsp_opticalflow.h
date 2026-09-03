#ifndef __BSP_OPTICALFLOW_H
#define __BSP_OPTICALFLOW_H

#include <stdint.h>

typedef struct{
	short x;
	short y;
}PositionType_t;

typedef struct{
	void (*StartRecvData)(void);
	PositionType_t (*getpos)(void);
	PositionType_t (*getpos_dot)(void);
	void (*HandleRecvData)(uint16_t size);
	void (*SetZeroPoint)(void);
}OpticalFlowInterface_t,*pOpticalFlowInterface_t;

extern OpticalFlowInterface_t  UserOpticalFlow;

#endif

