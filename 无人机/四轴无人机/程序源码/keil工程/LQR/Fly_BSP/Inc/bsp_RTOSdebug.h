#ifndef __BSP_RTOSDEBUG_H
#define __BSP_RTOSDEBUG_H

#include <stdint.h>

/*

使用说明：
1.使用该功能前，请提供变量 RtosDebugPrivateVar task1debugVar; //变量名称随意
2.使用接口： pRtosDebugInterface_t task1debug = &RTOSTaskDebug;
3.在某个任务计算任务频率： task1freq = task1debug->UpdateFreq(&task1debugVar); //获取的返回值即该任务的频率

*/

typedef struct {
    uint16_t TickNow;
    uint16_t TickLast;
    float UseTime;
    uint16_t TaskFreq;
    uint16_t LastFreq;
    uint8_t countState;
}RtosDebugPrivateVar;

typedef struct {
    void (*TickStart)(RtosDebugPrivateVar* priv_var);
    uint16_t (*UpdateFreq)(RtosDebugPrivateVar* priv_var);
    float (*UpdateUsedTime)(RtosDebugPrivateVar* priv_var);
}RtosDebugInterface_t,*pRtosDebugInterface_t;

extern RtosDebugInterface_t RTOSTaskDebug;

#endif /* __BSP_RTOSDEBUG_H */

