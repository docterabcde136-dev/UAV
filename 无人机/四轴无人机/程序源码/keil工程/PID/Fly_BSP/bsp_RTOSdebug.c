/**
 * ============================================================
 * bsp_RTOSdebug.c — RTOS调试工具集
 * ============================================================
 * 基于 TIM6 硬件定时器 (100kHz) 提供:
 *   1. 任务运行频率测量 (get_Freq) — 每两次调用测一次
 *   2. 函数执行时间测量 (get_UsedTime) — 单位ms
 *   3. 自动处理16位定时器溢出回绕
 *
 * 用法:
 *   RtosDebugInterface_t* debug = &RTOSTaskDebug;
 *   debug->TickStart(&priv);        // 开始计时
 *   freq = debug->UpdateFreq(&priv); // 获取频率 (Hz)
 *   time = debug->UpdateUsedTime(&priv); // 获取耗时 (ms)
 * ============================================================
 */

#include "bsp_RTOSdebug.h"
#include "tim.h"
#include "math.h"

//定时器频率,单位为Hz.根据配置不同修改
#define TIM_FREQ 100000

static uint16_t get_TickCount(void)
{
    //获取计数值的方法.注意该数值跟定时器配置有关,当前配置为 0.01ms 计数1
    return TIM6->CNT;
}

//获取debug计时的方法
static void get_StartCount(RtosDebugPrivateVar* priv_var)
{
    //0替换为读取计数的方法
    priv_var->TickLast = get_TickCount();
}

//获取频率的方法,注意：频率过慢无法检测,这里只处理了1次定时器数据溢出情况
static uint16_t get_Freq(RtosDebugPrivateVar* priv_var)
{
    priv_var->countState = !priv_var->countState;

    if( 1 == priv_var->countState )
        priv_var->TickLast = get_TickCount(),//保存1次计数
        priv_var->TaskFreq = priv_var->LastFreq;
    else
    {
        //第二次进入,开始计算频率
        priv_var->TickNow = get_TickCount();
        if( priv_var->TickNow < priv_var->TickLast )
        {   //数值溢出了
            priv_var->TaskFreq = priv_var->TickNow + 0xFFFF - priv_var->TickLast;
        }
        else
            priv_var->TaskFreq = priv_var->TickNow - priv_var->TickLast;

        priv_var->TaskFreq = round((float)TIM_FREQ/(float)priv_var->TaskFreq);
        priv_var->LastFreq = priv_var->TaskFreq;
    }

    return priv_var->TaskFreq;
}

//获取函数运行时间的方法,注意需要计时开始端使用 TickStart方法保存1次计数值
static float get_UsedTime(RtosDebugPrivateVar* priv_var)
{
    //获取计数
    priv_var->TickNow = get_TickCount();

    //上一次数据比本次获取的值大,说明计数溢出了
    if( priv_var->TickLast > priv_var->TickNow )  
    {
        priv_var->UseTime = priv_var->TickNow + 0xFFFF - priv_var->TickLast;
    }
    else
        priv_var->UseTime = priv_var->TickNow - priv_var->TickLast;
    
    priv_var->UseTime = ((float)priv_var->UseTime/(float)TIM_FREQ)*1000;//转换为ms单位输出

    return priv_var->UseTime;
}

//驱动挂载
RtosDebugInterface_t RTOSTaskDebug = {
    .TickStart = get_StartCount,
    .UpdateFreq = get_Freq,
    .UpdateUsedTime = get_UsedTime,
};

