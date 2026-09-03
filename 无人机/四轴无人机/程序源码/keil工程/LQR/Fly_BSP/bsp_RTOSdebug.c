/**
 * @file    bsp_RTOSdebug.c
 * @brief   FreeRTOS 任务运行时调试工具
 * @note    使用 TIM6 的 CNT 计数器测量任务执行频率和耗时
 *          TIM6 配置频率: 100kHz（每 0.01ms 计数 +1）
 *          提供: 频率测量（UpdateFreq）、运行时间测量（UpdateUsedTime）
 */

#include "bsp_RTOSdebug.h"
#include "tim.h"
#include "math.h"

/* 定时器计数频率（Hz）—— 根据 STM32CubeMX 中 TIM6 的配置修改 */
#define TIM_FREQ 100000                    /* 100kHz = 每 0.01ms 递增 1 */

/**
 * @brief  获取 TIM6 当前计数值
 * @retval TIM6->CNT 的当前值（16 位）
 */
static uint16_t get_TickCount(void)
{
	/* 直接读取 TIM6 计数器寄存器 */
	return TIM6->CNT;
}

/**
 * @brief  记录调试计时起点
 * @param  priv_var: 调试私有变量结构体指针
 * @note   保存当前 CNT 值到 TickLast，供后续频率/耗时计算使用
 */
static void get_StartCount(RtosDebugPrivateVar* priv_var)
{
	priv_var->TickLast = get_TickCount();
}

/**
 * @brief  获取任务调用频率（交替采样法）
 * @param  priv_var: 调试私有变量结构体指针
 * @retval 任务频率（Hz）
 * @note   每两次调用计算一次频率（交替模式）
 *          第一次调用记录计数值，第二次调用计算频率
 *          处理了定时器 16 位溢出（最大 65535）的情况
 */
static uint16_t get_Freq(RtosDebugPrivateVar* priv_var)
{
	priv_var->countState = !priv_var->countState;

	if (1 == priv_var->countState)
	{
		/* 第一次进入：保存当前计数值 */
		priv_var->TickLast = get_TickCount();
		priv_var->TaskFreq = priv_var->LastFreq; /* 返回上次计算的频率 */
	}
	else
	{
		/* 第二次进入：计算频率 */
		priv_var->TickNow = get_TickCount();
		if (priv_var->TickNow < priv_var->TickLast)
		{
			/* 16 位计数器溢出了，补偿 65536 */
			priv_var->TaskFreq = priv_var->TickNow + 0xFFFF - priv_var->TickLast;
		}
		else
		{
			priv_var->TaskFreq = priv_var->TickNow - priv_var->TickLast;
		}

		/* 频率 = 定时器频率 / 计数值差（四舍五入取整） */
		priv_var->TaskFreq = round((float)TIM_FREQ / (float)priv_var->TaskFreq);
		priv_var->LastFreq = priv_var->TaskFreq;
	}

	return priv_var->TaskFreq;
}

/**
 * @brief  获取函数/任务的实际运行耗时
 * @param  priv_var: 调试私有变量结构体指针（需先调用 TickStart 记录起点）
 * @retval 运行耗时（单位: ms）
 * @note   计算从 TickStart 到当前时刻的间隔
 *          处理了定时器 16 位溢出情况
 */
static float get_UsedTime(RtosDebugPrivateVar* priv_var)
{
	/* 获取当前计数值 */
	priv_var->TickNow = get_TickCount();

	/* 处理定时器溢出（上次值 > 本次值说明溢出） */
	if (priv_var->TickLast > priv_var->TickNow)
	{
		priv_var->UseTime = priv_var->TickNow + 0xFFFF - priv_var->TickLast;
	}
	else
	{
		priv_var->UseTime = priv_var->TickNow - priv_var->TickLast;
	}

	/* 转换为毫秒: (计数值差 / 定时器频率) * 1000 */
	priv_var->UseTime = ((float)priv_var->UseTime / (float)TIM_FREQ) * 1000;

	return priv_var->UseTime;
}

/* 调试接口结构体实例 ---------------------------------------------------------*/
/* 通过函数指针向各任务提供统一的调试工具接口 */

RtosDebugInterface_t RTOSTaskDebug = {
	.TickStart      = get_StartCount,   /* 记录调试计时起点 */
	.UpdateFreq     = get_Freq,         /* 获取任务调用频率 */
	.UpdateUsedTime = get_UsedTime,     /* 获取函数运行耗时 */
};
