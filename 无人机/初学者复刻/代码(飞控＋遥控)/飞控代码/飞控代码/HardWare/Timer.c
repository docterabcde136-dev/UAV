#include "stm32f10x.h"                  // Device header


void Timer_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	//内部时钟
	TIM_InternalClockConfig(TIM1);
	//配置时机单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=1000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	//中断输出
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	//中断分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//中断配置
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;       // TIM1中断
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;          // 使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;// 抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;       // 子优先级1
	NVIC_Init(&NVIC_InitStructure);
	//开启中断
	TIM_Cmd(TIM1,ENABLE);

}



//void Timer_Init(void)
//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
//	//内部时钟
//	TIM_InternalClockConfig(TIM1);
//	TIM_InternalClockConfig(TIM3);
//	//配置时机单元
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
//	TIM_TimeBaseInitStructure.TIM_Period=1000-1;
//	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;
//	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
//	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
//	//中断输出
//	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
//	//中断分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	//中断配置
//	//==================== 先配置TIM3（最高优先级） ====================
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;          // TIM3中断
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;          // 使能
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;// 抢占优先级0（最高）
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;       // 子优先级0
//	NVIC_Init(&NVIC_InitStructure);
//	
//	//==================== 再配置TIM1（次优先级） ====================
//	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;       // TIM1中断
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;          // 使能
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;// 抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;       // 子优先级1
//	NVIC_Init(&NVIC_InitStructure);
//	//开启中断
//	TIM_Cmd(TIM1,ENABLE);
//	TIM_Cmd(TIM3,ENABLE);
//}





