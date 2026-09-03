#include "stm32f10x.h"                  // Device header

//遥感按键初始化
void Rocker_Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

uint8_t Get_Rocker_Key_NUM(void)
{
	return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
}
