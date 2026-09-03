#include "stm32f10x.h"                  // Device header
#include "Variable.h"
#include "PWM.h"


void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);
}

void LED_Light(uint8_t LED_bit)
{
	if(LED_bit) GPIO_SetBits(GPIOA,GPIO_Pin_7);
	else GPIO_ResetBits(GPIOA,GPIO_Pin_7);
}

	static uint16_t LED_Time_Light=0;
	static uint16_t LED_Time_Dark=0;
	static uint8_t LED_Light_Dark_Bit=0;//1为亮，0为暗
void LED_Time(uint16_t LED_Time_number)
{

	
	if(LED_Light_Dark_Bit==1)//执行亮
	{
		LED_Light(1);
		LED_Time_Light++;  //记录亮的时间，必须与LED_Time值一致才能保证亮的时间与暗的时间一致
		if(LED_Time_Light==LED_Time_number) 
		{
			LED_Time_Light=0;//清零
			LED_Light_Dark_Bit=0;
		}
	}
	else if(LED_Light_Dark_Bit==0)//执行暗
	{
		LED_Light(0);
	  LED_Time_Dark++;
		if(LED_Time_Dark==LED_Time_number)
		{
			LED_Time_Dark=0;//清零
		  LED_Light_Dark_Bit=1;
		}
	}
}
