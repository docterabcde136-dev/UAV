/*头文件引用*/
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "NRF24L01.h"
#include "Rocker_ADC.h"
#include "Timer.h"
#include "Data_Proc.h"
#include "Variable.h"
#include "Key.h"
#include "LED.h"
#include "PowerDetection.h"
#include "Encoder.h"

/*主函数*/
int main(void)
{
	OLED_Init();			//OLED初始化
	NRF24L01_Init();	//无线模块初始化
	Key_Init();       //按键初始化
	Rocker_ADC_Init();//遥感初始化
	Timer_Init();			//定时器初始化
	LED_Init();       //LED初始化
	Encoder_Init();
	

	
	while(1)
	{
		KeyNum();   //按键值处理
		if(LED_Slow_time==0)  {LED_Control();  LED_Slow_time=1;}//LED显示
		if(Proc_Slow_time==0) {Data_Proc();   Proc_Slow_time=1;}//变量处理函数
		if(Disp_Slow_time==0) {Data_Display();Disp_Slow_time=1;}//数据显示函数
		if(Send_Slow_time==0) {Data_Send();   Send_Slow_time=1;}//发送消息
		//if(Receive_Slow_time==0)Data_Receive();
	}
}

/*定时中断，每1ms进一次中断*/ 
void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
	{
		/*每50ms发送,数据发送给飞控*/
  	if(++Send_Slow_time>4) Send_Slow_time=0;
		if(++Proc_Slow_time>10)  Proc_Slow_time=0;
		if(++Disp_Slow_time>15) Disp_Slow_time=0;
		if(++LED_Slow_time>30) LED_Slow_time=0;
		
		if(++connect_time>500) 
		{	
			connect_time=0;
			connect_value++;
			if(connect_value>200) connect_value=0;
		}
		
		
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}
