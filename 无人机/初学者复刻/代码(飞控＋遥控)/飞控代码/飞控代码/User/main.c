#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "NRF24L01.h"
#include "Motor.h"
#include "Timer.h"
#include "Variable.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "Data_Proc.h"
#include "PowerDetection.h"
#include "LED.h"

/*传送门*/
//CHUANSHONGMEN
int main(void)
{
  Timer_Init();       //定时器初始化
  OLED_Init();        //OLED初始化
  NRF24L01_Init();    //无线模块初始化
	Motor_Init();       //电机初始化
	MPU6050_Init();     //陀螺仪初始化
  MPU6050_DMP_Init(); //DMP库初始化
	GetBattery_Init();  //电量检测初始化
	LED_Init();         //状态LED初始化

	while(1)
	{
		/*获取姿态信息*/
		if(Attitude_Proc_time==SlowDown) {Attitude_Data_Proc();Attitude_Proc_time=1;}
	  /*接收信息*/
		if(Receive_Slow_time==SlowDown)  {Data_Receive(); Receive_Slow_time=1;}     
		/*数据处理*/
		if(Proc_Slow_time==SlowDown)     {Data_Proc(); Proc_Slow_time=1;}         
		/*数据显示*/
		if(Show_Slow_time==SlowDown)     {Data_Display(); Show_Slow_time=1;}       
		/*获取平均电量*/	
		if(PowerDete_Slow_time==SlowDown){GetCur_Power(); PowerDete_Slow_time=1;}   

	}
}

void TIM1_UP_IRQHandler(void) /*1ms*/
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
	{
		/*每6ms接收遥控数据*/
		if(++Receive_Slow_time>6)  Receive_Slow_time=0;
		/*每20ms处理接收数据*/
		if(++Proc_Slow_time>20)     Proc_Slow_time=0;
		/*每50ms显示数据*/
		if(++Show_Slow_time>50)    Show_Slow_time=0;
		/*每4ms解析姿态数据*/
		if(++Attitude_Proc_time>4)  Attitude_Proc_time=0;
		/*每3ms处理飞控算法*/
		if(++Task_Slow_time>3)      {Task_Slow_time=0; Fly_task();}
		/*每10ms电量检测*/
		if(++PowerDete_Slow_time>10)PowerDete_Slow_time=0;
		/*每0ms更新LED数据*/
		LED_Detect();
		/*每1s进行持续连接检测：*/
		//将当前获取到的connect_value与上一次的值作比较，
		//如果在一段时间之内，这个值有变动，说明正常，
		//如果恒定不变说明未能保持连接自动赋予油门值为0
		//每过1000ms，进行赋值，然后下一次1000ms内进行比较
		if(++connect_time>=1000)	{connect_time=0;Connect_Portect_Test();}

		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}
