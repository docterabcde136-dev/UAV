#include "show_task.h"

//C Lib
#include <stdio.h>
#include <string.h>
#include <math.h>

//RTOS
#include "FreeRTOS.h"
#include "task.h"

//HAL
#include "usart.h"

//bsp
#include "bsp_datascope.h"
#include "bsp_adc.h"
#include "bsp_stp23L.h"
#include "bsp_N10.h"
#include "bsp_datascope.h"

//app
#include "balance_task.h"

static char page1_paramshow[100];   //APP首页数据
static char page2_paramshow[1024];  //APP波形页面数据
static char page3_paramshow[1024];  //APP参数页面数据

#define userconfig_USE_WHEELTECAPP 1 //选择显示的工具. 1:使用APP显示数据  2:使用minibalance上位机显示数据

APPShowType_t appshow = { 0 }; //显示内容存储变量

extern uint8_t g_lost_pos_dev;

void ShowTask(void* param)
{
	pADCInterface_t adc1 = &UserADC1;
	
	//蓝牙对应串口3
	UART_HandleTypeDef* serial = &huart4; 

	//实现APP数据分时显示
	uint8_t appshowNum = 0;
	char* NeedShowBuf = NULL;

	float VOL_percen = 0;//电量百分比
	
	float cur_all = 0;
	
	while (1)
	{
		//电调总电流监测
		cur_all = (float)adc1->getValue(userconfigADC_Curr_CHANNEL)/4096.0f * 3.3f / 0.2f;
		
		//电量计算
		VOL_percen = (g_robotVOL-10.0f) / (12.6f - 10.0f) * 100;
		if( VOL_percen < 0 ) VOL_percen = 0;

	#if 1 == userconfig_USE_WHEELTECAPP
		//APP显示数据使用Demo.不可与上位机同时使用
		appshowNum++; //分时显示
		if( 1 == appshowNum )
		{
			sprintf(page1_paramshow,"{A%d:%d:%d:%d}$",(int)appshow.m3,(int)appshow.m1,(int)VOL_percen,(int)(appshow.yaw*57.3f)); //左码盘,右码盘,电量百分比,角度
			NeedShowBuf = page1_paramshow;
		}
		else if( 2 == appshowNum )
		{
			sprintf(page2_paramshow,"{B%d:%d:%d}$",(int)(appshow.roll*57.3f),(int)(appshow.pitch*57.3f),(int)(appshow.yaw*57.3f)); //波形显示
			NeedShowBuf = page2_paramshow;
		}
		else if( 3== appshowNum )
		{	
			appshowNum = 0;

			/* 参数页面显示 */
			sprintf(page3_paramshow,"{C%d:%d:%d}$",(int)(cur_all*1000),(int)(g_robotVOL*1000),(int)g_lost_pos_dev); 

			NeedShowBuf = page3_paramshow;
		}
		//发送数据到APP
		HAL_UART_Transmit_DMA(serial,(uint8_t*)NeedShowBuf,strlen(NeedShowBuf));

	#else
		//上位机使用Demo,不可与APP同时使用上位机请配合F570蓝牙透传软件使用,无需接线
		pDataScopeInterface_t datascope = &UserDataScope;
		float showtmp[10]={0}; //上位机最多显示10组波形
//		showtmp[0] = appshow.pitch;
//		showtmp[1] = appshow.roll;
//		showtmp[2] = appshow.yaw;
//		showtmp[3] = appshow.gyrox;
//		showtmp[4] = appshow.gyroy;
//		showtmp[5] = appshow.gyroz;
		showtmp[5] = appshow.m1*57.29f;
		showtmp[6] = appshow.m2*57.29f;
		showtmp[7] = appshow.posx;
		showtmp[8] = appshow.posy;
		showtmp[9] = appshow.balanceTaskFreq;
		datascope->show(showtmp,sizeof(showtmp)/sizeof(showtmp[0]));
		
//		#include "pid.h"
//		extern PIDControllerType_t RollRatePID,RollPID;
//		extern PIDControllerType_t PitchRatePID,PitchPID;
//		extern PIDControllerType_t YawRatePID,YawPID;
//		printf("rollratekp:%.2f\r\n",RollRatePID.kp);
//		printf("rollrateki:%.2f\r\n",RollRatePID.ki);
//		printf("rollratekd:%.2f\r\n",RollRatePID.kd);
//		printf("m1:%.3f\r\n",appshow.m1);
//		printf("m2:%.3f\r\n",appshow.m2);
//		printf("m3:%.3f\r\n",appshow.m3);
//		printf("m4:%.3f\r\n\r\n",appshow.m4);
		
	#endif

		vTaskDelay(50);
	}
  
}
