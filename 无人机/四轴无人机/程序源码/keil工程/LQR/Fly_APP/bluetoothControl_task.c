#include <math.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "balance_task.h"

#include "main.h"

enum{
	AppKeyNone = 0,
	AppKeyFront = 1,
	AppKeyFR=2,
	AppKeyRight=3,
	AppKeyBR=4,
	AppKeyBack=5,
	AppKeyBL = 6,
	AppKeyLeft=7,
	AppKeyFL = 8,
	APPKeyUp = 0x18,
	APPKeyDown = 0x19,
};

__weak void SendToAPPFlag(void)
{
	
}

__weak void SaveFlashFlag(void)
{
	
}

//四轴事件
extern void StopFlyAction(void);
extern void StartFlyAction(void);
extern void FlyAction_EnterUNUSEHeightMode(void);
extern void FlyAction_ChangeMotorDir(void);

void BluetoothAPPControl_task(void* param)
{
	extern QueueHandle_t g_xQueueBlueTooth_Ori;
	extern void WriteFlyControlQueue(FlyControlType_t val,uint8_t writeEnv,BaseType_t* HigherPriorityTask);
	
	uint8_t recv = 0;     //用于接收队列数据
	uint8_t app_page = 1; //手机APP控制页的页数. 0:重力页面 1:摇杆页面 2:按键页面
	uint8_t controlKey = 0; //控制值
	
	//APP调参页面辅助参数
	static uint8_t paramFlag=0,param_i=0,param_j=0,paramReceive[50]={0};
	float paramData=0;
	
	while( 1 )
	{
		FlyControlType_t controlVal = { 0 };//定义控制量
		
		if( pdPASS == xQueueReceive(g_xQueueBlueTooth_Ori,&recv,portMAX_DELAY) ) 
		{	
			#if 1 //WHEELTEC APP
			controlVal.source = APPCmd; //标记控制指令来源
			uint8_t writeFlag = 1;      //标记是否要写入控制队列
			
			/* APP按键页面切换 */
			if( recv == 'K' ) app_page = 2;      //按键页面
			else if( recv == 'J' ) app_page = 1; //摇杆页面
			else if( recv == 'I' ) app_page = 0; //重力球页面
			
			//读取控制信息
			controlKey = recv-0x40;
			
			//不接受重力球页的信息,难以控制
			if( app_page==0 ) controlKey = 0;
			
			//遥控幅度
			const float control_step = 5.0f;
			switch( controlKey )
			{
				case AppKeyFront: controlVal.roll = angle_to_rad(control_step);
					break;
				case AppKeyFR:    controlVal.roll = angle_to_rad(control_step);controlVal.pitch = -angle_to_rad(control_step);
					break;
				case AppKeyRight: 
					if( 2 == app_page )
					{
						controlVal.gyroz = angle_to_rad(60.0f);
					}
					else
						controlVal.pitch = -angle_to_rad(control_step);
					break;
				case AppKeyBR:   controlVal.roll = -angle_to_rad(control_step);controlVal.pitch = -angle_to_rad(control_step);
					break;
				case AppKeyBack: controlVal.roll = -angle_to_rad(control_step);
					break;
				case AppKeyBL:  controlVal.roll = -angle_to_rad(control_step);controlVal.pitch = angle_to_rad(control_step);
					break;
				case AppKeyLeft:
					if( 2==app_page )
					{
						controlVal.gyroz = -angle_to_rad(60.0f);
					}
					else
						controlVal.pitch = angle_to_rad(control_step);
					break;
				case AppKeyFL:  controlVal.roll = angle_to_rad(control_step);controlVal.pitch = angle_to_rad(control_step);
					break;
				case APPKeyUp: controlVal.height = 0.01f;
					break;
				case APPKeyDown: controlVal.height = -0.01f;
					break;
				default : writeFlag = 0;
					break;
			}
			
			//检查是否有控制,有控制则写入队列
			if( 1 == writeFlag ) WriteFlyControlQueue(controlVal,0,NULL);
			
			//APP参数页面 数据格式: {?:?}
			if(recv==0x7B) paramFlag=1;        //The start bit of the APP parameter instruction //APP参数指令起始位
			else if(recv==0x7D) paramFlag=2;   //The APP parameter instruction stops the bit    //APP参数指令停止位
			
			if(paramFlag==1) //Collect data //采集数据
			{
				paramReceive[(param_i%50)]=recv;
				param_i++;
			}
			else if(paramFlag==2) //Analyze the data //分析数据
			{
				if(paramReceive[3]==0x50) 	 SendToAPPFlag();      // {Q:P} 获取设备参数
				else if( paramReceive[3]==0x57 ) SaveFlashFlag(); // {Q:W} 设置掉电保存参数
				
				else if( paramReceive[1]=='M' && paramReceive[3]=='1' ) StartFlyAction(); //自定义协议：{M:1}
				else if( paramReceive[1]=='M' && paramReceive[3]=='2' ) StopFlyAction(); //自定义协议：{M:2}	
				else if( paramReceive[1]=='M' && paramReceive[3]=='3' ) FlyAction_EnterUNUSEHeightMode(); //自定义协议：{M:3}	
				
				else  if(paramReceive[1]!=0x23)                   // {0:xxx} {1:xxx} {2:xxx} 单通道数值设置
				{
					for(param_j=param_i; param_j>=4; param_j--)
					{
						paramData+=(paramReceive[param_j-1]-48)*pow(10,param_i-param_j);
					}
					switch(paramReceive[1])
					{
						case 0x30: break;
						case 0x31: break;
						case 0x32: break;
						case 0x33: break;
						case 0x34: break;
						case 0x35: break;
						case 0x36: break;
						case 0x37: break;
						case 0x38: break;
					}
				}
				else if( paramReceive[1]==0x23 ) //APP上点击“发送所有数据”处理方法  // {#xxx:xxx:xxx...xxx}
				{
					float num=0;
					uint8_t dataIndex=0;
					float dataArray[9]={0};

					if( param_i<=50 ) //数据在可接受范围
					{
						paramReceive[param_i]='}'; //补充帧尾

						for(uint8_t kk=0; paramReceive[kk]!='}'; kk++)
						{
							if( paramReceive[kk]>='0' && paramReceive[kk]<='9' )
							{
								num = num*10 + ( paramReceive[kk] - '0' );
							}
							else if( paramReceive[kk]==':' )
							{
								dataArray[dataIndex++] = num;
								num = 0;
							}

						}
						//处理最后一个数据
						dataArray[dataIndex] = num;
						
						//数据获取案例,例如 xxx = dataArray[0];
						UNUSED(dataArray[0]);
					}
				}

				//Relevant flag position is cleared
				//相关标志位清零
				paramFlag=0;param_i=0;param_j=0;paramData=0;
				memset(paramReceive, 0, sizeof(uint8_t)*50); //Clear the array to zero//数组清零
			}
		#endif //WHEELTEC APP
		
		}
	}
}
