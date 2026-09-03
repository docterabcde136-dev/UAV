#include "stm32f10x.h"                  // Device header
#include "Variable.h"
#include "OLED.h"
#include "NRF24L01.h"
#include "PowerDetection.h"
#include "Encoder.h"

//遥感定义
//       正
//       |
//  负---+---正
//       |
//       负
//
/*数据处理*/
void Data_Proc(void)
{
	/*------数据处理------*/
	//获取遥感数据
	LX=(uint8_t)(Rocker_Value[0]*(100.0/4095));
	LY=(uint8_t)(Rocker_Value[1]*(100.0/4095));
	RX=(uint8_t)(Rocker_Value[2]*(100.0/4095));
	RY=(uint8_t)(Rocker_Value[3]*(100.0/4095));
	//获取当前手柄电量
	Battery_Value=(3.3*2*Rocker_Value[4]/4096);

	//获得编码器数据
	Encoder_pulse=Encoder_Get();
	Encoder_number+=Encoder_pulse;
	
	Yaw_Control=(Encoder_number/2)+100;
	if(Yaw_Control>=190) 
	{
		Yaw_Control=190;
		Encoder_number=180;
	}
	if(Yaw_Control<=10)
	{
		Yaw_Control=10;
		Encoder_number=-180;
	}
	
	/*------无线通信信息处理------*/
	//获取当前的PID参数
	
	PID_Data[0][0]=pitch_angle_balance_Kp;
	PID_Data[0][1]=pitch_angle_balance_Ki;
	PID_Data[0][2]=pitch_angle_balance_Kd;
	PID_Data[1][0]=roll_angle_balance_Kp;
	PID_Data[1][1]=roll_angle_balance_Ki;
	PID_Data[1][2]=roll_angle_balance_Kd;
	PID_Data[2][0]=yaw_angle_balance_Kp;
	PID_Data[2][1]=yaw_angle_balance_Ki;
	PID_Data[2][2]=yaw_angle_balance_Kd;
	PID_Data[3][0]=pitch_gyro_balance_Kp;
	PID_Data[3][1]=pitch_gyro_balance_Ki;
	PID_Data[3][2]=pitch_gyro_balance_Kd;
	PID_Data[4][0]=roll_gyro_balance_Kp;
	PID_Data[4][1]=roll_gyro_balance_Ki;
	PID_Data[4][2]=roll_gyro_balance_Kd;
	PID_Data[5][0]=yaw_gyro_balance_Kp;
	PID_Data[5][1]=yaw_gyro_balance_Ki;
	PID_Data[5][2]=yaw_gyro_balance_Kd;

	//循环放置
	if(++index_type>21)index_type=0;
  //依据类型更改
	switch(index_type)
	{
		case 1://俯仰机械零点
			fly_pitch_zero+=100;
			PID_Data_Send[0]=(uint8_t)fly_pitch_zero;
	    PID_Data_Send[1]=(uint16_t)(fly_pitch_zero*100)%100;
			fly_pitch_zero-=100;		
		  break;
		case 2://翻滚机械零点
			fly_roll_zero+=100;
			PID_Data_Send[0]=(uint8_t)fly_roll_zero;
	    PID_Data_Send[1]=(uint16_t)(fly_roll_zero*100)%100;
		  fly_roll_zero-=100;
		  break;
		case 3://航向机械零点重置
			PID_Data_Send[0]=(uint8_t)fly_yaw_zero;
	    PID_Data_Send[1]=(uint16_t)(fly_yaw_zero*100)%100;
		  break;
		//---外环
		case 4://外环俯仰的P  0.112
		  PID_Data_Send[0]=(uint8_t)(PID_Data[0][0]*10);//获取0.1位之前的数
	    PID_Data_Send[1]=(uint16_t)(PID_Data[0][0]*1000)%100;//获取0.1位之后的数
		  break;
		case 5://外环俯仰的I
		  PID_Data_Send[0]=(uint8_t)(PID_Data[0][1]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[0][1]*1000)%100;
		 break;
		case 6://外环俯仰的D
			PID_Data_Send[0]=(uint8_t)(PID_Data[0][2]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[0][2]*1000)%100;	
		  break;
		case 7://外环翻滚的P
		  PID_Data_Send[0]=(uint8_t)(PID_Data[1][0]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[1][0]*1000)%100;
		  break;
		case 8://外环翻滚的I
		  PID_Data_Send[0]=(uint8_t)(PID_Data[1][1]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[1][1]*1000)%100;
		  break;
		case 9://外环翻滚的D
			PID_Data_Send[0]=(uint8_t)(PID_Data[1][2]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[1][2]*1000)%100;	
		  break;
		case 10://外环航向的P
		  PID_Data_Send[0]=(uint8_t)(PID_Data[2][0]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[2][0]*1000)%100;
		  break;
		case 11://外环航向的I
		  PID_Data_Send[0]=(uint8_t)(PID_Data[2][1]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[2][1]*1000)%100;
		  break;
		case 12://外环航向的D
			PID_Data_Send[0]=(uint8_t)(PID_Data[2][2]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[2][2]*1000)%100;	
		  break;				
		
		//---内环
		case 13://内环俯仰的P
		  PID_Data_Send[0]=(uint8_t)(PID_Data[3][0]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[3][0]*1000)%100;
		  break;
		case 14://内环俯仰的I
		  PID_Data_Send[0]=(uint8_t)(PID_Data[3][1]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[3][1]*1000)%100;
		 break;
		case 15://内环俯仰的D
			PID_Data_Send[0]=(uint8_t)(PID_Data[3][2]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[3][2]*1000)%100;	
		  break;
		case 16://内环翻滚的P
		  PID_Data_Send[0]=(uint8_t)(PID_Data[4][0]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[4][0]*1000)%100;
		  break;
		case 17://内环翻滚的I
		  PID_Data_Send[0]=(uint8_t)(PID_Data[4][1]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[4][1]*1000)%100;
		  break;
		case 18://内环翻滚的D
			PID_Data_Send[0]=(uint8_t)(PID_Data[4][2]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[4][2]*1000)%100;	
		  break;
		case 19://内环航向的P
		  PID_Data_Send[0]=(uint8_t)(PID_Data[5][0]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[5][0]*1000)%100;
		  break;
		case 20://内环航向的I
		  PID_Data_Send[0]=(uint8_t)(PID_Data[5][1]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[5][1]*1000)%100;
		  break;
		case 21://内环航向的D
			PID_Data_Send[0]=(uint8_t)(PID_Data[5][2]*10);
	    PID_Data_Send[1]=(uint16_t)(PID_Data[5][2]*1000)%100;	
		  break;	
	}

}

/*数据显示*/
void Data_Display(void)
{
	switch(OLED_Choose_Page)
	{
		case 0:/*主菜单界面*/
				
		  OLED_Clear();
			OLED_ShowString(0,0 ,"CTL",OLED_8X16);
			OLED_ShowString(0,16,"FLY",OLED_8X16);
			OLED_ShowString(0,32,"PID",OLED_8X16);
			OLED_ShowString(0,48,"BAT",OLED_8X16);
		  OLED_ShowString(24,16*(OLED_Choose_Row)-16,"*",OLED_8X16);
	  	OLED_ReverseArea(0,16*(OLED_Choose_Row)-16,32,16);
			if(Lock==1)
			{
				OLED_ShowString(50,0,"LOCK",OLED_8X16);	
				 OLED_Printf(50,16 ,OLED_8X16,"Oil:%d",0);
			}
			else
			{
				OLED_ShowString(50,0,"UNLOCK",OLED_8X16);
				OLED_Printf(50,16 ,OLED_8X16,"Oil:%d",LX);
			}
			OLED_Printf(50,32 ,OLED_8X16,"Enc:L+R-");
			OLED_Printf(50,48 ,OLED_8X16,"Enc:%d",Yaw_Control-100);		
		  OLED_Update();	
		  break;
		
		case 1:/*遥感数据*/
		  OLED_Clear();	
		 	OLED_ShowString(0,0 ,"CTL",OLED_8X16);
			OLED_ShowString(0,16,"FLY",OLED_8X16);
			OLED_ShowString(0,32,"PID",OLED_8X16);
			OLED_ShowString(0,48,"BAT",OLED_8X16);
	  	OLED_ShowString(24,16*(OLED_Choose_Row)-16,"*",OLED_8X16);
		  OLED_ReverseArea(0,16*(OLED_Choose_Row)-16,32,16);
		  OLED_Printf(50,0 ,OLED_8X16,"LY:%d",LX);
		  OLED_Printf(50,16,OLED_8X16,"LX:%d",LY);
		  OLED_Printf(50,32,OLED_8X16,"RX:%d",RY);
		  OLED_Printf(50,48,OLED_8X16,"RY:%d",RX);
		  OLED_Update();
			break;
		
		case 2:/*初始角度*/
			OLED_Clear();
	
		 	OLED_ShowString(0,0 ,"CTL",OLED_8X16);
			OLED_ShowString(0,16,"FLY",OLED_8X16);
			OLED_ShowString(0,32,"PID",OLED_8X16);
			OLED_ShowString(0,48,"BAT",OLED_8X16);
		  OLED_ShowString(24,16*(OLED_Choose_Row)-16,"*",OLED_8X16);
		  OLED_ReverseArea(0,16*(OLED_Choose_Row)-16,32,16);
			OLED_Printf(50,0 ,OLED_6X8,"Fly_Zero:");
			OLED_Printf(50,12,OLED_6X8,"Preci:%04.3f",Tuning_precision[Tuning_precision_index]);
			OLED_Printf(50,28,OLED_6X8,"pitch:%04.3f",fly_pitch_zero);
			OLED_Printf(50,40,OLED_6X8,"roll:%04.3f",fly_roll_zero);
			OLED_Printf(50,52,OLED_6X8,"yaw: RESET",fly_yaw_zero);
			switch(OLED_Choose_Data[OLED_Choose_Page])
			{	
				case 1://选择精度
					OLED_ReverseArea(52,12,66,8);
				  break;
				case 2://第一个数据
					OLED_ReverseArea(50,28,72,8);
					break;
				case 3://第二个数据
					OLED_ReverseArea(50,40,72,8);
					break;
			  case 4://第三个数据
					OLED_ReverseArea(50,52,72,8);
					break;
			}
	  	OLED_Update();
			break;
		
		case 3:/*参数设置*/
			OLED_Clear();
		 	OLED_ShowString(0,0 ,"CTL",OLED_8X16);
			OLED_ShowString(0,16,"FLY",OLED_8X16);
			OLED_ShowString(0,32,"PID",OLED_8X16);
			OLED_ShowString(0,48,"BAT",OLED_8X16);
		  OLED_ShowString(24,16*(OLED_Choose_Row)-16,"*",OLED_8X16);
		  OLED_ReverseArea(0,16*(OLED_Choose_Row)-16,32,16);
		  /*PID调试*/
			if(OLED_Choose_Data[OLED_Choose_Page]<=4)
			{
				OLED_Printf(52,0 ,OLED_6X8,"Angle_Pitch:");
				OLED_Printf(52,12,OLED_6X8,"Preci:%04.3f",Tuning_precision[Tuning_precision_index]);
				OLED_Printf(52,28,OLED_6X8,"P:%04.3f",pitch_angle_balance_Kp);
				OLED_Printf(52,40,OLED_6X8,"I:%04.3f",pitch_angle_balance_Ki);
				OLED_Printf(52,52,OLED_6X8,"D:%04.3f",pitch_angle_balance_Kd);
			}
			else if(OLED_Choose_Data[OLED_Choose_Page]<=8)
			{
				OLED_Printf(52,0,OLED_6X8,"Angle_Roll:");
				OLED_Printf(52,12,OLED_6X8,"Preci:%04.3f",Tuning_precision[Tuning_precision_index]);
				OLED_Printf(52,28,OLED_6X8,"P:%04.3f",roll_angle_balance_Kp);
				OLED_Printf(52,40,OLED_6X8,"I:%04.3f",roll_angle_balance_Ki);
				OLED_Printf(52,52,OLED_6X8,"D:%04.3f",roll_angle_balance_Kd);
	   	}
			else if(OLED_Choose_Data[OLED_Choose_Page]<=12)
			{
				OLED_Printf(52,0,OLED_6X8,"Angle_Yaw:");
				OLED_Printf(52,12,OLED_6X8,"Preci:%04.3f",Tuning_precision[Tuning_precision_index]);
				OLED_Printf(52,28,OLED_6X8,"P:%04.3f",yaw_angle_balance_Kp);
				OLED_Printf(52,40,OLED_6X8,"I:%04.3f",yaw_angle_balance_Ki);
				OLED_Printf(52,52,OLED_6X8,"D:%04.3f",yaw_angle_balance_Kd);
			}
			else if(OLED_Choose_Data[OLED_Choose_Page]<=16)
			{
				OLED_Printf(52,0,OLED_6X8,"Gyro_Pitch:");
				OLED_Printf(52,12,OLED_6X8,"Preci:%04.3f",Tuning_precision[Tuning_precision_index]);
				OLED_Printf(52,28,OLED_6X8,"P:%04.3f",pitch_gyro_balance_Kp);
				OLED_Printf(52,40,OLED_6X8,"I:%04.3f",pitch_gyro_balance_Ki);
				OLED_Printf(52,52,OLED_6X8,"D:%04.3f",pitch_gyro_balance_Kd);
			}
			else if(OLED_Choose_Data[OLED_Choose_Page]<=20)
			{
				OLED_Printf(52,0,OLED_6X8,"Gyro_Roll:");
				OLED_Printf(52,12,OLED_6X8,"Preci:%04.3f",Tuning_precision[Tuning_precision_index]);
				OLED_Printf(52,28,OLED_6X8,"P:%04.3f",roll_gyro_balance_Kp);
				OLED_Printf(52,40,OLED_6X8,"I:%04.3f",roll_gyro_balance_Ki);
				OLED_Printf(52,52,OLED_6X8,"D:%04.3f",roll_gyro_balance_Kd);
				
			}
			else if(OLED_Choose_Data[OLED_Choose_Page]<=24)
			{
				OLED_Printf(52,0,OLED_6X8,"Gyro_Yaw:");
				OLED_Printf(52,12,OLED_6X8,"Preci:%04.3f",Tuning_precision[Tuning_precision_index]);
				OLED_Printf(52,28,OLED_6X8,"P:%04.3f",yaw_gyro_balance_Kp);
				OLED_Printf(52,40,OLED_6X8,"I:%04.3f",yaw_gyro_balance_Ki);
				OLED_Printf(52,52,OLED_6X8,"D:%04.3f",yaw_gyro_balance_Kd);
			}
			switch(OLED_Choose_Data[OLED_Choose_Page])
			{
				case 1:
				case 5:
				case 9:
				case 13:
				case 17:
				case 21:
					OLED_ReverseArea(52,12,66,8);
				break;

				//显示外环数据
				case 2://第一个数据
				case 6://第四个数据
				case 10://第七个数据
					OLED_ReverseArea(52,28,42,8);
					break;
				case 3://第二个数据
				case 7://第五个数据
				case 11://第八个数据
					OLED_ReverseArea(52,40,42,8);
					break;
				case 4://第三个数据
				case 8://第六个数据
				case 12://第九个数据
					OLED_ReverseArea(52,52,42,8);
					break;
				
				//显示内环数据
				case 14://第一个数据
				case 18://第四个数据
				case 22://第七个数据
					OLED_ReverseArea(52,28,42,8);
					break;
				case 15://第二个数据
				case 19://第五个数据
				case 23://第八个数据
					OLED_ReverseArea(52,40,42,8);
					break;
				case 16://第三个数据
				case 20://第六个数据
				case 24://第九个数据
					OLED_ReverseArea(52,52,42,8);
					break;
			}
			
	    OLED_Update();	
			break;
			
	  case 4:/*显示电量*/
			OLED_Clear();
		
		 	OLED_ShowString(0,0 ,"CTL",OLED_8X16);
			OLED_ShowString(0,16,"FLY",OLED_8X16);
			OLED_ShowString(0,32,"PID",OLED_8X16);
			OLED_ShowString(0,48,"BAT",OLED_8X16);
		  OLED_ShowString(24,16*(OLED_Choose_Row)-16,"*",OLED_8X16);
		  OLED_ReverseArea(0,16*(OLED_Choose_Row)-16,32,16);
		
    	OLED_Printf(50,0,OLED_8X16,"B:%04.2f",Battery_Value);
		
		  OLED_Update();
		  break;
	}

}

/*数据发送*/
void Data_Send(void)
{
	//发送格式： 
	Send_Data[0]=Lock;				 //解锁
	Send_Data[1]=LX; 					 //油门
	Send_Data[2]=RX; 					 //俯仰角度
	Send_Data[3]=RY;           //翻滚角度
	Send_Data[4]=Yaw_Control;  //航向角度
	Send_Data[5]=connect_value;//保护值
	//调试相关
	Send_Data[6]=index_type;      //数据类型
	Send_Data[7]=PID_Data_Send[0];//浮点数的整数
	Send_Data[8]=PID_Data_Send[1];//浮点数的小数
	
	Send(Send_Data);
}

/*数据接收*/
void Data_Receive(void)
{
	if(NRF24L01_R_IRQ()==0)
	{
		Receive_NRF24L01_Data(Receive_Data);
	}

}
