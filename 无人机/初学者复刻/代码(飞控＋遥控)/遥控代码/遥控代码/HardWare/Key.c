#include "stm32f10x.h"                  // Device header
#include "Variable.h"


#define INPUT1 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define INPUT2 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)
#define INPUT3 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)
#define INPUT4 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)

#define OUTPUT1(x) GPIO_WriteBit(GPIOB,GPIO_Pin_12,(BitAction)(x))
#define OUTPUT2(x) GPIO_WriteBit(GPIOB,GPIO_Pin_6,(BitAction)(x))


void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	/*配置GPIO*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
}


uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if (Key_Num)
	{
		Temp = Key_Num;
		Key_Num = 0;
		return Temp;
	}
	return 0;
}

uint8_t Key_GetState(void)
{
	
	OUTPUT1(0);OUTPUT2(1);
	if(INPUT1==1 && INPUT2==1 && INPUT3==1 && INPUT4==0)	return 1;
	if(INPUT1==1 && INPUT2==1 && INPUT3==0 && INPUT4==1)	return 2;
	if(INPUT1==1 && INPUT2==0 && INPUT3==1 && INPUT4==1)	return 3;
	if(INPUT1==0 && INPUT2==1 && INPUT3==1 && INPUT4==1)	return 4;
	
	OUTPUT1(1);OUTPUT2(0);
	if(INPUT1==1 && INPUT2==1 && INPUT3==1 && INPUT4==0)	return 5;
	if(INPUT1==1 && INPUT2==1 && INPUT3==0 && INPUT4==1)	return 6;
	if(INPUT1==1 && INPUT2==0 && INPUT3==1 && INPUT4==1)	return 7;
	if(INPUT1==0 && INPUT2==1 && INPUT3==1 && INPUT4==1)	return 8;
	

	return 0;
}

void Key_Tick(void)
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;
	
	Count ++;
	if (Count >= 10)
	{
		Count = 0;
		
		PrevState = CurrState;
		CurrState = Key_GetState();
		
		if (CurrState == 0 && PrevState != 0)
		{
			Key_Num = PrevState;
		}
	}
}

//OLED_Choose_Row 意思是菜单列表界面的第几行，就是选择要进入哪个列表(按确认后进入)
//OLED_Choose_Page意思是菜单列表里第几个的内容是什么，0为选择列表的界面（初始界面）
void KeyNum(void)
{
		Key_Tick();
		Key_NumberCurr=Key_GetNum();
		
		if(Key_NumberCurr==1)   //上一个
		{
			switch(OLED_Choose_Page)
			{
				case 0://菜单界面
				  OLED_Choose_Row--;
			    if(OLED_Choose_Row<1) OLED_Choose_Row=1;
					break;
				case 2://飞行数据（机械零点调整）
					OLED_Choose_Data[OLED_Choose_Page]--;
				  if(OLED_Choose_Data[OLED_Choose_Page] <1) 
					OLED_Choose_Data[OLED_Choose_Page] =1;
					break;
				case 3://PID数据（飞控PID调整）
					OLED_Choose_Data[OLED_Choose_Page]--;
				  if(OLED_Choose_Data[OLED_Choose_Page] <1) 
					OLED_Choose_Data[OLED_Choose_Page] =1;
					break;
			}
			
			Key_LED=1;
		}
	  else if(Key_NumberCurr==2)//下一个
		{		
			switch(OLED_Choose_Page)
			{
				case 0://菜单界面
					OLED_Choose_Row++;
			    if(OLED_Choose_Row>4) OLED_Choose_Row=4;
					break;
				case 2://飞行数据（机械零点调整）
					OLED_Choose_Data[OLED_Choose_Page]++;
				  if(OLED_Choose_Data[OLED_Choose_Page] >4) 
					OLED_Choose_Data[OLED_Choose_Page] =4;
			  	break;
				case 3://PID数据（飞控PID调整）
					OLED_Choose_Data[OLED_Choose_Page]++;
				 if(OLED_Choose_Data[OLED_Choose_Page] >24) 
					OLED_Choose_Data[OLED_Choose_Page] =24;
					break;
			}
						
			Key_LED=2;
		}
	  else if(Key_NumberCurr==3)//数值减
		{			
			switch(OLED_Choose_Data[2])
			{
				case 1://选择精度
					if(Tuning_precision_index==0) Tuning_precision_index=3;
					Tuning_precision_index--;
				  break;
				
				case 2:
					fly_pitch_zero-=Tuning_precision[Tuning_precision_index];
					break;
				case 3:
					fly_roll_zero-=Tuning_precision[Tuning_precision_index];
					break;
				case 4:
					fly_yaw_zero=1;
					break;
			}
			
			switch(OLED_Choose_Data[3])
			{
				case 1:
				case 5:
				case 9:
				case 13:
				case 17:
				case 21:
					if(Tuning_precision_index==0) Tuning_precision_index=3;
					Tuning_precision_index--;
				  break;

				
				//外环
				case 2:
					pitch_angle_balance_Kp-=Tuning_precision[Tuning_precision_index];
				  if(pitch_angle_balance_Kp<0)pitch_angle_balance_Kp=0;
					break;
				case 3:
					pitch_angle_balance_Ki-=Tuning_precision[Tuning_precision_index];
				  if(pitch_angle_balance_Ki<0)pitch_angle_balance_Ki=0;
					break;
				case 4:
					pitch_angle_balance_Kd-=Tuning_precision[Tuning_precision_index];
				  if(pitch_angle_balance_Kd<0) pitch_angle_balance_Kd=0;
					break;
				case 6:
					roll_angle_balance_Kp-=Tuning_precision[Tuning_precision_index];
				  if(roll_angle_balance_Kp<0)roll_angle_balance_Kp=0;
					break;
				case 7:
					roll_angle_balance_Ki-=Tuning_precision[Tuning_precision_index];
				  if(roll_angle_balance_Ki<0)roll_angle_balance_Ki=0;
				  break;
				case 8:
					roll_angle_balance_Kd-=Tuning_precision[Tuning_precision_index];
			  	if(roll_angle_balance_Kd<0)	roll_angle_balance_Kd=0;
					break;
				case 10:
					yaw_angle_balance_Kp-=Tuning_precision[Tuning_precision_index];
				  if(yaw_angle_balance_Kp<0)yaw_angle_balance_Kp=0;
					break;
				case 11:
					yaw_angle_balance_Ki-=Tuning_precision[Tuning_precision_index];
			  	if(yaw_angle_balance_Ki<0)yaw_angle_balance_Ki=0;
					break;
				case 12:
					yaw_angle_balance_Kd-=Tuning_precision[Tuning_precision_index];
				  if(yaw_angle_balance_Kd<0) yaw_angle_balance_Kd=0;
					break;
				
				//内环
				case 14:
					pitch_gyro_balance_Kp-=Tuning_precision[Tuning_precision_index];
				  if(pitch_gyro_balance_Kp<0) pitch_gyro_balance_Kp=0;
					break;
				case 15:
					pitch_gyro_balance_Ki-=Tuning_precision[Tuning_precision_index];
				  if(pitch_gyro_balance_Ki<0) pitch_gyro_balance_Ki=0;
					break;
				case 16:
					pitch_gyro_balance_Kd-=Tuning_precision[Tuning_precision_index];
			  	if(pitch_gyro_balance_Kd<0) pitch_gyro_balance_Kd=0;
					break;
				case 18:
					roll_gyro_balance_Kp-=Tuning_precision[Tuning_precision_index];
				  if(roll_gyro_balance_Kp<0) roll_gyro_balance_Kp=0;
					break;
				case 19:
					roll_gyro_balance_Ki-=Tuning_precision[Tuning_precision_index];
			  	if(roll_gyro_balance_Ki<0) roll_gyro_balance_Ki=0;
				  break;
				case 20:
					roll_gyro_balance_Kd-=Tuning_precision[Tuning_precision_index];
				  if(roll_gyro_balance_Kd<0) roll_gyro_balance_Kd=0;
					break;
				case 22:
					yaw_gyro_balance_Kp-=Tuning_precision[Tuning_precision_index];
			  	if(yaw_gyro_balance_Kp<0) yaw_gyro_balance_Kp=0;
					break;
				case 23:
					yaw_gyro_balance_Ki-=Tuning_precision[Tuning_precision_index];
			  	if(yaw_gyro_balance_Ki<0) yaw_gyro_balance_Ki=0;
					break;
				case 24:
					yaw_gyro_balance_Kd-=Tuning_precision[Tuning_precision_index];
				  if(yaw_gyro_balance_Kd<0) yaw_gyro_balance_Kd=0;
					break;
			}
		}
	  else if(Key_NumberCurr==4)//数值加
		{
			switch(OLED_Choose_Data[2])
			{
				case 1://选择精度
					Tuning_precision_index++;
				  if(Tuning_precision_index>2) Tuning_precision_index=0;
				  break;
				
				case 2:
					fly_pitch_zero+=Tuning_precision[Tuning_precision_index];
					break;
				case 3:
					fly_roll_zero+=Tuning_precision[Tuning_precision_index];
					break;
				case 4:
					fly_yaw_zero=1;
					break;
			}
			switch(OLED_Choose_Data[3])
			{
				case 1:
				case 5:
				case 9:
				case 13:
				case 17:
				case 21:
				  Tuning_precision_index++;
				  if(Tuning_precision_index>2) Tuning_precision_index=0;
				  break;
	
				
				//外环
				case 2:
					pitch_angle_balance_Kp+=Tuning_precision[Tuning_precision_index];
					break;
				case 3:
					pitch_angle_balance_Ki+=Tuning_precision[Tuning_precision_index];
					break;
				case 4:
					pitch_angle_balance_Kd+=Tuning_precision[Tuning_precision_index];
					break;
				case 6:
					roll_angle_balance_Kp+=Tuning_precision[Tuning_precision_index];
					break;
				case 7:
					roll_angle_balance_Ki+=Tuning_precision[Tuning_precision_index];
				  break;
				case 8:
					roll_angle_balance_Kd+=Tuning_precision[Tuning_precision_index];
					break;
				case 10:
					yaw_angle_balance_Kp+=Tuning_precision[Tuning_precision_index];
					break;
				case 11:
					yaw_angle_balance_Ki+=Tuning_precision[Tuning_precision_index];
					break;
				case 12:
					yaw_angle_balance_Kd+=Tuning_precision[Tuning_precision_index];
					break;
				
				//内环
				case 14:
					pitch_gyro_balance_Kp+=Tuning_precision[Tuning_precision_index];
					break;
				case 15:
					pitch_gyro_balance_Ki+=Tuning_precision[Tuning_precision_index];
					break;
				case 16:
					pitch_gyro_balance_Kd+=Tuning_precision[Tuning_precision_index];
					break;
				case 18:
					roll_gyro_balance_Kp+=Tuning_precision[Tuning_precision_index];
					break;
				case 19:
					roll_gyro_balance_Ki+=Tuning_precision[Tuning_precision_index];
				  break;
				case 20:
					roll_gyro_balance_Kd+=Tuning_precision[Tuning_precision_index];
					break;
				case 22:
					yaw_gyro_balance_Kp+=Tuning_precision[Tuning_precision_index];
					break;
				case 23:
					yaw_gyro_balance_Ki+=Tuning_precision[Tuning_precision_index];
					break;
				case 24:
					yaw_gyro_balance_Kd+=Tuning_precision[Tuning_precision_index];
					break;
			}

		}
 
		else if(Key_NumberCurr==7)
		{	
			Lock=1;
			Key_LED=7;
		}
	  else if(Key_NumberCurr==8) 
		{
			Lock=0;
			Key_LED=8;
		}
	  else if(Key_NumberCurr==5) /*返回菜单界面*/
		{
			OLED_Choose_Page=0;
			//清除选择的数据
			for(int i=0;i<=4;i++)
			OLED_Choose_Data[i]=0;
		}
	  else if(Key_NumberCurr==6) /*确认按键*/
		{
			/*俯仰机械零点重置*/
			if(OLED_Choose_Data[2]==4)
			{
				fly_yaw_zero++;
				if(fly_yaw_zero>100) fly_yaw_zero=0;
			}
			
  		OLED_Choose_Page=OLED_Choose_Row;
			OLED_Choose_Data[OLED_Choose_Page]=1;
					
			Key_LED=11;
		}
	
	
	
	
}
