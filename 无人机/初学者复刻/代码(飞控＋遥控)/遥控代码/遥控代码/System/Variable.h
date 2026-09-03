#ifndef __VARIABLE_H
#define __VARIABLE_H

//将所有的变量放在这里，并允许外部引用

/*-------------------变量-------------------*/
/*遥感变量数组*/
extern uint16_t Rocker_Value[5];

/*遥感方向变量*/
extern int16_t LX,LY;
extern int16_t RX,RY;
extern float Rocker_coe;
	
/*数据交换数组*/
extern uint8_t Send_Data[9];
extern uint8_t Receive_Data[9];
extern uint8_t TxorRxVal;
/*定时器变量，单位为10ms*/  
extern uint16_t Send_Slow_time;
extern uint16_t Receive_Slow_time;
extern uint16_t Proc_Slow_time;
extern uint16_t Disp_Slow_time;
extern uint16_t LED_Slow_time;

/*速度外环*/
extern float pitch_angle_balance_Kp;//5
extern float pitch_angle_balance_Ki;//5
extern float pitch_angle_balance_Kd;//0.5

extern float roll_angle_balance_Kp;//5
extern float roll_angle_balance_Ki;//5
extern float roll_angle_balance_Kd;//0.5

extern float yaw_angle_balance_Kp;//5
extern float yaw_angle_balance_Ki;//5
extern float yaw_angle_balance_Kd;//0.2

/*角速度内环*/
extern float pitch_gyro_balance_Kp;//5
extern float pitch_gyro_balance_Ki;//5
extern float pitch_gyro_balance_Kd;//0.5

extern float roll_gyro_balance_Kp;//5
extern float roll_gyro_balance_Ki;//5
extern float roll_gyro_balance_Kd;//0.5

extern float yaw_gyro_balance_Kp;//5
extern float yaw_gyro_balance_Ki;//5
extern float yaw_gyro_balance_Kd;//0.2



extern float fly_pitch_zero;//俯仰机械零点1.4
extern float fly_roll_zero; //翻滚机械零点1.6
extern float fly_yaw_zero;  //航向机械零点12.10
extern uint8_t fly_yaw_zero_bit;

extern uint8_t i;
extern uint8_t num;
extern int16_t aaa;
extern uint8_t Key_NumberCurr;
extern uint8_t Key_LED;

/*按键*/
extern uint8_t Lock;
extern uint8_t Key_Num;
extern uint8_t Key;

/*连接保护功能*/
extern uint8_t connect_value;
extern uint16_t connect_time;

/*电量检测*/
extern float Battery_Value;
//extern float Battery_data;


/*机械零点值和PID设置*/
extern uint8_t index_type;
// 0为不变
// 1为俯仰机械零点
// 2为翻滚机械零点
// 3为俯仰的P的值
// 4为俯仰的D的值
// 5为翻滚的P的值
// 6为翻滚的D的值
// 7为航向的P的值
// 8为航向的D的值
extern float PID_Data[6][3]; 
//3是俯仰，翻滚，航向
//2是P的值和D的值

extern uint8_t PID_Data_Send[2];
//把浮点数转化为8两个进制数


/*反向显示的行*/
extern uint8_t OLED_Choose_Row;
extern uint8_t OLED_Choose_Page;
extern uint8_t OLED_Choose_Data[4];


extern int16_t Encoder_pulse;
extern int16_t Encoder_number;
extern int16_t Yaw_Control;

//调试精度
extern float Tuning_precision[3];
extern uint8_t Tuning_precision_index;

/*-------------------变量-------------------*/


#endif
