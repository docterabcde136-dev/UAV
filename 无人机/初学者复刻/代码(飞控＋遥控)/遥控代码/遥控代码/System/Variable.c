/*该文件定义所以相关的变量，并允许对外引用*/
#include "stm32f10x.h"                  // Device header

/*-------------------变量-------------------*/
/*遥感变量数组*/ 
uint16_t Rocker_Value[5];

/*遥感方向变量*/
float LX,LY;			      //左摇杆
float RX,RY;				    //右摇杆

/*数据交换数组*/
uint8_t Send_Data[9];
uint8_t Receive_Data[9];
uint8_t TxorRxVal;

/*定时器变量，单位为10ms*/  
uint16_t Send_Slow_time;
uint16_t Receive_Slow_time;
uint16_t Proc_Slow_time;
uint16_t Disp_Slow_time;
uint16_t LED_Slow_time;

/*角度外环*/
float pitch_angle_balance_Kp=2.85;
float pitch_angle_balance_Ki=0.003;
float pitch_angle_balance_Kd=0.048;
float roll_angle_balance_Kp=2.85;
float roll_angle_balance_Ki=0.003;
float roll_angle_balance_Kd=0.048;
float yaw_angle_balance_Kp=2.85;
float yaw_angle_balance_Ki=0.003;
float yaw_angle_balance_Kd=0.048;
/*角速度内环*/
float pitch_gyro_balance_Kp=4.0;
float pitch_gyro_balance_Ki=0;
float pitch_gyro_balance_Kd=0.025;
float roll_gyro_balance_Kp=4.0;
float roll_gyro_balance_Ki=0;
float roll_gyro_balance_Kd=0.025;
float yaw_gyro_balance_Kp=4.0;
float yaw_gyro_balance_Ki=0;
float yaw_gyro_balance_Kd=0.025;
/*零点*/
float fly_pitch_zero=-3.5;//俯仰机械零点
float fly_roll_zero=0; //翻滚机械零点
float fly_yaw_zero;  //航向机械零点标志位

/*按键*/
uint8_t Key_Num;
uint8_t Lock=1;   //1为上锁，0为解锁
uint8_t Key_NumberCurr=7;//获取当前按键值
uint8_t Key_LED=7;

/*连接保护功能*/
uint8_t connect_value;
uint16_t connect_time;

/*电量检测*/
float Battery_Value;

/*机械零点值和PID设置*/
uint8_t index_type;
//  0为不变
//  1为俯仰机械零点
//  2为翻滚机械零点
//  3为航向机械零点重置
//  4为外环俯仰的P的值
//  5为外环俯仰的I的值
//  6为外环俯仰的D的值
//  7为外环翻滚的P的值
//  8为外环翻滚的I的值
//  9为外环翻滚的D的值
// 10为外环航向的P的值
// 11为外环航向的I的值
// 12为外环航向的D的值
// 13为内环俯仰的P的值
// 14为内环俯仰的I的值
// 15为内环俯仰的D的值
// 16为内环翻滚的P的值
// 17为内环翻滚的I的值
// 18为内环翻滚的D的值
// 19为内环航向的P的值
// 20为内环航向的I的值
// 21为内环航向的D的值

float PID_Data[6][3];
//6：前三是外环俯仰，外环翻滚，外环航向
//   后三是内环俯仰，内环翻滚，内环航向
//3：P的值，I的值，D的值

uint8_t PID_Data_Send[2];
//把浮点数转化为8两个进制数

/*OLED显示*/
uint8_t OLED_Choose_Row=1;//显示的行
uint8_t OLED_Choose_Page=0;//选择页
uint8_t OLED_Choose_Data[4];//数据修改页
//0--主菜单的，不可修改
//1--遥感的，不可修改
//2--机械零点，可修改
//3--PID设置，可修改

int16_t Encoder_pulse;
int16_t Encoder_number;
int16_t Yaw_Control;

//调试精度
float Tuning_precision[3]={0.001,0.01,0.1};
uint8_t Tuning_precision_index;
/*-------------------变量-------------------*/
