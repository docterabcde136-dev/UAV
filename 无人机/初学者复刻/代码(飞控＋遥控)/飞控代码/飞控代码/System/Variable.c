/*该文件定义所以相关的变量，并允许对外引用*/
#include "stm32f10x.h"                  // Device header

/*-------------------变量-------------------*/
/*数据交换数组*/
uint8_t Send_Data[9]={0};
uint8_t Receive_Data[9]={0};
uint8_t TxorRxVal;

/*定时器变量，单位为1ms*/  
uint16_t Receive_Slow_time=1;
uint16_t Send_Slow_time=1;
uint16_t Proc_Slow_time=1;
uint16_t Motor_Slow_time=1;
uint16_t Show_Slow_time=1;
uint16_t Task_Slow_time=1;
uint8_t Attitude_Proc_time=1;
uint16_t PowerDete_Slow_time=1;
uint16_t LED_Slow_time=1;

/*获取陀螺仪*/
float Pitch,Roll,Yaw;
int16_t ax,ay,az;
float gx,gy,gz;

/*遥控数据处理相关*/
float getpitch;//获得的俯仰遥控数据
float getroll;//获得的翻滚遥控数据
float getyaw;//获得的航向遥控数据
float getpitch_SET;//由获得的俯仰遥控数据转化为设置的数据
float getroll_SET;//由获得的翻滚遥控数据转化为设置的数据
float getyaw_SET;//由获得的航向遥控数据转化为设置的数据

/*锁键*/
uint8_t Lock;

/*机械零点值*/
float fly_pitch_zero=0;  //俯仰机械零点1.4
float fly_roll_zero=0;    //翻滚机械零点1.6
float fly_yaw_zero=0;   //航向机械零点12.10
float fly_yaw_zero_bit;
float fly_yaw_zero_bit_old;
/*PID外环角度环*/
float pitch_angle_balance_Kp=0;//5
float pitch_angle_balance_Ki=0;
float pitch_angle_balance_Kd=0;//0.5
float roll_angle_balance_Kp=0;//5
float roll_angle_balance_Ki=0;
float roll_angle_balance_Kd=0;//0.5
float yaw_angle_balance_Kp=0;//5
float yaw_angle_balance_Ki=0;
float yaw_angle_balance_Kd=0;//0.2
/*PID内环角速度环*/
float pitch_gyro_balance_Kp=5.2;//5
float pitch_gyro_balance_Ki=0;
float pitch_gyro_balance_Kd=0.05;//0.5
float roll_gyro_balance_Kp=1.00;//5
float roll_gyro_balance_Ki=0;
float roll_gyro_balance_Kd=0.05;//0.5
float yaw_gyro_balance_Kp=1.5;//5
float yaw_gyro_balance_Ki=0;
float yaw_gyro_balance_Kd=0.02;//0.2

/*PID积分项*/
float PID_Pitch_Angle_Integral;
float PID_Pitch_Gyro_Integral;
float PID_Roll_Angle_Integral;
float PID_Roll_Gyro_Integral;
float PID_Yaw_Angle_Integral;
float PID_Yaw_Gyro_Integral;

//角度误差
float PID_Pitch_Angle_err;         //俯仰角度误差
float PID_Roll_Angle_err;          //翻滚角度误差
float PID_Yaw_Angle_err;           //航向角度误差
//角速环当前误差与上一次误差
float PID_Pitch_Gyro_err;
float PID_Pitch_Gyro_err_last;
float PID_Roll_Gyro_err;
float PID_Roll_Gyro_err_last;
float PID_Yaw_Gyro_err;
float PID_Yaw_Gyro_err_last;

/*外环与内环输出值*/
float Pitch_Angle_out;
float Roll_Angle_out;
float Yaw_Angle_out;
float Pitch_Balance_out;
float Roll_Balance_out;
float Yaw_Balance_out;

int16_t PWM_OUT1,PWM_OUT2,PWM_OUT3,PWM_OUT4;   //最后需施加的PWM
uint16_t Oil_Set;															 //施加的油门
uint16_t Oil_Get;													     //获得的油门

/*连接保护功能*/
uint8_t connect_protect=1;   //连接保护标志位
uint8_t connect_value;     //时时刻刻获取到的值
uint8_t connect_oldvalue;  //上一个时间节点对于的值
uint16_t connect_time;     //延迟时间
uint8_t connect_bit;       //每隔一段时间切换的标志位

/*电量检测*/
uint8_t Battery_Protect;//电量保护标志位
uint8_t Battery_num;//获取电量的次数
float Battery_temp;//获取当前的电量值
float Battery_total;//获取累计的电量值
float Battery_average=4.2;//获取电量的的平均值


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

/*-------------------变量-------------------*/
