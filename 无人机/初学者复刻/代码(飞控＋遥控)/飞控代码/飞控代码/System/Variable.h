#ifndef __VARIABLE_H
#define __VARIABLE_H

//将所有的变量放在这里，并允许外部引用

/*-------------------变量-------------------*/
#define SlowDown 0


/*数据交换数组*/
extern uint8_t Send_Data[9];
extern uint8_t Receive_Data[9];
extern uint8_t TxorRxVal;

/*定时器变量*/  
extern uint16_t Receive_Slow_time;
extern uint16_t Send_Slow_time;
extern uint16_t Proc_Slow_time;
extern uint16_t Motor_Slow_time;
extern uint16_t Show_Slow_time;
extern uint16_t Task_Slow_time;
extern uint8_t Attitude_Proc_time;
extern uint16_t PowerDete_Slow_time;
extern uint16_t LED_Slow_time;

/*获取陀螺仪*/
extern float Pitch,Roll,Yaw;
extern int16_t ax,ay,az;
extern float gx,gy,gz;

/*遥控数据处理相关*/
extern float getpitch;//获得的俯仰遥控数据
extern float getroll;//获得的翻滚遥控数据
extern float getyaw;//获得的航向遥控数据
extern float getpitch_SET;//由获得的俯仰遥控数据转化为设置的数据
extern float getroll_SET;//由获得的翻滚遥控数据转化为设置的数据
extern float getyaw_SET;//由获得的航向遥控数据转化为设置的数据

/*按键*/
extern uint8_t Lock;

/*机械零点值*/
extern float fly_pitch_zero;//俯仰机械零点
extern float fly_roll_zero; //翻滚机械零点
extern float fly_yaw_zero;  //航向机械零点
extern float fly_yaw_zero_bit;
extern float fly_yaw_zero_bit_old;

/*PID外环角度环*/
extern float pitch_angle_balance_Kp;
extern float pitch_angle_balance_Ki;
extern float pitch_angle_balance_Kd;
extern float roll_angle_balance_Kp;
extern float roll_angle_balance_Ki;
extern float roll_angle_balance_Kd;
extern float yaw_angle_balance_Kp;
extern float yaw_angle_balance_Ki;
extern float yaw_angle_balance_Kd;
/*PID内环角度环*/
extern float pitch_gyro_balance_Kp;
extern float pitch_gyro_balance_Ki;
extern float pitch_gyro_balance_Kd;
extern float roll_gyro_balance_Kp;
extern float roll_gyro_balance_Ki;
extern float roll_gyro_balance_Kd;
extern float yaw_gyro_balance_Kp;
extern float yaw_gyro_balance_Ki;
extern float yaw_gyro_balance_Kd;
/*PID积分项*/
extern float PID_Pitch_Angle_Integral;
extern float PID_Pitch_Gyro_Integral;
extern float PID_Roll_Angle_Integral;
extern float PID_Roll_Gyro_Integral;
extern float PID_Yaw_Angle_Integral;
extern float PID_Yaw_Gyro_Integral;

//角度误差
extern float PID_Pitch_Angle_err;         //俯仰角度误差
extern float PID_Roll_Angle_err;          //翻滚角度误差
extern float PID_Yaw_Angle_err;           //航向角度误差
//角速环当前误差与上一次误差
extern float PID_Pitch_Gyro_err;
extern float PID_Pitch_Gyro_err_last;
extern float PID_Roll_Gyro_err;
extern float PID_Roll_Gyro_err_last;
extern float PID_Yaw_Gyro_err;
extern float PID_Yaw_Gyro_err_last;

/*外环与内环输出值*/
extern float Pitch_Angle_out;
extern float Roll_Angle_out;
extern float Yaw_Angle_out;
extern float Pitch_Balance_out;
extern float Roll_Balance_out;
extern float Yaw_Balance_out;

extern uint16_t PWM_OUT1,PWM_OUT2,PWM_OUT3,PWM_OUT4; //最后需施加的PWM
extern uint16_t Oil_Set;														 //施加的油门
extern uint16_t Oil_Get;		                         //获得的油门

/*连接保护功能*/
extern uint8_t connect_value;
extern uint8_t connect_oldvalue;
extern uint16_t connect_time;
extern uint8_t connect_protect;
extern uint8_t connect_bit;

/*电量检测*/
extern uint8_t Battery_Protect;//电量保护标志位
extern uint8_t Battery_num;//获取电量的次数
extern float Battery_temp;//获取当前的电量值
extern float Battery_total;//获取累计的电量值
extern float Battery_average;//获取电量的的平均值

/*机械零点值和PID设置*/
extern uint8_t index_type;
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

extern float PID_Data[6][3];
//6：前三是外环俯仰，外环翻滚，外环航向
//   后三是内环俯仰，内环翻滚，内环航向
//3：P的值，I的值，D的值

extern uint8_t PID_Data_Send[2];
//把浮点数转化为8两个进制数

/*-------------------变量-------------------*/


#endif
