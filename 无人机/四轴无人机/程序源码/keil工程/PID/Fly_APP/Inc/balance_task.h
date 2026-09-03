/**
 * ============================================================
 * balance_task.h — 四轴飞行器平衡控制任务头文件
 * ============================================================
 * 定义:
 *   - DShot油门范围 (48~1500, 最大2047)
 *   - 事件标志位 (23位: 定高模式/标定/启动/低电量/测试/避障...)
 *   - 控制来源优先级 (IDLE < APP < PS2 < AVOID)
 *   - 控制量结构体 (FlyControlType_t: 俯仰/横滚/偏航/高度)
 * ============================================================
 */

#ifndef __BALANCE_TASK_H
#define __BALANCE_TASK_H

#include <stdint.h>

//Dshot油门范围(48~2047)
#define Dshot_MIN 48
#define Dshot_MAX 1500 //实际最大油门值2047

//四轴机体事件组
enum{
	UNUSE_HeightMode_Event = (1<<0),  //无定高模式事件
	IMU_CalibZeroDone_Event = (1<<1), //IMU标定完成事件
	StartFly_Event = (1<<2),           //启动飞行事件
	LowPower_Event = (1<<3),          //低电量事件
	
	//单路电机、4路电机测试
	TestMotorAup_Event   = ( 1<<4 ),
	TestMotorAdown_Event = ( 1<<5 ),
	TestMotorBup_Event   = ( 1<<6 ),
	TestMotorBdown_Event = ( 1<<7 ),
	TestMotorCup_Event   = ( 1<<8 ),
	TestMotorCdown_Event = ( 1<<9 ),
	TestMotorDup_Event   = ( 1<<10 ),
	TestMotorDdown_Event = ( 1<<11 ),
	TestMotorAllup_Event = ( 1<<12 ),
	TestMotorAlldown_Event=( 1<<13 ),
	
	//电机测试模式
	TestMotorMode_Event =  ( 1<<14 ),
	FlyMode_HeadLessMode_Event = ( 1<<15 ),
	
	//避障模式选择
	lidar_follow_mode = (1<<16),
	lidar_avoid_mode = (1<<17),
	
	/* 事件组最大23 */
};

//四轴控制源,控制优先级排序为低到高
enum{
	IDLECmd = 0,//控制空闲
	APPCmd = 1, //APP控制,最低优先级, 1 
	PS2Cmd = 2,
	AviodCmd = 3,
};

//四轴控制结构体
typedef struct{
	uint8_t source; //控制来源
	float pitch;   //前后控制,+前进,-后退
	float roll;    //左右控制,+左移,-右移
	float gyroz;     //方向控制,+逆时针,-顺时针
	float height;  //高度控制
}FlyControlType_t;

extern float g_readonly_mainTaskFreq;
extern float angle_to_rad(float angle);

//避障微调位置 (定义在 balance_task.c)
void avoid_targetpos_lc307(uint8_t flag);

#endif /* __BALANCE_TASK_H */
