#include "balance_task.h"

/* C Lib */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* RTOS */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "event_groups.h"

/* BSP */
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "bsp_ps2.h"
#include "bsp_dshot.h"
#include "bsp_adc.h"
#include "bsp_Rtosdebug.h"
#include "bsp_imu.h"
#include "bsp_stp23L.h"
#include "bsp_adc.h"
#include "main.h"

#include "lidar_task.h"
#include "show_task.h"
#include "pid.h"

typedef struct {
	dshotMotorVal_t A;
	dshotMotorVal_t B;
	dshotMotorVal_t C;
	dshotMotorVal_t D;
}MOTOR_t; /* 定义4个电机用于四轴控制 */

typedef struct{
	IMU_DATA_t* axis;
	ATTITUDE_DATA_t* attitude;
}IMU_ZEROPONIT_t; /* imu零点结构体，用于零点标定 */

//电机停止默认值
static MOTOR_t MotorStopVal = {
	.A = {0,Dshot_MIN},
	.B = {0,Dshot_MIN},
	.C = {0,Dshot_MIN},
	.D = {0,Dshot_MIN},
};

//电机控制值
static MOTOR_t motor = {
	.A = {0,Dshot_MIN},
	.B = {0,Dshot_MIN},
	.C = {0,Dshot_MIN},
	.D = {0,Dshot_MIN},
}; 

//定时器句柄
static TimerHandle_t priv_BuzzerTipsTimer,priv_WaitImuTipsTimer,priv_OperateResponseTimer,priv_OperateFullTimer,priv_UNUSEHeightTimer,priv_lowpowerTimer; 
static TimerHandle_t follow_tipsTimer;

//4轴事件定义
extern EventGroupHandle_t g_xEventFlyAction;

/* 内部使用函数 */
static void SetPwm(MOTOR_t* m);
static IMU_ZEROPONIT_t* WaitImuStable(uint16_t freq,IMU_DATA_t imu,ATTITUDE_DATA_t attitude);
static void BuzzerTipsTimer_Callback(TimerHandle_t xTimer);
static void LedTipsTimer_Callback(TimerHandle_t xTimer);
static void OperateResponse_Callback(TimerHandle_t xTimer);
static void OperateFull_Callback(TimerHandle_t xTimer);
static void UNUSEHeightTips_Callback(TimerHandle_t xTimer);
static void LowPowerTips_Callback(TimerHandle_t xTimer);
static int target_limit_s16(short insert,short low,short high);
static float target_limit_float(float insert,float low,float high);
static uint8_t check_HeightStable(float height);
static void StopVal_SelfRecovery(MOTOR_t* m);
static uint16_t weight_to_throttle(float weight,float height);
static void FollwTips_Callback(TimerHandle_t xTimer);
/* 内部使用函数  END */

//balance任务的检测频率
float g_readonly_BalanceTaskFreq = 0;

//对四轴的控制量
#define DefalutHeight 1.0f  //启动时默认的飞行高度
static float FlyControl_pitch = 0;
static float FlyControl_roll = 0;
static float FlyControl_yaw = 0;
static float FlyControl_gyroz = 0;
static float FlyControl_height = 0;
static float FlyControl_unuseHeight = 0.0f;

//微调控制值
static float userset_pitch = 0;
static float userset_roll = 0;

//控制命令优先级管理变量
static uint8_t controlCmdNumber = IDLECmd;

//起飞电压限制
#define ROBOT_VOL_LIMIT 10.0f

//定高模式,最大与最小高度设置
#define HeightMode_MAX 1.5f
#define HeightMode_Min -0.5f

//非定高模式,最大与最小高度设置
#define UNHeightMode_MAX 5.0f
#define UNHeightMode_Min -0.5f

//PID控制器
extern PIDControllerType_t RollRatePID,RollPID;
extern PIDControllerType_t PitchRatePID,PitchPID;
extern PIDControllerType_t YawRatePID,YawPID;
extern PIDControllerType_t HeightSpeedPID,HeightPID;

extern void PID_Update(PIDControllerType_t* pid,float target,float current);
extern void PID_Reset(PIDControllerType_t* pid);

extern uint8_t g_lost_pos_dev;

//Z轴角度归一化处理
#define M_PI 3.14159265f
static float normalize_radian(float angle) {
	const float TWO_PI = 2.0f * M_PI; // 2π ≈ 6.283185307
	while (angle > M_PI) {
		angle -= TWO_PI;
	}
	while (angle < -M_PI) {
		angle += TWO_PI;
	}
	return angle;
}

//实际使用的高度值
float use_distance = 0;

/* 定义滤波后的角速度 */
float gx_filtered = 0, gy_filtered = 0; 

//光流速度、位置、目标位置
float speedX=0,speedY=0;
float posX=0,posY=0;
float targetPosX=0,targetPosY=0;

//定点功能开启标志位
uint8_t startPos = 1;

// 速度(数据融合后的)
static float x_dot = 0, y_dot = 0; 


// 光流数据回调函数
void getOpticalFlowResult_Callback(float* buf)
{
	speedY = -buf[0] / 200.0f; // 速度(m/s)
	speedX = -buf[1] / 200.0f; // 速度(m/s)
	
	speedY = use_distance * (speedY - fmaxf(fminf(gx_filtered,2.0f),-2.0f)); // 旋转补偿 + 尺度缩放
	speedX = use_distance * (speedX + fmaxf(fminf(gy_filtered,2.0f),-2.0f)); // 旋转补偿 + 尺度缩放
	
	g_lost_pos_dev=0;
}

void avoid_targetpos_lc307(uint8_t flag)
{
	xTimerStart(priv_OperateFullTimer,0);
	
	switch( flag )
	{
		case 0:
			targetPosX += 0.03f;//右
			break;
		case 1:
			targetPosX -= 0.03f;//左
			break;
		case 2:
			targetPosY += 0.03f;//前
			break;
		case 3:
			targetPosY -= 0.03f;//后
			break;
		default:
			break;
	}
}

//跟随
/*
 距离：0.3米作为目标值,太近往反方向运动
       0.5米一直靠近，0.2米原理
*/
//
void follow_targetpos_lc307(uint8_t flag)
{
	//1s蜂鸣器提示
	static TickType_t LastTick;
	
	TickType_t now = xTaskGetTickCount();
	if( now - LastTick >= 1300 )
	{
		xTimerStart(follow_tipsTimer,0);
		LastTick = now;
	}
	
	switch( flag )
	{
		//靠近使用
		case 0:
			targetPosX -= 0.01f;//右
			break;
		case 1:
			targetPosX += 0.01f;//左
			break;
		case 2:
			targetPosY -= 0.01f;//前
			break;
		case 3:
			targetPosY += 0.01f;//后
			break;
		
		//远离使用
		case 4:
			targetPosX += 0.005f;//右
			break;
		case 5:
			targetPosX -= 0.005f;//左
			break;
		case 6:
			targetPosY += 0.005f;//前
			break;
		case 7:
			targetPosY -= 0.005f;//后
			break;
		
		default:
			break;
	}
}

//步进调整位置
void set_targetpos_lc307(uint8_t flag)
{
	xTimerStart(priv_OperateResponseTimer,0);
	switch( flag )
	{
		case 0:
			targetPosX += 0.2f;//B
			break;
		case 1:
			targetPosX -= 0.2f;//X
			break;
		case 2:
			targetPosY += 0.2f;//Y
			break;
		case 3:
			targetPosY -= 0.2f;//A
			break;
		default:
			break;
	}
}

void start_lc307_pos(void)
{
	startPos = !startPos;
}

void balance_task(void* param)
{
	//此任务会操作的队列
	extern QueueHandle_t g_xQueueFlyControl;
	
	//获取时基,用于辅助任务能指定固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//控制任务的频率,单位HZ
	const uint16_t TaskFreq = 200;
	
	//任务频率、时间调试变量
	pRtosDebugInterface_t debug = &RTOSTaskDebug;
	RtosDebugPrivateVar debugPriv = { 0 };
	
	//指定用到的驱动
	pIMUInterface_t imu = &UserICM20948;
	
	//ADC驱动
	pADCInterface_t adc1 = &UserADC1;
	
	//IMU零点,系统启动时、四轴起飞时,执行标定
	IMU_ZEROPONIT_t* zero_point = { NULL };
	
	//用于存放IMU数据
	IMU_DATA_t axis_9Val = { 0 };                 
	ATTITUDE_DATA_t AttitudeVal = { 0 };
	
	//系统时间,用于辅助IMU标定
	uint16_t syscount = 0;
	
	//创建不同类型的蜂鸣器提示定时器
	priv_BuzzerTipsTimer = xTimerCreate("BuzzerTips",pdMS_TO_TICKS(10),pdFALSE,NULL,BuzzerTipsTimer_Callback);
	priv_WaitImuTipsTimer = xTimerCreate("WaitImuTips",pdMS_TO_TICKS(100),pdTRUE,NULL,LedTipsTimer_Callback);
	priv_OperateResponseTimer = xTimerCreate("priv_OperateResponseTimer",pdMS_TO_TICKS(100),pdFALSE,NULL,OperateResponse_Callback);
	priv_OperateFullTimer = xTimerCreate("priv_OperateFullTimer",pdMS_TO_TICKS(100),pdFALSE,NULL,OperateFull_Callback);
	priv_UNUSEHeightTimer = xTimerCreate("UnUseHeightTimer",pdMS_TO_TICKS(100),pdFALSE,NULL,UNUSEHeightTips_Callback);
	priv_lowpowerTimer = xTimerCreate("LowPowerTimer",pdMS_TO_TICKS(100),pdFALSE,NULL,LowPowerTips_Callback);
	
	follow_tipsTimer = xTimerCreate("followtipsTimer",pdMS_TO_TICKS(1),pdFALSE,NULL,FollwTips_Callback);
	
	xTimerStart(priv_WaitImuTipsTimer,0);
	
	//事件标志位,用于获取四轴控制的事件
	EventBits_t uxBits ; 
	
	//高度零点
	float zero_distance = 0;
	
	//启动飞机时零点标定标志位
	uint8_t StarFly_UpdateFlag = 1;
	
	//高度平滑控制标志位
	uint8_t StartFly_SmoothHeightFlag = 1;
	
	//RTOS使用浮点
	portTASK_USES_FLOATING_POINT();
	
	//读取用户设定的基础零点值
	extern float g_userparam_pitchzero,g_userparam_rollzero;
	userset_pitch = g_userparam_pitchzero;
	userset_roll = g_userparam_rollzero;
	
	while(1)
	{
		//计算去除零点后的高度
		use_distance = zero_distance - g_readonly_distance;
		
		imu->Update_9axisVal(&axis_9Val);           //陀螺仪数据更新,获取的数据均为原始数据.此操作耗时 0.61 ms
		
		gx_filtered = 0.9f * gx_filtered + 0.1f * axis_9Val.gyro.x; /* 一阶低通滤波 */
		gy_filtered = 0.9f * gy_filtered + 0.1f * axis_9Val.gyro.y; /* 一阶低通滤波 */
		
		imu->UpdateAttitude(axis_9Val,&AttitudeVal);//更新姿态角
		
		//电压获取
		g_robotVOL = (float)adc1->getValue(userconfigADC_VBAT_CHANNEL)/4095.0f*3.3f*11.0f;
		
		//系统运行时间,30秒后不再计时
		if( syscount < TaskFreq*30 ) syscount++; 
		
		//获取飞行器相关的事件组
		uxBits = xEventGroupGetBits(g_xEventFlyAction);
		
		/* 标定零点程序 */
		if( 0 == (uxBits & IMU_CalibZeroDone_Event) )
		{  
			/* 开机5秒后再执行标定,等待期间需要保持四轴水平静止 */
			if( syscount>= TaskFreq * 5 )
			{
				/* 等待imu稳定并标定 */
				zero_point = WaitImuStable(TaskFreq,axis_9Val,AttitudeVal);        
				
				/* 检查高度数据是否稳定 */
				uint8_t heightstatble = check_HeightStable(g_readonly_distance);
				
				//标定顺序：6轴数据、姿态角
				if( zero_point->axis!=NULL )
				{
					imu->UpdateZeroPoint_axis(zero_point->axis);
				}
				
				if( zero_point->attitude!=NULL )
				{
					imu->UpdateZeroPoint_attitude(zero_point->attitude);           //imu姿态零点更新
					zero_distance = g_readonly_distance;                           //高度零点更新
					xTimerChangePeriod(priv_WaitImuTipsTimer,pdMS_TO_TICKS(800),0);//系统进入正常状态,LED慢闪提示
					xEventGroupSetBits(g_xEventFlyAction,IMU_CalibZeroDone_Event); //设置事件,IMU标定完成
					
					/* 高度信息稳定值异常(可能在室外).进入不定高模式 */
					if( heightstatble >= 6 )
					{
						xEventGroupSetBits(g_xEventFlyAction,UNUSE_HeightMode_Event);
						xTimerStart(priv_UNUSEHeightTimer,0);
					}
					else /* 正常模式 */
					{
						xTimerStart(priv_BuzzerTipsTimer,0); //蜂鸣器提示标定已完成
					}
						
					
					/* 控制方式,默认有头模式 */
					//xEventGroupSetBits(g_xEventFlyAction,FlyMode_HeadLessMode_Event);
				}
			}
		}
		/* 标定零点程序 END */
		
		/* 高度信息处理 */
		static float last_height=0,height_dot=0;
		static float last_height_dot = 0;
		height_dot = 0.4f*(use_distance - last_height)/0.005f + 0.6f*last_height_dot; // 一阶低通滤波
		last_height_dot = height_dot;
		last_height = use_distance;
		
		if( uxBits & UNUSE_HeightMode_Event )
		{   /* 不定高模式 */
			use_distance = FlyControl_height; 
			height_dot = 0;
			startPos=0;
		}
		/* 高度信息处理 END*/
		
		/* 读取控制指令 */
		FlyControlType_t controlVal = { 0 };
		static uint8_t refresh_cmdstate = 0;//控制状态刷新
		if( pdPASS == xQueueReceive(g_xQueueFlyControl,&controlVal,0) )
		{
			refresh_cmdstate = 0;
			
			//无头模式
			if( uxBits & FlyMode_HeadLessMode_Event )
			{
				FlyControl_pitch = -controlVal.roll * sin(AttitudeVal.yaw) + controlVal.pitch * cos(AttitudeVal.yaw);
				FlyControl_roll =  controlVal.roll * cos(AttitudeVal.yaw) + controlVal.pitch * sin(AttitudeVal.yaw);
			}
			else
			{
				FlyControl_pitch =  controlVal.pitch;
				FlyControl_roll =  controlVal.roll;
			}
			
			//Z轴控制量为累计值
			FlyControl_yaw -= controlVal.gyroz;
			
			//检查是否允许高度控制(低电量时不允许升高操作)
			if( (uxBits & LowPower_Event) && controlVal.height > 0 )
			{
				/* 进入低电量模式禁止升高 */
			}
			else
			{
				if( fabs(controlVal.height)!=0 )
					StartFly_SmoothHeightFlag=0;//起飞平滑控制参数,若用户介入控制则停止
				
				//高度控制使用两种模式，不定高模式与定高模式
				if( uxBits & UNUSE_HeightMode_Event )
					FlyControl_unuseHeight += (controlVal.height/6.0f);
				else
				{	
					FlyControl_height += controlVal.height;
					if( FlyControl_height > HeightMode_MAX )
						xTimerStart(priv_OperateFullTimer,0);//操作满限幅提示
				}
				
				//高度下降一定距离,关闭飞行
				if( FlyControl_unuseHeight<=UNHeightMode_Min||FlyControl_height<=HeightMode_Min)
					xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
			}

		}
		else
		{
			/* 控制状态刷新 */
			refresh_cmdstate++;
			if( refresh_cmdstate >= TaskFreq/4 )
			{
				FlyControl_pitch = 0; 
				FlyControl_roll = 0;
				FlyControl_gyroz = 0;

				refresh_cmdstate = (TaskFreq/4) + 1;
				controlCmdNumber = IDLECmd;
			}	
		}
		/* 读取控制指令 END */
		
		/* 低电量检测与处理 */
		static uint32_t lowVOLcount = 0;
		if( (uxBits & StartFly_Event) && g_robotVOL < 9.5f )
		{
			lowVOLcount++;
			if( 2*TaskFreq == lowVOLcount ) //连续2秒电量低于10V
			{   //低电量标记
				xEventGroupSetBits(g_xEventFlyAction,LowPower_Event); 
			}
		}
		else lowVOLcount = 0;
		
		//低电量处理
		if( (uxBits & LowPower_Event) && (uxBits & StartFly_Event) )
		{
			static uint16_t tipscount = 0;
			if( ++tipscount >= TaskFreq ) tipscount=0,xTimerStart(priv_lowpowerTimer,0);
			
			StartFly_SmoothHeightFlag=0;
			
			//自动降低飞行高度
			if( uxBits & UNUSE_HeightMode_Event )
				FlyControl_unuseHeight-= 0.0005f;
			else
				FlyControl_height -= 0.001f;
			
			//高度下降一定距离,关闭飞行
			if( FlyControl_unuseHeight<=UNHeightMode_Min||FlyControl_height<=HeightMode_Min)
				xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
				
		}
		else if( !(uxBits & StartFly_Event) && g_robotVOL > ROBOT_VOL_LIMIT ) 
			xEventGroupClearBits(g_xEventFlyAction,LowPower_Event); 
		/* 低电量检测与处理 END */
		
		//降落时关闭跟随、避障功能
		if( use_distance < 0.5f && controlVal.height<0 )
		{
			xEventGroupClearBits(g_xEventFlyAction,lidar_follow_mode);
			xEventGroupClearBits(g_xEventFlyAction,lidar_avoid_mode);
		}
		
		/* 高度限幅处理 */
		FlyControl_height = target_limit_float(FlyControl_height,HeightMode_Min,HeightMode_MAX);
		FlyControl_unuseHeight = target_limit_float(FlyControl_unuseHeight,UNHeightMode_Min,UNHeightMode_MAX);
		
		/* 平衡控制核心内容 */
		if( (uxBits & StartFly_Event)&&(uxBits & IMU_CalibZeroDone_Event ) )
		{
			if( 1==StarFly_UpdateFlag )
			{
				StarFly_UpdateFlag=0;
				
				/* 启动位置零点标定 */
				zero_point->attitude->yaw += AttitudeVal.yaw;
				imu->UpdateZeroPoint_attitude(zero_point->attitude);
				
				//光流位置
				targetPosX = posX;
				targetPosY = posY;
				
				/* 在启动位置,将所有控制量复位 */
				FlyControl_pitch = 0; FlyControl_roll = 0; FlyControl_gyroz = 0;
				FlyControl_yaw = AttitudeVal.yaw;
				FlyControl_unuseHeight = 0.0f;
				
				//PID控制器清空累计值
				PID_Reset(&RollPID);
				PID_Reset(&RollRatePID);
				PID_Reset(&PitchPID);
				PID_Reset(&PitchRatePID);
				PID_Reset(&YawPID);
				PID_Reset(&YawRatePID);
				PID_Reset(&HeightPID);
				PID_Reset(&HeightSpeedPID);
				//标零后跳过本次数据
				
				//清除避障/跟随的状态
				xEventGroupClearBits(g_xEventFlyAction,lidar_follow_mode);
				xEventGroupClearBits(g_xEventFlyAction,lidar_avoid_mode);
				continue;
			}
			
			//起飞时执行高度平滑控制
			if( 1 == StartFly_SmoothHeightFlag )
			{
				if( FlyControl_height < DefalutHeight ) FlyControl_height += 0.0025f;
				else StartFly_SmoothHeightFlag = 0,startPos=1;
			}
			
			//平衡状态下微调功能(非累加,AttitudeVal实时更新)
			AttitudeVal.pitch += userset_pitch;
			AttitudeVal.roll += userset_roll;
			
			/* 核心算法部分... */
			
			///////////////////// 光流数据处理 ///////////////////////////////////
			static float x_dot_Prev = 0, y_dot_Prev = 0; // 上一次的速度值
			
			// 计算重力分量
			float g = 9.8f;
			float g_x = -g * sin(AttitudeVal.pitch);
			float g_y =  g * sin(AttitudeVal.roll) * cos(AttitudeVal.pitch);
			float g_z =  g * cos(AttitudeVal.roll) * cos(AttitudeVal.pitch);

			// 计算运动加速度(机体坐标系)
			float a_x = axis_9Val.accel.x - g_x;
			float a_y = axis_9Val.accel.y - g_y;
			float a_z = axis_9Val.accel.z - g_z;

			// 坐标变换(机体坐标系 --> 世界坐标系)
			float a_motion_x = a_x * cos(AttitudeVal.pitch) + 
					   a_y * sin(AttitudeVal.roll) * sin(AttitudeVal.pitch) + 
							 a_z * cos(AttitudeVal.roll) * sin(AttitudeVal.pitch);
			float a_motion_y = a_y * cos(AttitudeVal.roll) - 
						   a_z * sin(AttitudeVal.roll);
			
			if( fabs(a_motion_x)<0.15f ) a_motion_x=0;
			if( fabs(a_motion_y)<0.15f ) a_motion_y=0;
			
			// 互补滤波
			x_dot = 0.95f * (x_dot + a_motion_x * 0.005f) + 0.05f * speedX; // 单位：m/s
			y_dot = 0.95f * (y_dot + a_motion_y * 0.005f) + 0.05f * speedY; // 单位：m/s

			// 使用梯形积分法计算位移
			posX += (x_dot + x_dot_Prev) * 0.5f * 0.005f; // 单位：m
			posY += (y_dot + y_dot_Prev) * 0.5f * 0.005f; // 单位：m

			// 积分限幅
			posX = fmaxf(fminf(posX, 5.0f), -5.0f);
			posY = fmaxf(fminf(posY, 5.0f), -5.0f);
			
			x_dot_Prev = x_dot;
			y_dot_Prev = y_dot;
			///////////////////// 光流数据处理 END ///////////////////////////////////
			///// 雷达跟踪障碍物方向 /////
			static float lastErr;
			static float lastYErr;
			if( (LidarFollowRegion.avg_distance <= 800)&& (uxBits&lidar_follow_mode) )
			{
				follow_targetpos_lc307(255);//蜂鸣器提示跟随中
				float err = 0 - LidarFollowRegion.center_angle;
				FlyControl_yaw += ((0.004f*err + 0.004f*(err-lastErr))/57.3f);
				lastErr = err;

				//旋转方向完成后,启用前进方向的pid计算
				if( fabs(err)< 8 )
				{
					float Yerr = 370 - LidarFollowRegion.avg_distance;
					targetPosY -= ((0.002f*Yerr + 0.002f*(Yerr-lastYErr))/1000.0f);
					lastYErr = Yerr;
				}
			}
			///////////// END ////////////////
			
			//位置PD计算
			const float limitPos = 0.35f;
			float controlX = 0.5f * (targetPosX - posX) - 0.7f * x_dot;//前后
			float controlY = 0.5f * (targetPosY - posY) - 0.7f * y_dot;//左右
			
			if( controlX > limitPos ) controlX = limitPos;
			if( controlX <-limitPos ) controlX = -limitPos;
			if( controlY > limitPos ) controlY = limitPos;
			if( controlY <-limitPos ) controlY = -limitPos;
			
			//存在控制量、光流设备不存在或定位功能未开启，忽略光流的数据信息
			if( FlyControl_pitch!=0 || FlyControl_roll!=0 || startPos==0 || g_lost_pos_dev == 1 )
			{
				posX=0;posY=0;
				speedX=0;speedY=0;
				x_dot=0;y_dot=0;
				x_dot_Prev=0;y_dot_Prev=0;
				controlX=0;controlY=0;
				targetPosX=0;targetPosY=0;
			}
						
			//角度---外环3轴控制
			PID_Update(&RollPID,-FlyControl_roll - controlY,AttitudeVal.roll);
			PID_Update(&PitchPID,-FlyControl_pitch + controlX,AttitudeVal.pitch);
			
			//偏航角控制,需要对角度进行归一化处理
			float yaw_error = normalize_radian(FlyControl_yaw - AttitudeVal.yaw);
			PID_Update(&YawPID,yaw_error,0);
			
			PID_Update(&HeightPID,FlyControl_height,use_distance);
			
			//角速度---内环3轴控制
			PID_Update(&RollRatePID,RollPID.output,axis_9Val.gyro.x);
			PID_Update(&PitchRatePID,PitchPID.output,axis_9Val.gyro.y);
			PID_Update(&YawRatePID,YawPID.output,axis_9Val.gyro.z);
			PID_Update(&HeightSpeedPID,HeightPID.output,height_dot);
			
			uint16_t base_throttle = weight_to_throttle((195.0f+0+FlyControl_unuseHeight*100.0f),use_distance); //基础油门值
			motor.A.throttle = base_throttle - RollRatePID.output - PitchRatePID.output + YawRatePID.output + HeightSpeedPID.output;
			motor.B.throttle = base_throttle + RollRatePID.output - PitchRatePID.output - YawRatePID.output + HeightSpeedPID.output;
			motor.C.throttle = base_throttle + RollRatePID.output + PitchRatePID.output + YawRatePID.output + HeightSpeedPID.output;
			motor.D.throttle = base_throttle - RollRatePID.output + PitchRatePID.output - YawRatePID.output + HeightSpeedPID.output;
			
			//角度超出范围，关停电机
			const float stop_angle = angle_to_rad(30);
			if( fabs(AttitudeVal.pitch) >stop_angle || fabs(AttitudeVal.roll) > stop_angle )
			{
				xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);	
			}

		}
		else
		{
			/* 下次启动时,需要标定零点 */
			StarFly_UpdateFlag = 1;
			StartFly_SmoothHeightFlag = 1;
			FlyControl_height = 0;
			
			/* 停转电机 */
			StopVal_SelfRecovery(&MotorStopVal);//用于自恢复电机的dshotCMD设置值.
			memcpy(&motor,&MotorStopVal,sizeof(MOTOR_t));
		}
		/* 平衡控制核心内容 END */
		
		/* 发送油门值到电机 */
		SetPwm(&motor); 

		/* 统计任务运行的运行频率 */
		g_readonly_BalanceTaskFreq = debug->UpdateFreq(&debugPriv);
		
		/* 拷贝部分重要变量到APP进行显示,与控制无关 */
		extern APPShowType_t appshow;
		appshow.pitch = AttitudeVal.pitch;
		appshow.roll = AttitudeVal.roll;
		appshow.yaw = AttitudeVal.yaw;
		appshow.gyrox = axis_9Val.gyro.x;
		appshow.gyroy = axis_9Val.gyro.y;
		appshow.gyroz = axis_9Val.gyro.z;
		appshow.accelx = axis_9Val.accel.x;
		appshow.accely = axis_9Val.accel.y;
		appshow.accelz = axis_9Val.accel.z;
		appshow.balanceTaskFreq = g_readonly_BalanceTaskFreq;
		appshow.height = use_distance;
		//appshow.c_pitch = FlyControl_pitch;
		appshow.c_roll = FlyControl_roll;
		appshow.c_yaw = FlyControl_gyroz;
		appshow.c_height = FlyControl_height;
		appshow.zero_roll = userset_roll;
		appshow.zero_pitch = userset_pitch;
		appshow.posx = posX;
		appshow.posy = posY;
		appshow.targetX = targetPosX;
		appshow.targetY = targetPosY;
		appshow.speedx = x_dot;
		appshow.speedy = y_dot;

		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}


float angle_to_rad(float angle)
{
	return angle*0.0174533f;
}

//static float rad_to_angle(float rad)
//{
//	return rad*57.29578f;
//}

//电机控制函数
static void SetPwm(MOTOR_t* m)
{
	//执行控制时对油门进行限幅
	if( 0 == m->A.Telemetry )
		m->A.throttle = target_limit_s16(m->A.throttle,Dshot_MIN,Dshot_MAX);
	
	if( 0 == m->B.Telemetry )
		m->B.throttle = target_limit_s16(m->B.throttle,Dshot_MIN,Dshot_MAX);
	
	if( 0 == m->C.Telemetry )
		m->C.throttle = target_limit_s16(m->C.throttle,Dshot_MIN,Dshot_MAX);
	
	if( 0 == m->D.Telemetry )
		m->D.throttle = target_limit_s16(m->D.throttle,Dshot_MIN,Dshot_MAX);
	
	//发送油门指令
	pMotorInterface_t motor = &UserDshotMotor;
	motor->set_target(m->A,m->D,m->C,m->B);

//电机编号与四轴飞行器对应关系
/*
	 机头朝向↑↑↑
电机C(顺)     电机B(逆)
         |
         |
-----------------------	
         |
         |
电机D(逆)     电机A(顺)
*/    
}

//限幅函数
static int target_limit_s16(short insert,short low,short high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}


static float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

static void FollwTips_Callback(TimerHandle_t xTimer)
{
	pBuzzerInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(1000);
	tips->off();
}


//操作反馈音
static void OperateResponse_Callback(TimerHandle_t xTimer)
{
	pBuzzerInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(50);
	tips->off();
}

//操作已达到上限反馈音
static void OperateFull_Callback(TimerHandle_t xTimer)
{
	pBuzzerInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(50);
	tips->off();
	vTaskDelay(50);
	tips->on();
	vTaskDelay(50);
	tips->off();
	vTaskDelay(50);
	tips->on();
	vTaskDelay(50);
	tips->off();
}


//标定完成蜂鸣器提示定时器
static void BuzzerTipsTimer_Callback(TimerHandle_t xTimer)
{
	pBuzzerInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(200);
	tips->off();
	vTaskDelay(300);
	for(uint8_t i=0;i<30;i++)
	{
		tips -> toggle();
		vTaskDelay(25);
	}
	tips->off();
}

static void UNUSEHeightTips_Callback(TimerHandle_t xTimer)
{
	pBuzzerInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(200);
	tips->off();
	vTaskDelay(300);
	for(uint8_t i=0;i<5;i++)
	{
		tips -> toggle();
		vTaskDelay(100);
	}
	tips->off();
}

static void LowPowerTips_Callback(TimerHandle_t xTimer)
{
	pBuzzerInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(500);
	tips->off();
}

//LED提示定时器
static void LedTipsTimer_Callback(TimerHandle_t xTimer)
{
	static uint8_t init = 0;
	pLedInterface_t led1 = &UserLed1;
	pLedInterface_t led2 = &UserLed2;
	if( 0 == init )
	{
		init = 1; led1->on(); led2->off();
	}
	
	//标定零点时交替闪烁,标定完后单颗LED闪烁
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( 0 == (uxBits & IMU_CalibZeroDone_Event) ) led1->toggle();
	led2->toggle();
}

//检查高度数据是否稳定
static uint8_t check_HeightStable(float height)
{
	static float last_height = 0 ;
	
	static uint8_t StableThreshold = 0; //稳定阈值
	
	//数据漂移不稳定,或者长时间为0,均代表数据异常
	if( fabs( last_height - height ) > 0.5f || ( last_height == 0 && height == 0 ) )
	{
		if( StableThreshold<255 ) StableThreshold++;
	}
	
	last_height = height;
	
	return StableThreshold;
}

//等待imu数据稳定
static IMU_ZEROPONIT_t* WaitImuStable(uint16_t freq,IMU_DATA_t NowImu,ATTITUDE_DATA_t NowAttitude)
{
	static uint16_t timecore = 0;
	
	//辅助零点标定
	static IMU_DATA_t LastImuData = { 0 };
	static ATTITUDE_DATA_t LastAttitudeData = { 0 };
	
	//用于零点保存
	static IMU_DATA_t ZeroPoint = { 0 };
	static ATTITUDE_DATA_t AttitudeZeroPoint = { 0 };
	static IMU_ZEROPONIT_t res_p = {NULL,NULL};//创建实例
	
	//持续稳定的次数
	static uint8_t stablecount = 0;
	
	//零点标定的次数
	const uint8_t calibratetimes = 5;
	
	static uint8_t step = 0;
	
	uint8_t state = 0;
	timecore++;
	if( timecore >= freq/5 ) //200ms检测1次
	{
		timecore = 0;
		
		if( 0 == step ) //步骤0,标定6轴数据的零点(gyro,accel)
		{
			if( fabs(NowImu.gyro.x - LastImuData.gyro.x) < 0.01f ) state++;
			if( fabs(NowImu.gyro.y - LastImuData.gyro.y) < 0.01f ) state++;
			if( fabs(NowImu.gyro.z - LastImuData.gyro.z) < 0.01f ) state++;
			if( fabs(NowImu.accel.x - LastImuData.accel.x) < 0.06f ) state++;
			if( fabs(NowImu.accel.y - LastImuData.accel.y) < 0.06f ) state++;
			if( fabs(NowImu.accel.z - LastImuData.accel.z) < 0.06f ) state++;
			
			if( state==6 )  
			{
				stablecount++;
				ZeroPoint.gyro.x+=NowImu.gyro.x;
				ZeroPoint.gyro.y+=NowImu.gyro.y;
				ZeroPoint.gyro.z+=NowImu.gyro.z;
				ZeroPoint.accel.x+=NowImu.accel.x;
				ZeroPoint.accel.y+=NowImu.accel.y;
				ZeroPoint.accel.z+=NowImu.accel.z;
				
				//持续的数据稳定,则认为当前数据稳定
				if( stablecount == calibratetimes ) 
				{
					stablecount = 0;//完成6轴数据标定
					step = 1;       //进入步骤1
					
					//取连续稳定期间的数据平均值
					ZeroPoint.gyro.x/=calibratetimes;
					ZeroPoint.gyro.y/=calibratetimes;
					ZeroPoint.gyro.z/=calibratetimes;
					ZeroPoint.accel.x/=calibratetimes;
					ZeroPoint.accel.y/=calibratetimes;
					ZeroPoint.accel.z/=calibratetimes;
					
					ZeroPoint.accel.z-=9.8f;
					res_p.axis = &ZeroPoint;
					
					//输出标定结果：
//					printf("======== step1 ==========\r\n");
//					printf("gx:%.3f\r\n",ZeroPoint.gyro.x);
//					printf("gy:%.3f\r\n",ZeroPoint.gyro.y);
//					printf("gz:%.3f\r\n",ZeroPoint.gyro.z);
//					printf("ax:%.3f\r\n",ZeroPoint.accel.x);
//					printf("ay:%.3f\r\n",ZeroPoint.accel.y);
//					printf("az:%.3f\r\n",ZeroPoint.accel.z);
//					printf("======== step1 ==========\r\n");	
					
					return &res_p;//返回一次,避免数据尚未更新时持续进入下一个步骤
				}
		
			}
			else  //非连续的稳定数据,清除累计数值重新等待标定
			{   
				stablecount=0;
				memset(&ZeroPoint,0,sizeof(IMU_DATA_t));
			}			
			
			//保存上一次数据
			memcpy(&LastImuData,&NowImu,sizeof(IMU_DATA_t));
		}
		
		else if( 1 == step ) //步骤1,标定欧拉角
		{
			if( fabs( NowAttitude.roll - LastAttitudeData.roll ) < 0.002f )   state++;
			if( fabs( NowAttitude.pitch - LastAttitudeData.pitch ) < 0.002f ) state++;
			if( fabs( NowAttitude.yaw - LastAttitudeData.yaw ) < 0.002f )     state++;
			
			if( state==3 )  
			{
				stablecount++;
				AttitudeZeroPoint.pitch+=NowAttitude.pitch;
				AttitudeZeroPoint.yaw+=NowAttitude.yaw;
				AttitudeZeroPoint.roll+=NowAttitude.roll;
				
				//持续的数据稳定,则认为当前数据稳定
				if( stablecount == calibratetimes ) 
				{
					stablecount = 0;//完成6轴数据标定
					step = 2;       //进入步骤1
					
					//取连续稳定期间的数据平均值
					AttitudeZeroPoint.pitch/=calibratetimes;
					AttitudeZeroPoint.yaw/=calibratetimes;
					AttitudeZeroPoint.roll/=calibratetimes;
					
					res_p.attitude = &AttitudeZeroPoint;
					
					//输出标定结果：
//					printf("======== step2 ==========\r\n");
//					printf("pitch:%.3f\r\n",AttitudeZeroPoint.pitch);
//					printf("  yaw:%.3f\r\n",AttitudeZeroPoint.yaw);
//					printf(" roll:%.3f\r\n",AttitudeZeroPoint.roll);
//					printf("======== step2 ==========\r\n");	
				}
			}
			else  //非连续的稳定数据,清除累计数值重新等待标定
			{
				stablecount=0;
				memset(&AttitudeZeroPoint,0,sizeof(ATTITUDE_DATA_t));
			}		
			memcpy(&LastAttitudeData,&NowAttitude,sizeof(ATTITUDE_DATA_t)); //保存上一次的数据
		} /* if( 1 == step ) */
	} /* if( timecore >= freq/5 ) */
	
	return &res_p;
}

static void StopVal_SelfRecovery(MOTOR_t* m)
{
	const uint8_t times = 30;
	static uint8_t Flag_recoveryA = 0,Flag_recoveryB = 0,Flag_recoveryC = 0,Flag_recoveryD = 0;
	
	//当有dshotCMD命令时,连续发布命令 times 次后自动恢复控制指令
	if( 1 == m->A.Telemetry )
	{
		Flag_recoveryA++;
		if( Flag_recoveryA > times ) 
		{
			Flag_recoveryA = 0;
			m->A.Telemetry = 0;
			m->A.throttle = Dshot_MIN;
			xTimerStart(priv_OperateFullTimer,0);//恢复提示音
		}
	}
	
	if( 1 == m->B.Telemetry )
	{
		Flag_recoveryB++;
		if( Flag_recoveryB > times ) 
		{
			Flag_recoveryB = 0;
			m->B.Telemetry = 0;
			m->B.throttle = Dshot_MIN;
			xTimerStart(priv_OperateFullTimer,0);//恢复提示音
		}
	}
	
	if( 1 == m->C.Telemetry )
	{
		Flag_recoveryC++;
		if( Flag_recoveryC > times ) 
		{
			Flag_recoveryC = 0;
			m->C.Telemetry = 0;
			m->C.throttle = Dshot_MIN;
			xTimerStart(priv_OperateFullTimer,0);//恢复提示音
		}
	}
	
	if( 1 == m->D.Telemetry )
	{
		Flag_recoveryD++;
		if( Flag_recoveryD > times ) 
		{
			Flag_recoveryD = 0;
			m->D.Telemetry = 0;
			m->D.throttle = Dshot_MIN;
			xTimerStart(priv_OperateFullTimer,0);//恢复提示音
		}
	}
	
}

//重量转为油门值,入口参数单位克g
static uint16_t weight_to_throttle(float weight,float height)
{
//	weight = weight + height*14.0f;
	
    return 1537.96f + 8.70f*weight - 225.2f*g_robotVOL - 0.0088f*weight*weight - 0.24f*weight*g_robotVOL + 9.01f*g_robotVOL*g_robotVOL;
}

/**************************************************************************
函数功能：向四轴控制队列中写入控制指令,并自动处理控制优先级与数据是否合法等问题
入口参数：需要写入的控制指令地址,操作队列的环境(0任务环境,1 ISR环境),检查是否有更高优先级任务唤醒(仅ISR环境使用)
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void WriteFlyControlQueue(FlyControlType_t val,uint8_t writeEnv,BaseType_t* HigherPriorityTask)
{
	extern QueueHandle_t g_xQueueFlyControl;
	
	//检查指令是否来源于更高优先级
	if( val.source > controlCmdNumber )  controlCmdNumber = val.source;
	
	//优先级匹配,写入控制队列
	if( val.source == controlCmdNumber  )
	{
		if( 1 == writeEnv ) //ISR环境
		{
			xQueueOverwriteFromISR(g_xQueueFlyControl,&val,HigherPriorityTask);
		}
		else //任务环境
		{
			xQueueOverwrite(g_xQueueFlyControl,&val);
		}
	}
}

void BeepTips(void)
{
	xTimerStart(priv_OperateResponseTimer,0);
}

//启动事件(调用环境要求：任务内)
void StartFlyAction(void)
{
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	//如果已经启动,用户再次操作时,则关闭
	if( uxBits&StartFly_Event )
	{
		xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
	}
	
	//启动
	else if( g_robotVOL > ROBOT_VOL_LIMIT && (uxBits & IMU_CalibZeroDone_Event) && !(uxBits & LowPower_Event ) )
	{
		xEventGroupSetBits(g_xEventFlyAction,StartFly_Event);
		xTimerStart(priv_OperateResponseTimer,0);
		if( uxBits&TestMotorMode_Event ) xEventGroupClearBits(g_xEventFlyAction,TestMotorMode_Event);
	}

	else
		xTimerStart(priv_OperateFullTimer,0);
}

//停止事件(调用环境要求：任务内)
void StopFlyAction(void)
{
	xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
	xTimerStart(priv_OperateResponseTimer,0);
}

//不定高模式进入或退出(调用环境要求：任务内)
void FlyAction_EnterUNUSEHeightMode(void)
{
	static uint8_t mode = 0;
	
	mode = !mode;
	if( mode ) xEventGroupSetBits(g_xEventFlyAction,UNUSE_HeightMode_Event),xTimerStart(priv_UNUSEHeightTimer,0);
	else xEventGroupClearBits(g_xEventFlyAction,UNUSE_HeightMode_Event),xTimerStart(priv_OperateResponseTimer,0);
}

//对陀螺仪的零点进行调节
void FlyAction_AdjustDiffAngle(uint8_t changeNum)
{
	const float step = 0.2f;
	        if( 1==changeNum ) userset_pitch += angle_to_rad(step);
	else if( 2 == changeNum ) userset_pitch -= angle_to_rad(step);
	else if( 3 == changeNum ) userset_roll += angle_to_rad(step);
	else if( 4 == changeNum ) userset_roll -= angle_to_rad(step);
	
	xTimerStart(priv_OperateResponseTimer,0);
}

void FlyAction_ChangeLidarAvoidMode(uint8_t id)
{
	xTimerStart(priv_OperateResponseTimer,0);
	if( id==1 ) 
	{	//启动避障
		xEventGroupClearBits(g_xEventFlyAction,lidar_follow_mode);
		xEventGroupSetBits(g_xEventFlyAction,lidar_avoid_mode);
	}
	else if( id==2 )
	{	//启动跟随
		xEventGroupClearBits(g_xEventFlyAction,lidar_avoid_mode);
		xEventGroupSetBits(g_xEventFlyAction,lidar_follow_mode);
	}
	else 
	{
		xEventGroupClearBits(g_xEventFlyAction,lidar_follow_mode);
		xEventGroupClearBits(g_xEventFlyAction,lidar_avoid_mode);
	}
}

//保存用户设定的微调参数
void FlyAction_SaveDiffAngleParam(void)
{
	extern uint8_t User_Flash_SaveParam(uint32_t* data,uint16_t datalen);
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	if( !(uxBits & StartFly_Event) ) //仅在不工作时允许保存参数
	{
		int32_t saveparam[2] = { 0 };
		saveparam[0] = *((int32_t*)&userset_pitch);
		saveparam[1] = *((int32_t*)&userset_roll);
		
		taskENTER_CRITICAL(); //进入临界区,挂起所有任务和中断
		uint8_t res = User_Flash_SaveParam((uint32_t*)saveparam,2);
		taskEXIT_CRITICAL(); //写入完毕,退出临界
		if( 1 == res ) xTimerStart(priv_OperateFullTimer,0);
	}
}

//有头无头模式切换
void FlyAction_HeadLessModeChange(void)
{
	static uint8_t flag = 1;
	
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	if( !(uxBits & StartFly_Event) ) //只支持起飞前设置
	{
		flag = !flag;
		
		//关闭无头模式
		if( flag )
		{
			xEventGroupClearBits(g_xEventFlyAction,FlyMode_HeadLessMode_Event);
			xTimerStart(priv_OperateResponseTimer,0);
		}
		else
		{
			xEventGroupSetBits(g_xEventFlyAction,FlyMode_HeadLessMode_Event);
			xTimerStart(priv_UNUSEHeightTimer,0);
		}
	}

		
}

//复位,环境要求任务内
void ResetSystem(uint8_t isFromISR)
{
	EventBits_t uxBits = 0;
	
	if( isFromISR ) uxBits = xEventGroupGetBitsFromISR(g_xEventFlyAction);
	else            uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	if( !(uxBits&StartFly_Event) )
	{
		NVIC_SystemReset();
	}
	else
	{
		if( isFromISR ) xTimerStartFromISR(priv_OperateFullTimer,0);
		else xTimerStart(priv_OperateFullTimer,0);
	}

}

//进入电机测试模式
void FlyAction_TestMotorMode(void)
{
	static uint8_t flag = 0;
	xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
	xTimerStart(priv_OperateResponseTimer,0);
	
	flag=!flag;
	
	if(flag)
		xEventGroupSetBits(g_xEventFlyAction,TestMotorMode_Event);
	else
		xEventGroupClearBits(g_xEventFlyAction,TestMotorMode_Event);
}

const uint16_t testmotorVal = 160;

#if 1
void TestMotorMode_TestA(uint8_t operateNum)
{
	if( operateNum>2 ) return; //不接收未处理的标号控制
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //操作标号,0号测试电机
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.A.Telemetry = 0;
				MotorStopVal.A.throttle = testmotorVal;
			}
			else
			{
				MotorStopVal.A.Telemetry = 0;
				MotorStopVal.A.throttle = Dshot_MIN;
			}
		}
		else if( 1==operateNum ) //操作标号,1号修改电机的转向
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.A.Telemetry = 1;
				MotorStopVal.A.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.A.Telemetry = 1;
				MotorStopVal.A.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //操作标号,2号保存电机的转向
		{
			MotorStopVal.A.Telemetry = 1;
			MotorStopVal.A.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestB(uint8_t operateNum)
{
	if( operateNum>2 ) return; //不接收未处理的标号控制
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //操作标号,0号测试电机
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.B.Telemetry = 0;
				MotorStopVal.B.throttle = testmotorVal;
			}
			else
			{
				MotorStopVal.B.Telemetry = 0;
				MotorStopVal.B.throttle = Dshot_MIN;
			}
		}
		else if( 1==operateNum ) //操作标号,1号修改电机的转向
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.B.Telemetry = 1;
				MotorStopVal.B.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.B.Telemetry = 1;
				MotorStopVal.B.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //操作标号,2号保存电机的转向
		{
			MotorStopVal.B.Telemetry = 1;
			MotorStopVal.B.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestC(uint8_t operateNum)
{
	if( operateNum>2 ) return; //不接收未处理的标号控制
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //操作标号,0号测试电机
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.C.Telemetry = 0;
				MotorStopVal.C.throttle = testmotorVal;
			}
			else
			{
				MotorStopVal.C.Telemetry = 0;
				MotorStopVal.C.throttle = Dshot_MIN;
			}
		}
		else if( 1==operateNum ) //操作标号,1号修改电机的转向
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.C.Telemetry = 1;
				MotorStopVal.C.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.C.Telemetry = 1;
				MotorStopVal.C.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //操作标号,2号保存电机的转向
		{
			MotorStopVal.C.Telemetry = 1;
			MotorStopVal.C.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestD(uint8_t operateNum)
{
	if( operateNum>2 ) return; //不接收未处理的标号控制
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //操作标号,0号测试电机
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.D.Telemetry = 0;
				MotorStopVal.D.throttle = testmotorVal;
			}
			else
			{
				MotorStopVal.D.Telemetry = 0;
				MotorStopVal.D.throttle = Dshot_MIN;
			}
		}
		else if( 1==operateNum ) //操作标号,1号修改电机的转向
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.D.Telemetry = 1;
				MotorStopVal.D.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.D.Telemetry = 1;
				MotorStopVal.D.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //操作标号,2号保存电机的转向
		{
			MotorStopVal.D.Telemetry = 1;
			MotorStopVal.D.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

#else //电机推力测试方法
void TestMotorMode_TestA(uint8_t operateNum)
{
	if( operateNum>2 ) return; //不接收未处理的标号控制
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //操作标号,0号测试电机
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.A.Telemetry = 0;
				MotorStopVal.A.throttle += 50;
			}
			else
			{
				MotorStopVal.A.Telemetry = 0;
				MotorStopVal.A.throttle += 50;
			}
		}
		else if( 1==operateNum ) //操作标号,1号修改电机的转向
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.A.Telemetry = 1;
				MotorStopVal.A.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.A.Telemetry = 1;
				MotorStopVal.A.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //操作标号,2号保存电机的转向
		{
			MotorStopVal.A.Telemetry = 0;
			MotorStopVal.A.throttle = Dshot_MIN;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestB(uint8_t operateNum)
{
	if( operateNum>2 ) return; //不接收未处理的标号控制
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //操作标号,0号测试电机
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.B.Telemetry = 0;
				MotorStopVal.B.throttle += 50;
			}
			else
			{
				MotorStopVal.B.Telemetry = 0;
				MotorStopVal.B.throttle += 50;
			}
		}
		else if( 1==operateNum ) //操作标号,1号修改电机的转向
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.B.Telemetry = 1;
				MotorStopVal.B.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.B.Telemetry = 1;
				MotorStopVal.B.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //操作标号,2号保存电机的转向
		{
			MotorStopVal.B.Telemetry = 0;
			MotorStopVal.B.throttle = Dshot_MIN;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestC(uint8_t operateNum)
{
	if( operateNum>2 ) return; //不接收未处理的标号控制
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //操作标号,0号测试电机
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.C.Telemetry = 0;
				MotorStopVal.C.throttle += 50;
			}
			else
			{
				MotorStopVal.C.Telemetry = 0;
				MotorStopVal.C.throttle += 50;
			}
		}
		else if( 1==operateNum ) //操作标号,1号修改电机的转向
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.C.Telemetry = 1;
				MotorStopVal.C.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.C.Telemetry = 1;
				MotorStopVal.C.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //操作标号,2号保存电机的转向
		{
			MotorStopVal.C.Telemetry = 0;
			MotorStopVal.C.throttle = Dshot_MIN;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestD(uint8_t operateNum)
{
	if( operateNum>2 ) return; //不接收未处理的标号控制
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //操作标号,0号测试电机
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.D.Telemetry = 0;
				MotorStopVal.D.throttle += 50;
			}
			else
			{
				MotorStopVal.D.Telemetry = 0;
				MotorStopVal.D.throttle += 50;
			}
		}
		else if( 1==operateNum ) //操作标号,1号修改电机的转向
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.D.Telemetry = 1;
				MotorStopVal.D.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.D.Telemetry = 1;
				MotorStopVal.D.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //操作标号,2号保存电机的转向
		{
			MotorStopVal.D.Telemetry = 0;
			MotorStopVal.D.throttle = Dshot_MIN;
		}

	}
	else flag1 = 0 , flag2 = 0;
}
#endif
