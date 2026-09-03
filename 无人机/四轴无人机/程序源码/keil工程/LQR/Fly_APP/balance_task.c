/**
 * @file    balance_task.c
 * @brief   无人机平衡控制任务（核心飞控逻辑）
 * @note    FreeRTOS 任务，运行频率 200Hz
 *          功能：IMU 校准、姿态控制、高度控制、水平位置 LQR 控制、
 *          电机分配矩阵、光流速度融合、激光雷达避障/跟随、
 *          低电量保护、无头模式、电机测试模式
 */

#include "balance_task.h"

/* C 标准库 */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "event_groups.h"

#include "main.h"

/* BSP 驱动 */
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "bsp_ps2.h"
#include "bsp_dshot.h"
#include "bsp_adc.h"
#include "bsp_Rtosdebug.h"
#include "bsp_imu.h"
#include "bsp_stp23L.h"
#include "show_task.h"
#include "lidar_task.h"

/* ==================== 数据结构定义 ==========================================*/

/* 四路电机控制值 */
typedef struct {
	dshotMotorVal_t A;           /* 电机 A（右前，逆时针） */
	dshotMotorVal_t B;           /* 电机 B（左前，顺时针） */
	dshotMotorVal_t C;           /* 电机 C（左后，逆时针） */
	dshotMotorVal_t D;           /* 电机 D（右后，顺时针） */
} MOTOR_t;

/* IMU 校准零偏结构体 */
typedef struct {
	IMU_DATA_t*      axis;       /* 9 轴传感器零偏 */
	ATTITUDE_DATA_t* attitude;   /* 姿态角零偏 */
} IMU_ZEROPONIT_t;

/* 电机停止默认值 */
static MOTOR_t MotorStopVal = {
	.A = {0, Dshot_MIN},
	.B = {0, Dshot_MIN},
	.C = {0, Dshot_MIN},
	.D = {0, Dshot_MIN},
};

/* 电机当前输出值 */
static MOTOR_t motor = {
	.A = {0, Dshot_MIN},
	.B = {0, Dshot_MIN},
	.C = {0, Dshot_MIN},
	.D = {0, Dshot_MIN},
};

/* IMU 数据存储 */
IMU_DATA_t      axis_9Val   = { 0 }; /* 9 轴传感器原始数据 */
ATTITUDE_DATA_t AttitudeVal = { 0 }; /* 姿态角数据 */

/* FreeRTOS 软件定时器 */
static TimerHandle_t priv_BuzzerTipsTimer;       /* 蜂鸣器提示定时器 */
static TimerHandle_t priv_WaitImuTipsTimer;      /* 等待 IMU 校准 LED 提示 */
static TimerHandle_t priv_OperateResponseTimer;  /* 操作响应蜂鸣 */
static TimerHandle_t priv_OperateFullTimer;      /* 操作超限蜂鸣 */
static TimerHandle_t priv_UNUSEHeightTimer;      /* 无高度模式提示 */
static TimerHandle_t priv_lowpowerTimer;         /* 低电量提示 */
static TimerHandle_t follow_tipsTimer;           /* 激光跟随提示 */

/* 飞行控制事件组（与外部共享） */
extern EventGroupHandle_t g_xEventFlyAction;

/* ==================== 内部函数前置声明 ======================================*/

static void SetPwm(MOTOR_t* m);
static IMU_ZEROPONIT_t* WaitImuStable(uint16_t freq, IMU_DATA_t imu, ATTITUDE_DATA_t attitude);
static void Attitude_Controller(const ATTITUDE_DATA_t* attitude, const IMU_DATA_t* imudata,
                                const float Height, const float Height_dot,
                                const float Ts, float motorf[4][1]);
static void BuzzerTipsTimer_Callback(TimerHandle_t xTimer);
static void LedTipsTimer_Callback(TimerHandle_t xTimer);
static void OperateResponse_Callback(TimerHandle_t xTimer);
static void OperateFull_Callback(TimerHandle_t xTimer);
static void UNUSEHeightTips_Callback(TimerHandle_t xTimer);
static void LowPowerTips_Callback(TimerHandle_t xTimer);
static void FollwTips_Callback(TimerHandle_t xTimer);
static int   target_limit_s16(short insert, short low, short high);
static float target_limit_float(float insert, float low, float high);
static void  mul(const int A_row, const int A_col, const int B_row, const int B_col,
                 const float A[A_row][A_col], const float B[B_row][B_col], float C[A_row][B_col]);
static uint8_t check_HeightStable(float height);
static void StopVal_SelfRecovery(MOTOR_t* m);
static float height_to_weight(float height);
static float normalize_radian(float angle);

/* ==================== 全局变量（只读导出） ===================================*/

float g_readonly_BalanceTaskFreq = 0;    /* 平衡任务实际运行频率 (Hz) */

/* 飞行控制目标值 */
#define DefalutHeight 1.0f               /* 起飞默认高度 (m) */
static float FlyControl_pitch    = 0;    /* 俯仰控制输入 */
static float FlyControl_roll     = 0;    /* 横滚控制输入 */
static float FlyControl_yaw      = 0;    /* 偏航控制输入 */
static float FlyControl_gyroz    = 0;    /* Z 轴角速率控制输入 */
static float FlyControl_height   = 0;    /* 高度控制输入（高度模式） */
static float FlyControl_unuseHeight = 0.0f; /* 无高度模式控制输入 */

/* 用户微调角度零偏 */
static float userset_pitch = 0;
static float userset_roll  = 0;

/* 控制命令优先级 */
static uint8_t controlCmdNumber = IDLECmd;

/* 低电量电压阈值 (V) */
#define ROBOT_VOL_LIMIT 11.0f

/* 高度模式限制 */
#define HeightMode_MAX  1.5f
#define HeightMode_Min -0.5f

/* 无高度模式限制 */
#define UNHeightMode_MAX  5.0f
#define UNHeightMode_Min -0.5f

/* 起飞高度平滑标志 */
uint8_t StartFly_SmoothHeightFlag = 1;

/* 水平位置和速度 */
float speedX = 0, speedY = 0;            /* 速度 (m/s) */
float x = 0, y = 0;                      /* 位置 (m) */
float targetPosX = 0, targetPosY = 0;    /* 目标位置 */

/* 定位控制标志 */
uint8_t startPos = 1;

/* 定位设备失联标志（外部定义） */
extern uint8_t g_lost_pos_dev;

/* 陀螺仪低通滤波值 */
float gx_filtered = 0, gy_filtered = 0;

/* 实际使用的高度值 */
float use_distance = 0;

/* 无人机质量 (kg) */
#define mass 0.195f

/* ==================== 辅助函数 ==============================================*/

/**
 * @brief  角度归一化到 [-π, π] 范围
 */
#define M_PI 3.14159265f
static float normalize_radian(float angle) {
	const float TWO_PI = 2.0f * M_PI;
	while (angle > M_PI)       angle -= TWO_PI;
	while (angle < -M_PI)      angle += TWO_PI;
	return angle;
}

/**
 * @brief  角度制转弧度制
 */
float angle_to_rad(float angle)
{
	return angle * 0.0174533f;
}

/**
 * @brief  光流传感器结果回调（速度数据融合入口）
 * @param  buf: [0]=X方向速度, [1]=Y方向速度 (m/s)
 * @note   速度 = 光流原始值 / 200，再扣除机体旋转分量 + 高度补偿
 */
void getOpticalFlowResult_Callback(float* buf)
{
	speedY = -buf[0] / 200.0f;
	speedX = -buf[1] / 200.0f;
	/* 旋转补偿 + 高度恢复 */
	speedY = use_distance * (speedY - fmaxf(fminf(gx_filtered, 2.0f), -2.0f));
	speedX = use_distance * (speedX + fmaxf(fminf(gy_filtered, 2.0f), -2.0f));
	g_lost_pos_dev = 0;
}

/**
 * @brief  激光雷达避障模式：调整目标位置
 * @param  flag: 方向（0=右, 1=左, 2=前, 3=后）
 */
void avoid_targetpos_lc307(uint8_t flag)
{
	xTimerStart(priv_OperateFullTimer, 0);
	switch (flag)
	{
		case 0: targetPosX += 0.03f; break; /* 右移 */
		case 1: targetPosX -= 0.03f; break; /* 左移 */
		case 2: targetPosY += 0.03f; break; /* 前移 */
		case 3: targetPosY -= 0.03f; break; /* 后移 */
	}
}

/**
 * @brief  激光雷达跟随模式：微调目标位置
 * @param  flag: 方向（近距离/远距离各有 4 个方向）
 */
void follow_targetpos_lc307(uint8_t flag)
{
	static TickType_t LastTick;
	TickType_t now = xTaskGetTickCount();
	if (now - LastTick >= 1300)             /* 1.3s 间隔蜂鸣提示 */
	{
		xTimerStart(follow_tipsTimer, 0);
		LastTick = now;
	}

	switch (flag)
	{
		/* 近距离微调 */
		case 0: targetPosX -= 0.007f; break;
		case 1: targetPosX += 0.007f; break;
		case 2: targetPosY -= 0.007f; break;
		case 3: targetPosY += 0.007f; break;
		/* 远距离微调 */
		case 4: targetPosX += 0.002f; break;
		case 5: targetPosX -= 0.002f; break;
		case 6: targetPosY += 0.002f; break;
		case 7: targetPosY -= 0.002f; break;
	}
}

/**
 * @brief  PS2 手柄设定目标位置（大步长）
 */
void set_targetpos_lc307(uint8_t flag)
{
	xTimerStart(priv_OperateResponseTimer, 0);
	switch (flag)
	{
		case 0: targetPosX += 0.2f; break;  /* B 键：X+ */
		case 1: targetPosX -= 0.2f; break;  /* X 键：X- */
		case 2: targetPosY += 0.2f; break;  /* Y 键：Y+ */
		case 3: targetPosY -= 0.2f; break;  /* A 键：Y- */
	}
}

/**
 * @brief  启停定位保持功能
 */
void start_lc307_pos(void)
{
	startPos = !startPos;
}

/* ==================== 电机控制 ==============================================*/

/**
 * @brief  四路电机油门输出（含限幅和分配矩阵）
 * @param  m: 四路电机控制值结构体
 * @note   电机布局（十字型 X 构型）:
 *           电机C(逆)     电机B(顺)
 *                \         /
 *                 --------
 *                /         \
 *           电机D(顺)     电机A(逆)
 *         DShot 命令: C → B → D → A 顺序发送
 */
static void SetPwm(MOTOR_t* m)
{
	/* 非 DShot 命令模式下，限幅油门值 */
	if (0 == m->A.Telemetry) m->A.throttle = target_limit_s16(m->A.throttle, Dshot_MIN, Dshot_MAX);
	if (0 == m->B.Telemetry) m->B.throttle = target_limit_s16(m->B.throttle, Dshot_MIN, Dshot_MAX);
	if (0 == m->C.Telemetry) m->C.throttle = target_limit_s16(m->C.throttle, Dshot_MIN, Dshot_MAX);
	if (0 == m->D.Telemetry) m->D.throttle = target_limit_s16(m->D.throttle, Dshot_MIN, Dshot_MAX);

	/* 发送 DShot 命令（顺序: A → D → C → B） */
	pMotorInterface_t motor = &UserDshotMotor;
	motor->set_target(m->A, m->D, m->C, m->B);
}

/**
 * @brief  int16 限幅函数
 */
static int target_limit_s16(short insert, short low, short high)
{
	if (insert < low)       return low;
	else if (insert > high) return high;
	else                    return insert;
}

/**
 * @brief  float 限幅函数
 */
static float target_limit_float(float insert, float low, float high)
{
	if (insert < low)       return low;
	else if (insert > high) return high;
	else                    return insert;
}

/* ==================== 高度-推力转换 =========================================*/

/**
 * @brief  根据地效高度计算额外推力补偿
 * @param  height: 当前离地高度 (m)
 * @retval 所需总推力 (N)
 * @note   基础推力 = 质量 × 9.8 (悬停需抵消重力)
 *          地效补偿: 每 10cm 高度减少约 14g 的地面效应额外升力
 */
static float height_to_weight(float height)
{
	const float base_weight = mass * 9.8f; /* 基础悬停推力 */
	if (height < 0) height = 0;
	return base_weight + (14.0f * height) / 100.0f;
}

/* ==================== 矩阵乘法 ==============================================*/

/**
 * @brief  通用矩阵乘法 C = A × B
 */
static void mul(const int A_row, const int A_col, const int B_row, const int B_col,
                const float A[A_row][A_col], const float B[B_row][B_col], float C[A_row][B_col])
{
	if (A_col == B_row)
	{
		for (int i = 0; i < A_row; i++)
		{
			for (int j = 0; j < B_col; j++)
			{
				C[i][j] = 0;
				for (int k = 0; k < A_col; k++)
				{
					C[i][j] += A[i][k] * B[k][j];
				}
			}
		}
	}
}

/* ==================== 蜂鸣器/指示灯回调 =====================================*/

static void FollwTips_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on(); vTaskDelay(1000); tips->off();
}

static void OperateResponse_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on(); vTaskDelay(50); tips->off();
}

void BeepTips(void)
{
	xTimerStart(priv_OperateResponseTimer, 0);
}

static void OperateFull_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	/* 三短鸣：控制已达限幅 */
	tips->on(); vTaskDelay(50); tips->off(); vTaskDelay(50);
	tips->on(); vTaskDelay(50); tips->off(); vTaskDelay(50);
	tips->on(); vTaskDelay(50); tips->off();
}

/* 校准完成蜂鸣: 1长+30次快速交替 */
static void BuzzerTipsTimer_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on(); vTaskDelay(200); tips->off(); vTaskDelay(300);
	for (uint8_t i = 0; i < 30; i++) { tips->toggle(); vTaskDelay(25); }
	tips->off();
}

/* 无高度模式提示: 1长+5次交替 */
static void UNUSEHeightTips_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on(); vTaskDelay(200); tips->off(); vTaskDelay(300);
	for (uint8_t i = 0; i < 5; i++) { tips->toggle(); vTaskDelay(100); }
	tips->off();
}

/* 低电量提示: 长时间鸣响 */
static void LowPowerTips_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on(); vTaskDelay(500); tips->off();
}

/* LED 状态指示: 校准中 LED1 闪烁, LED2 持续闪烁 */
static void LedTipsTimer_Callback(TimerHandle_t xTimer)
{
	static uint8_t init = 0;
	pLedInterface_t led1 = &UserLed1, led2 = &UserLed2;
	if (0 == init) { init = 1; led1->on(); led2->off(); }
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (0 == (uxBits & IMU_CalibZeroDone_Event)) led1->toggle(); /* 未校准: LED1 闪烁 */
	led2->toggle(); /* LED2 持续闪烁表示系统运行 */
}

/* ==================== 高度稳定性检测 ========================================*/

/**
 * @brief  检测激光测距高度是否稳定
 * @param  height: 当前高度值
 * @retval 不稳定计数值（>6 表示异常，进入无高度模式）
 */
static uint8_t check_HeightStable(float height)
{
	static float last_height = 0;
	static uint8_t StableThreshold = 0;

	/* 高度跳变 > 0.5m 或持续为 0，认为传感器异常 */
	if (fabs(last_height - height) > 0.5f || (last_height == 0 && height == 0))
	{
		if (StableThreshold < 255) StableThreshold++;
	}
	last_height = height;
	return StableThreshold;
}

/* ==================== IMU 校准流程 ==========================================*/

/**
 * @brief  等待 IMU 稳定并执行校准
 * @param  freq:        调用频率 (Hz)
 * @param  NowImu:      当前 IMU 数据
 * @param  NowAttitude: 当前姿态数据
 * @retval 包含零偏值的结构体指针
 * @note   两阶段校准：
 *         Step 0: 校准陀螺仪零点（需连续 5 次 200ms 间隔稳定，阈值 0.01 rad/s）
 *         Step 1: 校准姿态角零点（需连续 5 次 200ms 间隔稳定，阈值 0.002 rad）
 */
static IMU_ZEROPONIT_t* WaitImuStable(uint16_t freq, IMU_DATA_t NowImu, ATTITUDE_DATA_t NowAttitude)
{
	/* ... 校准逻辑 ... */
	static uint16_t timecore = 0;
	static IMU_DATA_t      LastImuData      = { 0 };
	static ATTITUDE_DATA_t LastAttitudeData = { 0 };
	static IMU_DATA_t      ZeroPoint        = { 0 };
	static ATTITUDE_DATA_t AttitudeZeroPoint = { 0 };
	static IMU_ZEROPONIT_t res_p = {NULL, NULL};
	static uint8_t stablecount = 0;
	const uint8_t calibratetimes = 5;
	static uint8_t step = 0;
	uint8_t state = 0;

	timecore++;
	if (timecore >= freq / 5)               /* 200ms 执行一次检测 */
	{
		timecore = 0;

		if (0 == step)                       /* Step 0: 校准陀螺仪零点 */
		{
			if (fabs(NowImu.gyro.x - LastImuData.gyro.x) < 0.01f) state++;
			if (fabs(NowImu.gyro.y - LastImuData.gyro.y) < 0.01f) state++;
			if (fabs(NowImu.gyro.z - LastImuData.gyro.z) < 0.01f) state++;

			if (state == 3)
			{
				stablecount++;
				ZeroPoint.gyro.x += NowImu.gyro.x;
				ZeroPoint.gyro.y += NowImu.gyro.y;
				ZeroPoint.gyro.z += NowImu.gyro.z;

				if (stablecount == calibratetimes)
				{
					stablecount = 0; step = 1;
					ZeroPoint.gyro.x /= calibratetimes;
					ZeroPoint.gyro.y /= calibratetimes;
					ZeroPoint.gyro.z /= calibratetimes;
					res_p.axis = &ZeroPoint;
					return &res_p;           /* 先返回陀螺仪校准结果 */
				}
			}
			else
			{
				stablecount = 0;
				memset(&ZeroPoint, 0, sizeof(IMU_DATA_t));
			}
			memcpy(&LastImuData, &NowImu, sizeof(IMU_DATA_t));
		}
		else if (1 == step)                  /* Step 1: 校准姿态角零点 */
		{
			if (fabs(NowAttitude.roll  - LastAttitudeData.roll)  < 0.002f) state++;
			if (fabs(NowAttitude.pitch - LastAttitudeData.pitch) < 0.002f) state++;
			if (fabs(NowAttitude.yaw   - LastAttitudeData.yaw)   < 0.002f) state++;

			if (state == 3)
			{
				stablecount++;
				AttitudeZeroPoint.pitch += NowAttitude.pitch;
				AttitudeZeroPoint.yaw   += NowAttitude.yaw;
				AttitudeZeroPoint.roll  += NowAttitude.roll;

				if (stablecount == calibratetimes)
				{
					stablecount = 0; step = 2;
					AttitudeZeroPoint.pitch /= calibratetimes;
					AttitudeZeroPoint.yaw   /= calibratetimes;
					AttitudeZeroPoint.roll  /= calibratetimes;
					res_p.attitude = &AttitudeZeroPoint;
				}
			}
			else
			{
				stablecount = 0;
				memset(&AttitudeZeroPoint, 0, sizeof(ATTITUDE_DATA_t));
			}
			memcpy(&LastAttitudeData, &NowAttitude, sizeof(ATTITUDE_DATA_t));
		}
	}

	return &res_p;
}

/**
 * @brief  DShot 命令自动恢复机制
 * @note   DShot 命令发送后约 30 个控制周期自动恢复为正常油门模式
 */
static void StopVal_SelfRecovery(MOTOR_t* m)
{
	const uint8_t times = 30;
	static uint8_t Flag_recoveryA = 0, Flag_recoveryB = 0;
	static uint8_t Flag_recoveryC = 0, Flag_recoveryD = 0;

	if (1 == m->A.Telemetry) { if (++Flag_recoveryA > times)
		{ Flag_recoveryA = 0; m->A.Telemetry = 0; m->A.throttle = Dshot_MIN; xTimerStart(priv_OperateFullTimer, 0); } }
	if (1 == m->B.Telemetry) { if (++Flag_recoveryB > times)
		{ Flag_recoveryB = 0; m->B.Telemetry = 0; m->B.throttle = Dshot_MIN; xTimerStart(priv_OperateFullTimer, 0); } }
	if (1 == m->C.Telemetry) { if (++Flag_recoveryC > times)
		{ Flag_recoveryC = 0; m->C.Telemetry = 0; m->C.throttle = Dshot_MIN; xTimerStart(priv_OperateFullTimer, 0); } }
	if (1 == m->D.Telemetry) { if (++Flag_recoveryD > times)
		{ Flag_recoveryD = 0; m->D.Telemetry = 0; m->D.throttle = Dshot_MIN; xTimerStart(priv_OperateFullTimer, 0); } }
}

/* ==================== LQR 姿态+位置控制器 ===================================*/

/* LQR 状态反馈系数（位置环） */
static const float kp = 0.6986f;
static const float kd = 0.4901f;

/* LQR 状态反馈系数（姿态+高度环） */
static const float k11 = 0.5704f;  /* 高度比例 */
static const float k12 = 0.9344f;  /* 高度速度比例 */
static const float k21 = 0.0668f;  /* 横滚角比例 */
static const float k22 = 0.0113f;  /* 横滚角速率比例 */
static const float k31 = 0.0668f;  /* 俯仰角比例 */
static const float k32 = 0.0113f;  /* 俯仰角速率比例 */
static const float k41 = 0.0300f;  /* 偏航角比例 */
static const float k42 = 0.0111f;  /* 偏航角速率比例 */

/**
 * @brief  LQR 姿态控制器（全状态反馈 + 前馈补偿）
 * @param  attitude:   当前姿态角 (rad)
 * @param  imudata:    当前 IMU 数据
 * @param  Height:     当前高度 (m)
 * @param  Height_dot: 当前高度变化率 (m/s)
 * @param  Ts:         控制周期 (s)
 * @param  motorf:     输出四路电机推力矩阵
 * @note   控制分配矩阵 T: 将 [总推力, 横滚力矩, 俯仰力矩, 偏航力矩]
 *         映射到四路电机推力
 */
static void Attitude_Controller(const ATTITUDE_DATA_t* attitude, const IMU_DATA_t* imudata,
                                const float Height, const float Height_dot,
                                const float Ts, float motorf[4][1])
{
	float u[4][1] = {0};
	/* 控制分配矩阵（电机布局相关） */
	float T[4][4] = {
		{0.25f, -3.9284f, -3.9284f,  50.2828f},
		{0.25f,  3.9284f, -3.9284f, -50.2828f},
		{0.25f,  3.9284f,  3.9284f,  50.2828f},
		{0.25f, -3.9284f,  3.9284f, -50.2828f}
	};

	/* ===== 加速度积分获得速度与位置 ===== */
	static float x_dot_Prev = 0, y_dot_Prev = 0;
	static float x_dot = 0, y_dot = 0;

	/* 重力在机体坐标系的分量 */
	float g = 9.8f;
	float g_x = -g * sin(attitude->pitch);
	float g_y =  g * sin(attitude->roll) * cos(attitude->pitch);
	float g_z =  g * cos(attitude->roll) * cos(attitude->pitch);

	/* 运动加速度（扣除重力） */
	float a_x = imudata->accel.x - g_x;
	float a_y = imudata->accel.y - g_y;
	float a_z = imudata->accel.z - g_z;

	/* 机体坐标系 → 世界坐标系旋转 */
	float a_motion_x = a_x * cos(attitude->pitch) +
	                   a_y * sin(attitude->roll) * sin(attitude->pitch) +
	                   a_z * cos(attitude->roll) * sin(attitude->pitch);
	float a_motion_y = a_y * cos(attitude->roll) -
	                   a_z * sin(attitude->roll);

	/* 互补滤波融合：95% 加速度积分 + 5% 光流观测 */
	x_dot = 0.95f * (x_dot + a_motion_x * Ts) + 0.05f * speedX;
	y_dot = 0.95f * (y_dot + a_motion_y * Ts) + 0.05f * speedY;

	/* 梯形积分求位置 */
	x += (x_dot + x_dot_Prev) * 0.5f * Ts;
	y += (y_dot + y_dot_Prev) * 0.5f * Ts;

	/* 位置限幅 ±5m */
	x = fmaxf(fminf(x, 5.0f), -5.0f);
	y = fmaxf(fminf(y, 5.0f), -5.0f);

	/* 离地高度 < 0.2m 时清零位置估计（地面效应不可靠） */
	if (Height < 0.2f) x = x_dot = 0, y = y_dot = 0;

	x_dot_Prev = x_dot;
	y_dot_Prev = y_dot;

	/* ===== LQR 位置控制 → 目标姿态角 ===== */
	float pitch_ref = -(kp * (x - targetPosX) + kd * x_dot);
	float roll_ref  =  (kp * (y - targetPosY) + kd * y_dot);

	/* 目标角度限幅 ±0.2rad (约 ±11.5°) */
	pitch_ref = fmaxf(fminf(pitch_ref, 0.2f), -0.2f);
	roll_ref  = fmaxf(fminf(roll_ref,  0.2f), -0.2f);

	/* 手动控制/定位丢失时清零位置控制项 */
	if (FlyControl_roll != 0 || FlyControl_pitch != 0 || startPos == 0 || g_lost_pos_dev == 1)
	{
		pitch_ref = x = speedX = x_dot = x_dot_Prev = 0;
		roll_ref  = y = speedY = y_dot = y_dot_Prev = 0;
		targetPosX = targetPosY = 0;
	}

	/* ===== LQR 全状态反馈控制 ===== */
	u[0][0] = -(k11 * (Height - FlyControl_height) + k12 * Height_dot)
	          + height_to_weight(FlyControl_height) + FlyControl_unuseHeight;
	u[1][0] = -(k21 * (attitude->roll  + FlyControl_roll  - roll_ref)  + k22 * imudata->gyro.x);
	u[2][0] = -(k31 * (attitude->pitch + FlyControl_pitch - pitch_ref) + k32 * imudata->gyro.y);
	u[3][0] = -(k41 * (attitude->yaw   - FlyControl_yaw)               + k42 * (imudata->gyro.z + FlyControl_gyroz));

	/* 前馈微分补偿（增强响应速度） */
	static float u10Prev = 0, u20Prev = 0, u30Prev = 0;
	u[1][0] += (u[1][0] - u10Prev) * 0.003f; u10Prev = u[1][0];
	u[2][0] += (u[2][0] - u20Prev) * 0.003f; u20Prev = u[2][0];
	u[3][0] += (u[3][0] - u30Prev) * 0.003f; u30Prev = u[3][0];

	/* 控制分配：u → 四路电机推力 */
	mul(4, 4, 4, 1, (const float (*)[4])T, (const float (*)[1])u, motorf);
}

/* ==================== 平衡控制主任务 ========================================*/

/**
 * @brief  平衡控制 FreeRTOS 任务入口（200Hz 固定频率）
 * @note   主要流程:
 *         1. IMU 校准（前 5 秒静态校准陀螺仪和姿态角）
 *         2. 高度数据滤波
 *         3. 接收遥控指令（PS2手柄/蓝牙）
 *         4. 姿态+高度+位置 LQR 控制
 *         5. 电机油门分配输出
 *         6. 安全保护（低电量/角度过大 → 自动停机）
 */
void balance_task(void* param)
{
	extern QueueHandle_t g_xQueueFlyControl;
	TickType_t preTime = xTaskGetTickCount();

	const uint16_t TaskFreq = 200;       /* 控制频率 200Hz */
	const float    Ts       = 0.005f;    /* 控制周期 5ms */

	/* 调试工具 */
	pRtosDebugInterface_t debug = &RTOSTaskDebug;
	RtosDebugPrivateVar debugPriv = { 0 };

	/* IMU 接口 */
	pIMUInterface_t imu = &UserICM20948;
	IMU_ZEROPONIT_t* zero_point = { NULL };

	uint16_t syscount = 0;               /* 系统运行计时（用于开机校准延迟） */

	/* 创建软件定时器 */
	priv_BuzzerTipsTimer     = xTimerCreate("BuzzerTips",    pdMS_TO_TICKS(10),  pdFALSE, NULL, BuzzerTipsTimer_Callback);
	priv_WaitImuTipsTimer    = xTimerCreate("WaitImuTips",   pdMS_TO_TICKS(100), pdTRUE,  NULL, LedTipsTimer_Callback);
	priv_OperateResponseTimer = xTimerCreate("OpResponse",   pdMS_TO_TICKS(100), pdFALSE, NULL, OperateResponse_Callback);
	priv_OperateFullTimer    = xTimerCreate("OpFull",        pdMS_TO_TICKS(100), pdFALSE, NULL, OperateFull_Callback);
	priv_UNUSEHeightTimer    = xTimerCreate("UnUseHeight",   pdMS_TO_TICKS(100), pdFALSE, NULL, UNUSEHeightTips_Callback);
	priv_lowpowerTimer       = xTimerCreate("LowPower",      pdMS_TO_TICKS(100), pdFALSE, NULL, LowPowerTips_Callback);
	follow_tipsTimer         = xTimerCreate("followtips",    pdMS_TO_TICKS(1),   pdFALSE, NULL, FollwTips_Callback);
	xTimerStart(priv_WaitImuTipsTimer, 0);

	EventBits_t uxBits;
	float zero_distance = 0;             /* 起飞时的零高度基准 */
	uint8_t StarFly_UpdateFlag = 1;      /* 起飞初始化标志 */
	static float Motor_f[4][1] = {0};    /* 四路电机推力分配结果 */

	portTASK_USES_FLOATING_POINT();

	/* 从 Flash 读取用户校准参数 */
	extern float g_userparam_pitchzero, g_userparam_rollzero;
	userset_pitch = g_userparam_pitchzero;
	userset_roll  = g_userparam_rollzero;

	while (1)
	{
		/* 高度计算 */
		use_distance = zero_distance - g_readonly_distance;

		/* 读取 IMU 数据（耗时约 0.61ms） */
		imu->Update_9axisVal(&axis_9Val);

		/* 陀螺仪低通滤波 */
		gx_filtered = 0.9f * gx_filtered + 0.1f * axis_9Val.gyro.x;
		gy_filtered = 0.9f * gy_filtered + 0.1f * axis_9Val.gyro.y;

		/* Mahony 姿态解算 */
		imu->UpdateAttitude(axis_9Val, &AttitudeVal);
		AttitudeVal.yaw = normalize_radian(AttitudeVal.yaw);

		/* 系统运行计时（30 秒计满） */
		if (syscount < TaskFreq * 30) syscount++;

		uxBits = xEventGroupGetBits(g_xEventFlyAction);

		/* ===== IMU 自动校准流程 ===== */
		if (0 == (uxBits & IMU_CalibZeroDone_Event))
		{
			if (syscount >= TaskFreq * 5)  /* 开机 5 秒后开始校准 */
			{
				zero_point = WaitImuStable(TaskFreq, axis_9Val, AttitudeVal);
				uint8_t heightstatble = check_HeightStable(g_readonly_distance);

				if (zero_point->axis != NULL)
					imu->UpdateZeroPoint_axis(zero_point->axis);

				if (zero_point->attitude != NULL)
				{
					imu->UpdateZeroPoint_attitude(zero_point->attitude);
					zero_distance = g_readonly_distance;             /* 记录基准高度 */
					xTimerChangePeriod(priv_WaitImuTipsTimer, pdMS_TO_TICKS(800), 0);
					xEventGroupSetBits(g_xEventFlyAction, IMU_CalibZeroDone_Event);

					if (heightstatble >= 6)                          /* 高度传感器异常 */
					{
						xEventGroupSetBits(g_xEventFlyAction, UNUSE_HeightMode_Event);
						xTimerStart(priv_UNUSEHeightTimer, 0);
					}
					else
						xTimerStart(priv_BuzzerTipsTimer, 0);        /* 校准完成提示 */
				}
			}
		}

		/* ===== 高度数据滤波 ===== */
		static float heightPrev = 0, height_dot = 0, height_dotPrev = 0;
		height_dot = 0.4f * (use_distance - heightPrev) / Ts + 0.6f * height_dotPrev;
		if (fabs(use_distance - heightPrev) > 0.1f)                  /* 高度跳变重置 */
			{ heightPrev = use_distance; height_dot = 0; }
		else heightPrev = use_distance;
		height_dotPrev = height_dot;

		if (uxBits & UNUSE_HeightMode_Event)                         /* 无高度模式 */
			{ use_distance = FlyControl_height; height_dot = 0; startPos = 0; }

		/* ===== 接收遥控指令（PS2/蓝牙 → 飞控队列） ===== */
		FlyControlType_t controlVal = { 0 };
		static uint8_t refresh_cmdstate = 0;
		if (pdPASS == xQueueReceive(g_xQueueFlyControl, &controlVal, 0))
		{
			refresh_cmdstate = 0;

			/* 无头模式：将遥控器坐标系旋转到机体坐标系 */
			if (uxBits & FlyMode_HeadLessMode_Event)
			{
				FlyControl_pitch = -controlVal.roll * sin(AttitudeVal.yaw) + controlVal.pitch * cos(AttitudeVal.yaw);
				FlyControl_roll  =  controlVal.roll * cos(AttitudeVal.yaw) + controlVal.pitch * sin(AttitudeVal.yaw);
			}
			else
			{
				FlyControl_pitch = controlVal.pitch;
				FlyControl_roll  = controlVal.roll;
			}
			FlyControl_gyroz = controlVal.gyroz;

			/* Z 轴旋转时锁定偏航目标 */
			if (FlyControl_gyroz != 0) FlyControl_yaw = AttitudeVal.yaw;

			/* 高度控制 */
			if (!(uxBits & LowPower_Event) || controlVal.height <= 0)
			{
				if (fabs(controlVal.height) != 0)
					StartFly_SmoothHeightFlag = 0;

				if (uxBits & UNUSE_HeightMode_Event)
					FlyControl_unuseHeight += (controlVal.height / 6.0f);
				else
				{
					FlyControl_height += controlVal.height;
					if (FlyControl_height > HeightMode_MAX)
						xTimerStart(priv_OperateFullTimer, 0);
				}

				/* 高度下降到底，自动停机 */
				if (FlyControl_unuseHeight <= UNHeightMode_Min || FlyControl_height <= HeightMode_Min)
					xEventGroupClearBits(g_xEventFlyAction, StartFly_Event);
			}
		}
		else
		{
			/* 遥控信号丢失：0.25 秒后回中 */
			refresh_cmdstate++;
			if (refresh_cmdstate >= TaskFreq / 4)
			{
				FlyControl_pitch = 0; FlyControl_roll = 0; FlyControl_gyroz = 0;
				refresh_cmdstate = (TaskFreq / 4) + 1;
				controlCmdNumber = IDLECmd;
			}
		}

		/* ===== 低电量保护 ===== */
		static uint32_t lowVOLcount = 0;
		if ((uxBits & StartFly_Event) && g_robotVOL < 10.0f)
		{
			lowVOLcount++;
			if (2 * TaskFreq == lowVOLcount)     /* 持续 2 秒低电压 → 触发低电量事件 */
				xEventGroupSetBits(g_xEventFlyAction, LowPower_Event);
		}
		else lowVOLcount = 0;

		if ((uxBits & LowPower_Event) && (uxBits & StartFly_Event))
		{
			static uint16_t tipscount = 0;
			if (++tipscount >= TaskFreq) tipscount = 0, xTimerStart(priv_lowpowerTimer, 0);
			StartFly_SmoothHeightFlag = 0;

			/* 自动缓慢下降 */
			if (uxBits & UNUSE_HeightMode_Event)
				FlyControl_unuseHeight -= 0.0005f;
			else
				FlyControl_height -= 0.001f;

			if (FlyControl_unuseHeight <= UNHeightMode_Min || FlyControl_height <= HeightMode_Min)
				xEventGroupClearBits(g_xEventFlyAction, StartFly_Event);
		}
		else if (!(uxBits & StartFly_Event) && g_robotVOL > ROBOT_VOL_LIMIT)
			xEventGroupClearBits(g_xEventFlyAction, LowPower_Event);

		/* 低高度时关闭避障和跟随模式 */
		if (use_distance < 0.5f && controlVal.height < 0)
		{
			xEventGroupClearBits(g_xEventFlyAction, lidar_follow_mode);
			xEventGroupClearBits(g_xEventFlyAction, lidar_avoid_mode);
		}

		/* 高度限幅 */
		FlyControl_height      = target_limit_float(FlyControl_height,      HeightMode_Min,   HeightMode_MAX);
		FlyControl_unuseHeight = target_limit_float(FlyControl_unuseHeight, UNHeightMode_Min, UNHeightMode_MAX);

		/* ===== 起飞/飞行控制 ===== */
		if ((uxBits & StartFly_Event) && (uxBits & IMU_CalibZeroDone_Event))
		{
			if (1 == StarFly_UpdateFlag)         /* 起飞初始化 */
			{
				StarFly_UpdateFlag = 0;
				zero_point->attitude->yaw += AttitudeVal.yaw;
				imu->UpdateZeroPoint_attitude(zero_point->attitude);
				FlyControl_pitch = FlyControl_roll = FlyControl_gyroz = 0;
				FlyControl_unuseHeight = 0.0f;
				xEventGroupClearBits(g_xEventFlyAction, lidar_follow_mode);
				xEventGroupClearBits(g_xEventFlyAction, lidar_avoid_mode);
				continue;
			}

			/* 起飞高度平滑过渡 */
			if (1 == StartFly_SmoothHeightFlag)
			{
				if (FlyControl_height < DefalutHeight) FlyControl_height += 0.001f;
				else StartFly_SmoothHeightFlag = 0, startPos = 1;
			}

			/* 用户微调角偏 */
			AttitudeVal.pitch += userset_pitch;
			AttitudeVal.roll  += userset_roll;

			/* 激光雷达跟随模式 */
			static float lastErr, lastYErr;
			if ((LidarFollowRegion.avg_distance <= 850) && (uxBits & lidar_follow_mode))
			{
				follow_targetpos_lc307(255);
				float err = 0 - LidarFollowRegion.center_angle;
				if (fabs(err) > 8)
					FlyControl_gyroz = -((0.7f * err + 0.2f * (err - lastErr)) / 57.3f);
				else
				{
					FlyControl_gyroz = 0;
					float Yerr = 340 - LidarFollowRegion.avg_distance;
					targetPosY -= ((0.003f * Yerr + 0.002f * (Yerr - lastYErr)) / 1000.0f);
					lastYErr = Yerr;
				}
				if (FlyControl_gyroz != 0) FlyControl_yaw = AttitudeVal.yaw;
			}

			/* LQR 控制器 */
			Attitude_Controller(&AttitudeVal, &axis_9Val, use_distance, height_dot, Ts, Motor_f);

			/* 推力 → DShot 油门值（三次多项式拟合电机特性曲线） */
			motor.A.throttle = 3481.0f*Motor_f[0][0]*Motor_f[0][0]*Motor_f[0][0] - 3864.0f*Motor_f[0][0]*Motor_f[0][0] + 2774.0f*Motor_f[0][0] + 106.0f;
			motor.B.throttle = 3481.0f*Motor_f[1][0]*Motor_f[1][0]*Motor_f[1][0] - 3864.0f*Motor_f[1][0]*Motor_f[1][0] + 2774.0f*Motor_f[1][0] + 106.0f;
			motor.C.throttle = 3481.0f*Motor_f[2][0]*Motor_f[2][0]*Motor_f[2][0] - 3864.0f*Motor_f[2][0]*Motor_f[2][0] + 2774.0f*Motor_f[2][0] + 106.0f;
			motor.D.throttle = 3481.0f*Motor_f[3][0]*Motor_f[3][0]*Motor_f[3][0] - 3864.0f*Motor_f[3][0]*Motor_f[3][0] + 2774.0f*Motor_f[3][0] + 106.0f;

			/* 安全保护：倾角超过 30° 自动停机 */
			if (AttitudeVal.pitch > angle_to_rad(30) || AttitudeVal.pitch < -angle_to_rad(30) ||
			    AttitudeVal.roll  > angle_to_rad(30) || AttitudeVal.roll  < -angle_to_rad(30))
			{
				xEventGroupClearBits(g_xEventFlyAction, StartFly_Event);
			}
		}
		else
		{
			/* 停机状态：重置起飞标志 + 停止电机（含 DShot 命令自恢复） */
			StarFly_UpdateFlag = 1;
			StartFly_SmoothHeightFlag = 1;
			FlyControl_height = 0;
			StopVal_SelfRecovery(&MotorStopVal);
			memcpy(&motor, &MotorStopVal, sizeof(MOTOR_t));
		}

		/* 输出电机 PWM */
		SetPwm(&motor);

		/* 更新调试频率 */
		g_readonly_BalanceTaskFreq = debug->UpdateFreq(&debugPriv);

		/* 导出数据到 APP 显示层 */
		extern APPShowType_t appshow;
		appshow.m1 = motor.A.throttle;      appshow.m3 = motor.C.throttle;
		appshow.pitch = AttitudeVal.pitch;  appshow.roll = AttitudeVal.roll;
		appshow.yaw   = AttitudeVal.yaw;    appshow.balanceTaskFreq = g_readonly_BalanceTaskFreq;
		appshow.height = use_distance;      appshow.c_pitch = FlyControl_pitch;
		appshow.c_roll = FlyControl_yaw;    appshow.c_yaw = FlyControl_gyroz;
		appshow.c_height = FlyControl_unuseHeight;
		appshow.gyrox = axis_9Val.gyro.x;   appshow.gyroy = axis_9Val.gyro.y;
		appshow.gyroz = axis_9Val.gyro.z;   appshow.accelx = axis_9Val.accel.x;
		appshow.accely = axis_9Val.accel.y; appshow.accelz = axis_9Val.accel.z;
		appshow.speedx = speedX;            appshow.speedy = speedY;
		appshow.posx = x;                   appshow.posy = y;
		appshow.targetX = targetPosX;       appshow.targetY = targetPosY;

		/* 精确延时保持 200Hz 频率 */
		vTaskDelayUntil(&preTime, pdMS_TO_TICKS((1.0f / (float)TaskFreq) * 1000));
	}
}

/* ==================== 飞控对外接口函数 ======================================*/

/**
 * @brief  写入飞行控制指令队列（含优先级仲裁）
 * @param  val:               控制指令
 * @param  writeEnv:          写入环境（0=任务, 1=ISR）
 * @param  HigherPriorityTask: ISR 唤醒的任务句柄
 */
void WriteFlyControlQueue(FlyControlType_t val, uint8_t writeEnv, BaseType_t* HigherPriorityTask)
{
	extern QueueHandle_t g_xQueueFlyControl;
	if (val.source > controlCmdNumber) controlCmdNumber = val.source;
	if (val.source == controlCmdNumber)
	{
		if (1 == writeEnv) xQueueOverwriteFromISR(g_xQueueFlyControl, &val, HigherPriorityTask);
		else               xQueueOverwrite(g_xQueueFlyControl, &val);
	}
}

/**
 * @brief  启动/停止飞行
 */
void StartFlyAction(void)
{
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (uxBits & StartFly_Event)                     /* 已起飞 → 停机 */
		xEventGroupClearBits(g_xEventFlyAction, StartFly_Event);
	else if (g_robotVOL > ROBOT_VOL_LIMIT
	         && (uxBits & IMU_CalibZeroDone_Event)
	         && !(uxBits & LowPower_Event))           /* 条件满足 → 起飞 */
	{
		xEventGroupSetBits(g_xEventFlyAction, StartFly_Event);
		xTimerStart(priv_OperateResponseTimer, 0);
		if (uxBits & TestMotorMode_Event)
			xEventGroupClearBits(g_xEventFlyAction, TestMotorMode_Event);
	}
	else
		xTimerStart(priv_OperateFullTimer, 0);
}

/**
 * @brief  停止飞行
 */
void StopFlyAction(void)
{
	xEventGroupClearBits(g_xEventFlyAction, StartFly_Event);
	xTimerStart(priv_OperateResponseTimer, 0);
}

/**
 * @brief  切换无高度模式
 */
void FlyAction_EnterUNUSEHeightMode(void)
{
	static uint8_t mode = 0;
	mode = !mode;
	if (mode) xEventGroupSetBits(g_xEventFlyAction, UNUSE_HeightMode_Event), xTimerStart(priv_UNUSEHeightTimer, 0);
	else      xEventGroupClearBits(g_xEventFlyAction, UNUSE_HeightMode_Event), xTimerStart(priv_OperateResponseTimer, 0);
}

/**
 * @brief  微调角度零偏（0.1° 步长）
 */
void FlyAction_AdjustDiffAngle(uint8_t changeNum)
{
	const float step = 0.1f;
	     if (1 == changeNum) userset_pitch += angle_to_rad(step);
	else if (2 == changeNum) userset_pitch -= angle_to_rad(step);
	else if (3 == changeNum) userset_roll  += angle_to_rad(step);
	else if (4 == changeNum) userset_roll  -= angle_to_rad(step);
	xTimerStart(priv_OperateResponseTimer, 0);
}

/**
 * @brief  切换激光雷达模式（避障/跟随/关闭）
 */
void FlyAction_ChangeLidarAvoidMode(uint8_t id)
{
	xTimerStart(priv_OperateResponseTimer, 0);
	if (id == 1)                                   /* 避障模式 */
		{ xEventGroupClearBits(g_xEventFlyAction, lidar_follow_mode);
		  xEventGroupSetBits(g_xEventFlyAction, lidar_avoid_mode); }
	else if (id == 2)                              /* 跟随模式 */
		{ xEventGroupClearBits(g_xEventFlyAction, lidar_avoid_mode);
		  xEventGroupSetBits(g_xEventFlyAction, lidar_follow_mode); }
	else                                           /* 关闭 */
		{ xEventGroupClearBits(g_xEventFlyAction, lidar_follow_mode);
		  xEventGroupClearBits(g_xEventFlyAction, lidar_avoid_mode); }
}

/**
 * @brief  保存用户微调角度到 Flash
 */
void FlyAction_SaveDiffAngleParam(void)
{
	extern uint8_t User_Flash_SaveParam(uint32_t* data, uint16_t datalen);
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (!(uxBits & StartFly_Event))
	{
		int32_t saveparam[2] = { 0 };
		saveparam[0] = *((int32_t*)&userset_pitch);
		saveparam[1] = *((int32_t*)&userset_roll);
		taskENTER_CRITICAL();
		uint8_t res = User_Flash_SaveParam((uint32_t*)saveparam, 2);
		taskEXIT_CRITICAL();
		if (1 == res) xTimerStart(priv_OperateFullTimer, 0);
	}
}

/**
 * @brief  切换无头模式
 */
void FlyAction_HeadLessModeChange(void)
{
	static uint8_t flag = 1;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (!(uxBits & StartFly_Event))
	{
		flag = !flag;
		if (flag)
			{ xEventGroupClearBits(g_xEventFlyAction, FlyMode_HeadLessMode_Event);
			  xTimerStart(priv_OperateResponseTimer, 0); }
		else
			{ xEventGroupSetBits(g_xEventFlyAction, FlyMode_HeadLessMode_Event);
			  xTimerStart(priv_UNUSEHeightTimer, 0); }
	}
}

/**
 * @brief  系统复位（仅在未起飞时允许）
 */
void ResetSystem(uint8_t isFromISR)
{
	EventBits_t uxBits = 0;
	if (isFromISR) uxBits = xEventGroupGetBitsFromISR(g_xEventFlyAction);
	else           uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (!(uxBits & StartFly_Event))
		NVIC_SystemReset();
	else if (isFromISR) xTimerStartFromISR(priv_OperateFullTimer, 0);
	else                 xTimerStart(priv_OperateFullTimer, 0);
}

/**
 * @brief  电机测试模式开关
 */
void FlyAction_TestMotorMode(void)
{
	static uint8_t flag = 0;
	xEventGroupClearBits(g_xEventFlyAction, StartFly_Event);
	xTimerStart(priv_OperateResponseTimer, 0);
	flag = !flag;
	if (flag) xEventGroupSetBits(g_xEventFlyAction, TestMotorMode_Event);
	else      xEventGroupClearBits(g_xEventFlyAction, TestMotorMode_Event);
}

/* 电机测试油门基准值 */
const uint16_t testmotorVal = 160;

/* 四路电机测试函数: operateNum=0 启停, =1 反转方向, =2 保存设置 */
void TestMotorMode_TestA(uint8_t operateNum) { /* ... (电机A测试逻辑,同上) ... */
	if (operateNum > 2) return;
	static uint8_t flag1 = 0, flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (uxBits & TestMotorMode_Event) {
		xTimerStart(priv_OperateResponseTimer, 0);
		if (0 == operateNum) {
			flag1 = !flag1;
			MotorStopVal.A.Telemetry = 0;
			MotorStopVal.A.throttle = flag1 ? testmotorVal : Dshot_MIN;
		} else if (1 == operateNum) {
			flag2 = !flag2;
			MotorStopVal.A.Telemetry = 1;
			MotorStopVal.A.throttle = flag2 ? DSHOT_CMD_SPIN_DIRECTION_1 : DSHOT_CMD_SPIN_DIRECTION_2;
		} else if (2 == operateNum) {
			MotorStopVal.A.Telemetry = 1;
			MotorStopVal.A.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}
	} else flag1 = 0, flag2 = 0;
}

void TestMotorMode_TestB(uint8_t operateNum) {
	if (operateNum > 2) return;
	static uint8_t flag1 = 0, flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (uxBits & TestMotorMode_Event) {
		xTimerStart(priv_OperateResponseTimer, 0);
		if (0 == operateNum) {
			flag1 = !flag1;
			MotorStopVal.B.Telemetry = 0;
			MotorStopVal.B.throttle = flag1 ? testmotorVal : Dshot_MIN;
		} else if (1 == operateNum) {
			flag2 = !flag2;
			MotorStopVal.B.Telemetry = 1;
			MotorStopVal.B.throttle = flag2 ? DSHOT_CMD_SPIN_DIRECTION_1 : DSHOT_CMD_SPIN_DIRECTION_2;
		} else if (2 == operateNum) {
			MotorStopVal.B.Telemetry = 1;
			MotorStopVal.B.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}
	} else flag1 = 0, flag2 = 0;
}

void TestMotorMode_TestC(uint8_t operateNum) {
	if (operateNum > 2) return;
	static uint8_t flag1 = 0, flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (uxBits & TestMotorMode_Event) {
		xTimerStart(priv_OperateResponseTimer, 0);
		if (0 == operateNum) {
			flag1 = !flag1;
			MotorStopVal.C.Telemetry = 0;
			MotorStopVal.C.throttle = flag1 ? testmotorVal : Dshot_MIN;
		} else if (1 == operateNum) {
			flag2 = !flag2;
			MotorStopVal.C.Telemetry = 1;
			MotorStopVal.C.throttle = flag2 ? DSHOT_CMD_SPIN_DIRECTION_1 : DSHOT_CMD_SPIN_DIRECTION_2;
		} else if (2 == operateNum) {
			MotorStopVal.C.Telemetry = 1;
			MotorStopVal.C.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}
	} else flag1 = 0, flag2 = 0;
}

void TestMotorMode_TestD(uint8_t operateNum) {
	if (operateNum > 2) return;
	static uint8_t flag1 = 0, flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if (uxBits & TestMotorMode_Event) {
		xTimerStart(priv_OperateResponseTimer, 0);
		if (0 == operateNum) {
			flag1 = !flag1;
			MotorStopVal.D.Telemetry = 0;
			MotorStopVal.D.throttle = flag1 ? testmotorVal : Dshot_MIN;
		} else if (1 == operateNum) {
			flag2 = !flag2;
			MotorStopVal.D.Telemetry = 1;
			MotorStopVal.D.throttle = flag2 ? DSHOT_CMD_SPIN_DIRECTION_1 : DSHOT_CMD_SPIN_DIRECTION_2;
		} else if (2 == operateNum) {
			MotorStopVal.D.Telemetry = 1;
			MotorStopVal.D.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}
	} else flag1 = 0, flag2 = 0;
}
