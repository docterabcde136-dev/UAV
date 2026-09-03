/**
 * @file    pid.c
 * @brief   PID 控制器实现（位置式 + 增量式）+ 级联 PID 参数定义
 * @note    包含姿态环（角速度+角度）、高度环、水平位置环的 PID 控制器参数
 *          位置式 PID 特性：积分分离 + 抗积分饱和 + 输出限幅 + 微分低通滤波
 */

#include "pid.h"
#include "math.h"

/* ======================== PID 复位函数 ======================================*/

/**
 * @brief  位置式 PID 控制器复位
 */
void PID_Reset(PIDControllerType_t* pid)
{
	pid->intergral  = 0;
	pid->output     = 0;
	pid->prev_error = 0;
	pid->derivative = 0;
}

/**
 * @brief  增量式 PID 控制器复位
 */
void PIDIncremental_Reset(PIDIncrementalType_t* pid)
{
	pid->prev_error = 0;
	pid->output     = 0;
}

/* =================== 位置式 PID 更新函数 =====================================*/

/**
 * @brief  俯仰角速度环 PID 更新（微分项直接使用陀螺仪角速率）
 * @param  pid:       PID 控制器实例
 * @param  target:    目标角速率 (rad/s)
 * @param  current:   当前角速率 (rad/s)
 * @param  pitchrate: 陀螺仪直接测量的角速率（用于微分项）
 * @note   微分使用一阶低通滤波：derivative = alpha*new + (1-alpha)*old
 */
void PID_UpdatePitch(PIDControllerType_t* pid, float target, float current, float pitchrate)
{
	float error = target - current;

	/* 积分分离因子：误差大时降低积分作用 */
	float integral_factor = (fabs(error) < pid->IntegralThreshold)
	                        ? 1.0f : (pid->IntegralThreshold / fabs(error));
	pid->intergral += integral_factor * error;

	/* 微分项（一阶低通滤波） */
	float Tmpderivative = pitchrate;
	pid->derivative = pid->alpha * Tmpderivative + (1.0f - pid->alpha) * pid->derivative;

	/* PID 输出 */
	pid->output = pid->kp * error + pid->ki * pid->intergral - pid->kd * pid->derivative;

	/* 输出限幅 + 积分退饱和（积分分离） */
	if (pid->output > pid->LimitOutputMax)
	{
		pid->output = pid->LimitOutputMax;
		if (error > 0) pid->intergral *= 0.9f; /* 正向饱和：减小积分 */
	}
	if (pid->output < pid->LimitOutputMin)
	{
		pid->output = pid->LimitOutputMin;
		if (error < 0) pid->intergral *= 0.9f; /* 负向饱和：减小积分 */
	}

	/* 积分限幅 */
	if (pid->intergral > pid->LimitIntegralMax) pid->intergral = pid->LimitIntegralMax;
	if (pid->intergral < pid->LimitIntegralMin) pid->intergral = pid->LimitIntegralMin;
}

/**
 * @brief  横滚角速度环 PID 更新（微分项直接使用陀螺仪角速率）
 */
void PID_UpdateRoll(PIDControllerType_t* pid, float target, float current, float rollrate)
{
	float error = target - current;

	float integral_factor = (fabs(error) < pid->IntegralThreshold)
	                        ? 1.0f : (pid->IntegralThreshold / fabs(error));
	pid->intergral += integral_factor * error;

	float Tmpderivative = rollrate;
	pid->derivative = pid->alpha * Tmpderivative + (1.0f - pid->alpha) * pid->derivative;

	pid->output = pid->kp * error + pid->ki * pid->intergral - pid->kd * pid->derivative;

	if (pid->output > pid->LimitOutputMax)
	{
		pid->output = pid->LimitOutputMax;
		if (error > 0) pid->intergral *= 0.9f;
	}
	if (pid->output < pid->LimitOutputMin)
	{
		pid->output = pid->LimitOutputMin;
		if (error < 0) pid->intergral *= 0.9f;
	}

	if (pid->intergral > pid->LimitIntegralMax) pid->intergral = pid->LimitIntegralMax;
	if (pid->intergral < pid->LimitIntegralMin) pid->intergral = pid->LimitIntegralMin;
}

/**
 * @brief  通用位置式 PID 更新（微分项由误差差分计算）
 * @note   适用于角度环、高度环等慢速控制回路
 */
void PID_Update(PIDControllerType_t* pid, float target, float current)
{
	float error = target - current;

	float integral_factor = (fabs(error) < pid->IntegralThreshold)
	                        ? 1.0f : (pid->IntegralThreshold / fabs(error));
	pid->intergral += integral_factor * error;

	/* 微分项由误差差分计算（而非直接使用传感器角速率） */
	float Tmpderivative = (error - pid->prev_error);
	pid->derivative = pid->alpha * Tmpderivative + (1.0f - pid->alpha) * pid->derivative;
	pid->prev_error = error;

	pid->output = pid->kp * error + pid->ki * pid->intergral + pid->kd * pid->derivative;

	if (pid->output > pid->LimitOutputMax)
	{
		pid->output = pid->LimitOutputMax;
		if (error > 0) pid->intergral *= 0.9f;
	}
	if (pid->output < pid->LimitOutputMin)
	{
		pid->output = pid->LimitOutputMin;
		if (error < 0) pid->intergral *= 0.9f;
	}

	if (pid->intergral > pid->LimitIntegralMax) pid->intergral = pid->LimitIntegralMax;
	if (pid->intergral < pid->LimitIntegralMin) pid->intergral = pid->LimitIntegralMin;
}

/**
 * @brief  增量式 PID 更新
 */
void PIDIncremental_Update(PIDIncrementalType_t* pid, float target, float current)
{
	float error = target - current;
	pid->output += pid->kp * (error - pid->prev_error) + pid->ki * pid->prev_error;

	if (pid->output > pid->LimitOutputMax) pid->output = pid->LimitOutputMax;
	if (pid->output < pid->LimitOutputMin) pid->output = pid->LimitOutputMin;

	pid->prev_error = error;
}

/* ======================== 级联 PID 参数表 ====================================*/

/* 横滚角速度环 PID（内环，快速响应） */
PIDControllerType_t RollRatePID = {
	.kp = 85.0f, .ki = 0.2f, .kd = 0.5f,
	.LimitIntegralMax = 3.5f,  .LimitIntegralMin = -3.5f,  /* 积分限幅: ±200°/s */
	.LimitOutputMax   = 700.0f, .LimitOutputMin   = -700.0f, /* 输出为电机油门值 */
	.IntegralThreshold = 0.35f, /* 20°/s 以上开始减弱积分 */
	.alpha = 0.8f,              /* 微分低通滤波系数 */
};

/* 俯仰角速度环 PID（内环，快速响应） */
PIDControllerType_t PitchRatePID = {
	.kp = 85.0f, .ki = 0.2f, .kd = 0.5f,
	.LimitIntegralMax = 3.5f,  .LimitIntegralMin = -3.5f,
	.LimitOutputMax   = 700.0f, .LimitOutputMin   = -700.0f,
	.IntegralThreshold = 0.35f,
	.alpha = 0.8f,
};

/* 横滚角度环 PID（外环） */
PIDControllerType_t RollPID = {
	.kp = 6.0f, .ki = 0.0f, .kd = 0.0f,
	.LimitIntegralMax = 0.52f,  .LimitIntegralMin = -0.52f,  /* 积分限幅: ±30° */
	.LimitOutputMax   = 1.7f,   .LimitOutputMin   = -1.7f,   /* 输出限幅: ±100°/s */
	.IntegralThreshold = 0.2f,  /* 10°以上减弱积分 */
	.alpha = 0.9f,
};

/* 俯仰角度环 PID（外环） */
PIDControllerType_t PitchPID = {
	.kp = 6.0f, .ki = 0.0f, .kd = 0.0f,
	.LimitIntegralMax = 0.52f,  .LimitIntegralMin = -0.52f,
	.LimitOutputMax   = 1.7f,   .LimitOutputMin   = -1.7f,
	.IntegralThreshold = 0.2f,
	.alpha = 0.9f,
};

/* 偏航角速度环 PID */
PIDControllerType_t YawRatePID = {
	.kp = 200.0f, .ki = 1.0f, .kd = 10.0f,
	.LimitIntegralMax = 1.04f,  .LimitIntegralMin = -1.04f,  /* 积分限幅: ±60°/s */
	.LimitOutputMax   = 500,    .LimitOutputMin   = -500,
	.IntegralThreshold = 0.17f, /* 10°/s 以上减弱积分 */
	.alpha = 0.8,
};

/* 偏航角度环 PID */
PIDControllerType_t YawPID = {
	.kp = 6.0f, .ki = 0.0f, .kd = 1.5f,
	.LimitIntegralMax = 0.52f,  .LimitIntegralMin = -0.52f,
	.LimitOutputMax   = 1.7f,   .LimitOutputMin   = -1.7f,
	.IntegralThreshold = 0.2f,
	.alpha = 1.0f,
};

/* 高度速度环 PID */
PIDControllerType_t HeightSpeedPID = {
	.kp = 60.0f, .ki = 0.02f, .kd = 80.0f,
	.LimitIntegralMax = 100.0f,  .LimitIntegralMin = -100.0f,
	.LimitOutputMax   = 1000,    .LimitOutputMin   = -1000,
	.IntegralThreshold = 20,
	.alpha = 0.4,
};

/* 高度环 PID */
PIDControllerType_t HeightPID = {
	.kp = 5.5f, .ki = 0.0f, .kd = 0.5f,
	.LimitIntegralMax = 10,  .LimitIntegralMin = -10,
	.LimitOutputMax   = 1.0f, .LimitOutputMin = -1.0f,
	.IntegralThreshold = 10,
	.alpha = 1.0f,
};

/* 水平位置 X 环 PID */
PIDControllerType_t Position_X_PID = {
	.kp = 0.06f, .ki = 0.002f, .kd = 0.00f,
	.LimitIntegralMax = 0.5f,  .LimitIntegralMin = -0.5f,
	.LimitOutputMax   = 2.0f,  .LimitOutputMin   = -2.0f,  /* 输出: 目标速度 1m/s */
	.IntegralThreshold = 0.5f,
	.alpha = 0.8f,
};

/* 水平位置 Y 环 PID */
PIDControllerType_t Position_Y_PID = {
	.kp = 0.06f, .ki = 0.002f, .kd = 0.00f,
	.LimitIntegralMax = 0.5f,  .LimitIntegralMin = -0.5f,
	.LimitOutputMax   = 2.0f,  .LimitOutputMin   = -2.0f,
	.IntegralThreshold = 0.5f,
	.alpha = 0.8f,
};

/* 水平速度 X 环 PID */
PIDControllerType_t Position_XSpeed_PID = {
	.kp = 0.08f, .ki = 0.00f, .kd = 0.01f,
	.LimitIntegralMax = 0.3f,  .LimitIntegralMin = -0.3f,
	.LimitOutputMax   = 0.17f, .LimitOutputMin   = -0.17f, /* 输出: ±10° */
	.IntegralThreshold = 0.5f,
	.alpha = 0.8f,
};

/* 水平速度 Y 环 PID */
PIDControllerType_t Position_YSpeed_PID = {
	.kp = 0.08f, .ki = 0.0f, .kd = 0.01f,
	.LimitIntegralMax = 0.3f,  .LimitIntegralMin = -0.3f,
	.LimitOutputMax   = 0.17f, .LimitOutputMin   = -0.17f,
	.IntegralThreshold = 0.5f,
	.alpha = 0.8f,
};
