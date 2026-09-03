/**
 * @file    bsp_dshot.c
 * @brief   DShot 数字电调协议驱动（DShot300）
 * @note    使用 TIM8 的四路 PWM 通道输出 DShot300 协议信号（300 kbps）
 *          支持两种驱动方式：DMA 自动发送（默认）和定时器中断逐位发送
 *          DShot 数据帧：11 位油门值 + 1 位遥测请求 + 4 位 CRC 校验 = 16 位
 */

#include "bsp_dshot.h"
#include "tim.h"

/* DShot300 协议位定义 --------------------------------------------------------*/
/* PWM 频率 300kHz，周期约 3.33us
 * BIT_1: 占空比 75%（高电平 2.5us）
 * BIT_0: 占空比 37.5%（高电平 1.25us） */
#define ESC_BIT_1 420                    /* PWM 比较值：75% 占空比 → 逻辑 1 */
#define ESC_BIT_0 210                    /* PWM 比较值：37.5% 占空比 → 逻辑 0 */
#define ESC_CMD_BUF_LEN 20               /* 16 位数据帧 + 4 个填充位 = 20 个 PWM 周期 */

/* 四路电机 DShot 命令缓冲区（每个电机独立一组 PWM 比较值序列） */
static uint16_t g_esc_cmd[4][ESC_CMD_BUF_LEN] = { 0 };

/* 驱动模式选择：1=定时器中断逐位发送, 0=DMA 自动发送 */
#define NOT_DMA_MOTOR 0

#if 1 == NOT_DMA_MOTOR
static uint8_t g_esc_index = ESC_CMD_BUF_LEN; /* 当前发送位索引 */
#endif

/* DShot 使用的定时器（TIM8，4 路 PWM 通道） */
TIM_HandleTypeDef* DshotTIM = &htim8;

/**
 * @brief  DShot 数据帧编码（11 位油门 + 1 位遥测 + 4 位 CRC）
 * @param  val: 包含油门值和遥测标志的结构体
 * @retval 编码后的 16 位 DShot 数据帧
 * @note   CRC 算法: (payload ^ (payload>>4) ^ (payload>>8)) & 0x0F
 *         帧结构: [11:0] = {throttle[10:0], telemetry}, [15:12] = CRC4
 */
static uint16_t DshotDecode(dshotMotorVal_t val)
{
	/* 组装 12 位 payload：11 位油门 + 1 位遥测请求标志 */
	uint16_t packet = (val.throttle << 1) | (val.Telemetry ? 1 : 0);

	/* 计算 4 位 CRC 校验：异或移位法 */
	uint8_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;

	/* 将 CRC 附加到 payload 低 4 位后 */
	packet = (packet << 4) | crc;

	return packet;
}

/**
 * @brief  设置四路电机的 DShot 数字油门值
 * @param  m1~m4: 四路电机的油门值（0~2047）和遥测请求标志
 * @note   油门值被限制在 0~2047 范围内（DShot 协议 11 位分辨率）
 *          编码后通过 DMA 一次性发送四路 PWM 波形序列
 */
static void pwmWriteDigital(dshotMotorVal_t m1, dshotMotorVal_t m2,
                            dshotMotorVal_t m3, dshotMotorVal_t m4)
{
	/* 油门值限幅（DShot 协议最大值 2047） */
	m1.throttle = ((m1.throttle > 2047) ? 2047 : m1.throttle);
	m2.throttle = ((m2.throttle > 2047) ? 2047 : m2.throttle);
	m3.throttle = ((m3.throttle > 2047) ? 2047 : m3.throttle);
	m4.throttle = ((m4.throttle > 2047) ? 2047 : m4.throttle);

	/* 数据帧编码（加入 CRC） */
	m1.throttle = DshotDecode(m1);
	m2.throttle = DshotDecode(m2);
	m3.throttle = DshotDecode(m3);
	m4.throttle = DshotDecode(m4);

	/* 将 16 位数据帧转换为 PWM 占空比序列（MSB 优先发送） */
	uint8_t i = 0;
	for (i = 0; i < 16; i++)
	{
		g_esc_cmd[0][i] = ((m1.throttle >> (15 - i)) & 0x01) ? ESC_BIT_1 : ESC_BIT_0;
		g_esc_cmd[1][i] = ((m2.throttle >> (15 - i)) & 0x01) ? ESC_BIT_1 : ESC_BIT_0;
		g_esc_cmd[2][i] = ((m3.throttle >> (15 - i)) & 0x01) ? ESC_BIT_1 : ESC_BIT_0;
		g_esc_cmd[3][i] = ((m4.throttle >> (15 - i)) & 0x01) ? ESC_BIT_1 : ESC_BIT_0;
	}

#if NOT_DMA_MOTOR
	/* 非 DMA 模式：使用定时器中断逐位更新 PWM 占空比 */
	while (1) if (ESC_CMD_BUF_LEN == g_esc_index) break;
	g_esc_index = 0;
	HAL_TIM_Base_Start_IT(DshotTIM);
#else
	/* DMA 模式：一次性启动四路 PWM DMA 传输 */
	HAL_TIM_PWM_Start_DMA(DshotTIM, TIM_CHANNEL_1, (uint32_t*)&g_esc_cmd[0][0], ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM, TIM_CHANNEL_2, (uint32_t*)&g_esc_cmd[1][0], ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM, TIM_CHANNEL_3, (uint32_t*)&g_esc_cmd[2][0], ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM, TIM_CHANNEL_4, (uint32_t*)&g_esc_cmd[3][0], ESC_CMD_BUF_LEN);
#endif
}

#if 1 == NOT_DMA_MOTOR
/**
 * @brief  TIM8 更新中断回调（非 DMA 模式下逐位发送 DShot 数据）
 * @note   每次定时器溢出中断更新四路 PWM 占空比
 *          发送完 16 位数据帧后停止定时器并清零占空比
 */
void User_TIM8_UpdateCallback(void)
{
	/* 更新四路 PWM 通道的比较值（当前位的占空比） */
	__HAL_TIM_SET_COMPARE(DshotTIM, TIM_CHANNEL_1, g_esc_cmd[0][g_esc_index]);
	__HAL_TIM_SET_COMPARE(DshotTIM, TIM_CHANNEL_2, g_esc_cmd[1][g_esc_index]);
	__HAL_TIM_SET_COMPARE(DshotTIM, TIM_CHANNEL_3, g_esc_cmd[2][g_esc_index]);
	__HAL_TIM_SET_COMPARE(DshotTIM, TIM_CHANNEL_4, g_esc_cmd[3][g_esc_index]);

	g_esc_index++;                           /* 移至下一位 */

	if (g_esc_index == ESC_CMD_BUF_LEN)      /* 一帧发送完毕 */
	{
		HAL_TIM_Base_Stop_IT(DshotTIM);      /* 停止定时器 */
		/* 清零所有通道占空比 */
		__HAL_TIM_SET_COMPARE(DshotTIM, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(DshotTIM, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(DshotTIM, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(DshotTIM, TIM_CHANNEL_4, 0);
	}
}
#endif

/**
 * @brief  电机驱动初始化
 * @note   非 DMA 模式下需预先启动四路 PWM 输出通道
 */
static void motor_init(void)
{
#if NOT_DMA_MOTOR
	/* 预启动 PWM 通道（DMA 模式下在 set_target 中启动） */
	HAL_TIM_PWM_Start(DshotTIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(DshotTIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(DshotTIM, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(DshotTIM, TIM_CHANNEL_4);
#endif
}

/* 电机接口结构体实例 ---------------------------------------------------------*/
/* 通过函数指针向上层 APP 暴露统一的电机控制接口 */

MotorInterface_t UserDshotMotor = {
	.init      = motor_init,       /* 电机驱动初始化 */
	.set_target = pwmWriteDigital, /* 设置四路电机油门值 */
};
