/**
 * ============================================================
 * bsp_dshot.c — DShot300 电调协议驱动
 * ============================================================
 * 协议: DShot300 (300kbps 数字电调协议)
 * 帧格式: 11位油门 + 1位遥测请求 + 4位CRC = 16位
 * 位编码: 占空比75%=1, 37.5%=0 (PWM脉宽调制)
 *
 * 功能:
 *   - set_target: 设置4路电机油门 (非DMA模式逐位输出)
 *   - DShotCMD:   发送特殊命令 (电机转向/保存设置/蜂鸣等)
 *   - Dshot_ReverseMotor: 反转电机转向 (含20次重复+保存)
 *
 * 定时器: TIM1_CH1~CH4, 频率8MHz, ARR=29 (对应300kbps)
 * ============================================================
 */

#include "bsp_dshot.h"
#include "tim.h"

//Dshot300 --> 300k bit/s  PWM频率需要为 300k
#define ESC_BIT_1 420       //PWM 75%   占空比为逻辑1
#define ESC_BIT_0 210       //PWM 37.5% 占空比为逻辑0
#define ESC_CMD_BUF_LEN 20  //16bit的数据帧.这里20是为了增加数据帧之间的延迟

static uint16_t g_esc_cmd[4][ESC_CMD_BUF_LEN] = { 0 };

//是否失能DMA
#define NOT_DMA_MOTOR 0


#if 1 == NOT_DMA_MOTOR
static uint8_t g_esc_index = ESC_CMD_BUF_LEN;
#endif


//生成dshot信号的定时器
TIM_HandleTypeDef* DshotTIM = &htim8;


//Dshot数据解码
static uint16_t DshotDecode(dshotMotorVal_t val)
{
    // 油门大小为11位  所以这里先左移一位 添加上请求回传标志共12位
    uint16_t packet = (val.throttle << 1) | (val.Telemetry ? 1 : 0);
	
	uint8_t crc = ( packet^(packet>>4)^(packet>>8) ) & 0x0F;
	
    // append checksum 将CRC添加到后四位
    packet = (packet << 4) | crc;
	
    return packet;
	
}

//设置电机转速
static void pwmWriteDigital(dshotMotorVal_t m1,dshotMotorVal_t m2,dshotMotorVal_t m3,dshotMotorVal_t m4)
{
	
	//最大值限幅
	m1.throttle = ( (m1.throttle > 2047) ? 2047 : m1.throttle );
	m2.throttle = ( (m2.throttle > 2047) ? 2047 : m2.throttle );
	m3.throttle = ( (m3.throttle > 2047) ? 2047 : m3.throttle );
	m4.throttle = ( (m4.throttle > 2047) ? 2047 : m4.throttle );
	
	//数据解码
	m1.throttle = DshotDecode(m1);
	m2.throttle = DshotDecode(m2);
	m3.throttle = DshotDecode(m3);
	m4.throttle = DshotDecode(m4);
	
	//16bit数据赋值,逻辑0和1均使用PWM占空比表示
	uint8_t i=0;
	for(i=0;i<16;i++)
	{
		g_esc_cmd[0][i] =  (( m1.throttle>>(15-i) )&0x01 ) ? ESC_BIT_1 : ESC_BIT_0;
		g_esc_cmd[1][i] =  (( m2.throttle>>(15-i) )&0x01 ) ? ESC_BIT_1 : ESC_BIT_0;
		g_esc_cmd[2][i] =  (( m3.throttle>>(15-i) )&0x01 ) ? ESC_BIT_1 : ESC_BIT_0;
		g_esc_cmd[3][i] =  (( m4.throttle>>(15-i) )&0x01 ) ? ESC_BIT_1 : ESC_BIT_0;
	}
	
	#if NOT_DMA_MOTOR //非DMA方式
	while(1) if( ESC_CMD_BUF_LEN == g_esc_index ) break;
	g_esc_index = 0;
	HAL_TIM_Base_Start_IT(DshotTIM);
	#else //DMA
	HAL_TIM_PWM_Start_DMA(DshotTIM,TIM_CHANNEL_1,(uint32_t*)&g_esc_cmd[0][0],ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM,TIM_CHANNEL_2,(uint32_t*)&g_esc_cmd[1][0],ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM,TIM_CHANNEL_3,(uint32_t*)&g_esc_cmd[2][0],ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM,TIM_CHANNEL_4,(uint32_t*)&g_esc_cmd[3][0],ESC_CMD_BUF_LEN);
	#endif
}

//不使用DMA时,需要定义定时器8回调函数处理信息
#if 1 == NOT_DMA_MOTOR
void User_TIM8_UpdateCallback(void)
{
	__HAL_TIM_SET_COMPARE(DshotTIM,TIM_CHANNEL_1,g_esc_cmd[0][g_esc_index]);//占空比设置
	__HAL_TIM_SET_COMPARE(DshotTIM,TIM_CHANNEL_2,g_esc_cmd[1][g_esc_index]);
	__HAL_TIM_SET_COMPARE(DshotTIM,TIM_CHANNEL_3,g_esc_cmd[2][g_esc_index]);
	__HAL_TIM_SET_COMPARE(DshotTIM,TIM_CHANNEL_4,g_esc_cmd[3][g_esc_index]);
	
	g_esc_index++; //索引自增
	
	if( g_esc_index == ESC_CMD_BUF_LEN )
	{
		HAL_TIM_Base_Stop_IT(DshotTIM);
		__HAL_TIM_SET_COMPARE(DshotTIM,TIM_CHANNEL_1,0);//占空比设置
		__HAL_TIM_SET_COMPARE(DshotTIM,TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(DshotTIM,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(DshotTIM,TIM_CHANNEL_4,0);
	}
}
#endif

//电机初始化
static void motor_init(void)
{
	//非DMA方式,需要先使能电机输出PWM
	#if NOT_DMA_MOTOR
	HAL_TIM_PWM_Start(DshotTIM,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(DshotTIM,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(DshotTIM,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(DshotTIM,TIM_CHANNEL_4);
	#endif
}

//设置电机方向
//static void set_motor_dir(uint8_t who,uint8_t dir)
//{
//	
//	uint16_t i = 0;
//	for(i=0;i<8;i++)
//	{
//		pwmWriteDigital(0,0,0,0);
//		HAL_Delay(5);
//	}
//	
//	HAL_Delay(35);
//}




//驱动挂载
MotorInterface_t UserDshotMotor = {
	.init = motor_init,
	.set_target = pwmWriteDigital,
};


