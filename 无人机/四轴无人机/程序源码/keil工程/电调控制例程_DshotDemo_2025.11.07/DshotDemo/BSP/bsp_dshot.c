#include "bsp_dshot.h"
#include "tim.h"

//Dshot300 --> 300k bit/s  PWM频率需要为 300k
#define ESC_BIT_1 420       //PWM 75%   占空比为逻辑1
#define ESC_BIT_0 210       //PWM 37.5% 占空比为逻辑0
#define ESC_CMD_BUF_LEN 20  //16bit的数据帧.这里20是为了增加数据帧之间的延迟

static uint16_t g_esc_cmd[4][ESC_CMD_BUF_LEN] = { 0 };


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
	
	HAL_TIM_PWM_Start_DMA(DshotTIM,TIM_CHANNEL_1,(uint32_t*)&g_esc_cmd[0][0],ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM,TIM_CHANNEL_2,(uint32_t*)&g_esc_cmd[1][0],ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM,TIM_CHANNEL_3,(uint32_t*)&g_esc_cmd[2][0],ESC_CMD_BUF_LEN);
	HAL_TIM_PWM_Start_DMA(DshotTIM,TIM_CHANNEL_4,(uint32_t*)&g_esc_cmd[3][0],ESC_CMD_BUF_LEN);
}



//电机初始化
static void motor_init(void)
{
	//已由HAL库完成
}



//驱动挂载
MotorInterface_t UserDshotMotor = {
	.init = motor_init,
	.set_target = pwmWriteDigital,
};


