/**
 * @file    bsp_siic.c
 * @brief   软件模拟 I2C 通信驱动
 * @note    使用 GPIO 模拟 I2C 时序，适用于无硬件 I2C 或作为备用的场景
 *          SDA 引脚需支持开漏输出 + 输入模式切换
 *          可配置是否使用延时微调（userconfig_USE_DELAY）
 *          包含完整的使用示例（MPU6050 和 IO 扩展芯片）
 */

#include "bsp_siic.h"
#include "gpio.h"

/* 软件 I2C 配置 --------------------------------------------------------------*/
#define userconfig_USE_DELAY  1          /* 是否启用逻辑延时（1=启用，某些高速设备需要延时才能正常通信） */
#define userconfig_DELAY_TIME 25         /* 延时循环次数（while(delay--) 方式）
                                             主频越高该值需调大，根据实际通信效果微调
                                             过大会导致时序过慢而无法正常通信 */

/* GPIO 宏定义：简化 SDA/SCL 电平控制 */
#define sIIC_SDA_H HAL_GPIO_WritePin(sIIC_SDA_GPIO_Port, sIIC_SDA_Pin, GPIO_PIN_SET)
#define sIIC_SDA_L HAL_GPIO_WritePin(sIIC_SDA_GPIO_Port, sIIC_SDA_Pin, GPIO_PIN_RESET)
#define sIIC_SCL_H HAL_GPIO_WritePin(sIIC_SCL_GPIO_Port, sIIC_SCL_Pin, GPIO_PIN_SET)
#define sIIC_SCL_L HAL_GPIO_WritePin(sIIC_SCL_GPIO_Port, sIIC_SCL_Pin, GPIO_PIN_RESET)

/**
 * @brief  将 SDA 引脚配置为输入模式（用于读取数据和应答检测）
 */
static void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin   = sIIC_SDA_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(sIIC_SDA_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief  将 SDA 引脚配置为推挽输出模式（用于发送数据和应答）
 */
static void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin   = sIIC_SDA_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(sIIC_SDA_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief  读取 SDA 引脚当前电平
 * @retval GPIO_PIN_SET(1) 或 GPIO_PIN_RESET(0)
 */
static uint8_t READ_SDA(void)
{
	return HAL_GPIO_ReadPin(sIIC_SDA_GPIO_Port, sIIC_SDA_Pin);
}

/**
 * @brief  发送 I2C 起始信号（START: SCL 高时 SDA 下降沿）
 */
static void sIIC_Start(void)
{
	SDA_OUT();                               /* SDA 设为输出 */
	sIIC_SDA_H;                              /* SDA 拉高 */
	sIIC_SCL_H;                              /* SCL 拉高 */

#if userconfig_USE_DELAY == 1
	uint8_t delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SDA_L;                              /* SCL 高时 SDA 拉低 → START 信号 */

#if userconfig_USE_DELAY == 1
	delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SCL_L;                              /* 拉低 SCL，准备传输数据 */
}

/**
 * @brief  发送 I2C 停止信号（STOP: SCL 高时 SDA 上升沿）
 */
static void sIIC_Stop(void)
{
	SDA_OUT();                               /* SDA 设为输出 */
	sIIC_SCL_L;                              /* 先拉低 SCL */
	sIIC_SDA_L;                              /* 拉低 SDA */

#if userconfig_USE_DELAY == 1
	uint8_t delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SCL_H;                              /* SCL 拉高 */

#if userconfig_USE_DELAY == 1
	delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SDA_H;                              /* SCL 高时 SDA 上升沿 → STOP 信号 */
}

/**
 * @brief  等待从设备应答信号（ACK）
 * @param  timeout: 超时计数值
 * @retval 1=收到 ACK（SDA 被从设备拉低）, 0=超时未应答
 * @note   主机释放 SDA 后拉高 SCL，检测 SDA 是否被从设备拉低
 */
static uint8_t sIIC_WaitAck(uint32_t timeout)
{
	uint32_t time = 0;

	SDA_IN();                                /* SDA 设为输入，释放总线 */
	sIIC_SDA_H;                              /* 上拉电阻拉高 SDA */

#if userconfig_USE_DELAY == 1
	uint8_t delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SCL_H;                              /* 拉高 SCL，从设备此时应拉低 SDA */

#if userconfig_USE_DELAY == 1
	delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	while (READ_SDA())                       /* 等待 SDA 被从设备拉低 */
	{
		time++;
		if (time > timeout)
		{
			sIIC_Stop();                     /* 超时则发送 STOP 信号 */
			return 0;                        /* 返回失败 */
		}
	}

	sIIC_SCL_L;                              /* 拉低 SCL，继续后续传输 */
	return 1;                                /* 成功收到 ACK */
}

/**
 * @brief  主机发送 ACK 应答（SDA 拉低 1 个时钟周期）
 */
static void sIIC_Ack(void)
{
	sIIC_SCL_L;                              /* 拉低 SCL */
	SDA_OUT();                               /* SDA 设为输出 */
	sIIC_SDA_L;                              /* SDA 拉低 = ACK */

#if userconfig_USE_DELAY == 1
	uint8_t delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SCL_H;                              /* 产生 ACK 时钟脉冲 */

#if userconfig_USE_DELAY == 1
	delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SCL_L;                              /* 结束 ACK 信号 */
}

/**
 * @brief  主机发送 NACK 应答（SDA 保持高电平 1 个时钟周期）
 * @note   通常在读取最后一个字节后发送 NACK 告知从设备停止发送
 */
static void sIIC_NAck(void)
{
	sIIC_SCL_L;                              /* 拉低 SCL */
	SDA_OUT();                               /* SDA 设为输出 */
	sIIC_SDA_H;                              /* SDA 保持高 = NACK */

#if userconfig_USE_DELAY == 1
	uint8_t delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SCL_H;                              /* 产生 NACK 时钟脉冲 */

#if userconfig_USE_DELAY == 1
	delay = userconfig_DELAY_TIME;
	while (delay--);
#endif

	sIIC_SCL_L;                              /* 结束 NACK 信号 */
}

/**
 * @brief  发送一个字节（MSB 优先，标准 I2C 时序）
 * @param  byte: 要发送的 8 位数据
 */
static void sIIC_SendByte(uint8_t byte)
{
	uint8_t i;

	SDA_OUT();                               /* SDA 设为输出 */
	sIIC_SCL_L;                              /* 拉低 SCL，准备发送数据 */

	for (i = 0; i < 8; i++)
	{
		if (byte & 0x80)
			sIIC_SDA_H;                      /* 发送 bit=1 */
		else
			sIIC_SDA_L;                      /* 发送 bit=0 */
		byte <<= 1;                          /* 左移准备下一位 */

#if userconfig_USE_DELAY == 1
		uint8_t delay = userconfig_DELAY_TIME;
		while (delay--);
#endif

		sIIC_SCL_H;                          /* SCL 上升沿：从设备锁存数据 */

#if userconfig_USE_DELAY == 1
		delay = userconfig_DELAY_TIME;
		while (delay--);
#endif

		sIIC_SCL_L;                          /* 拉低 SCL，准备下一位 */
	}
}

/**
 * @brief  接收一个字节（MSB 优先）
 * @param  ack: 1=发送 ACK（继续读取）, 0=发送 NACK（结束读取）
 * @retval 接收到的 8 位数据
 */
static uint8_t sIIC_ReadByte(uint8_t ack)
{
	uint8_t i, byte = 0;

	SDA_IN();                                /* SDA 设为输入，释放总线 */

	for (i = 0; i < 8; i++)
	{
		sIIC_SCL_L;                          /* 拉低 SCL，从设备准备数据 */

#if userconfig_USE_DELAY == 1
		uint8_t delay = userconfig_DELAY_TIME;
		while (delay--);
#endif

		sIIC_SCL_H;                          /* SCL 上升沿：主机读取数据 */

#if userconfig_USE_DELAY == 1
		delay = userconfig_DELAY_TIME;
		while (delay--);
#endif

		byte <<= 1;
		if (READ_SDA())
			byte |= 0x01;                    /* 读取 SDA 电平 */
	}

	if (!ack)
		sIIC_NAck();                         /* 最后一个字节：发送 NACK */
	else
		sIIC_Ack();                          /* 非最后字节：发送 ACK */

	return byte;
}

/**
 * @brief  I2C 主机发送数据（不指定寄存器地址）
 * @param  dev_addr: 从设备 7 位地址（需左移 1 位，LSB=0 表示写）
 * @param  data:     发送数据缓冲区
 * @param  size:     发送字节数
 * @param  timeout:  等待 ACK 的超时计数值
 * @retval IIC_OK=成功, IIC_TIMEOUT=超时失败
 */
static IIC_Status_t IIC_Master_Transmit(uint16_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout)
{
	sIIC_Start();                            /* 发送起始信号 */

	sIIC_SendByte(dev_addr);                 /* 发送从设备地址（写模式：LSB=0） */
	if (!sIIC_WaitAck(timeout))              /* 等待从设备应答 */
	{
		sIIC_Stop();                         /* 无应答则发 STOP 并退出 */
		return IIC_TIMEOUT;
	}

	for (uint16_t i = 0; i < size; i++)
	{
		sIIC_SendByte(data[i]);              /* 逐字节发送数据 */
		if (!sIIC_WaitAck(timeout))          /* 每字节等待应答 */
		{
			sIIC_Stop();
			return IIC_TIMEOUT;
		}
	}

	sIIC_Stop();                             /* 发送停止信号 */
	return IIC_OK;
}

/**
 * @brief  I2C 主机接收数据（不指定寄存器地址）
 * @param  dev_addr: 从设备 7 位地址（需左移 1 位，LSB=1 表示读）
 * @param  data:     接收数据缓冲区
 * @param  size:     接收字节数
 * @param  timeout:  等待 ACK 的超时计数值
 * @retval IIC_OK=成功, IIC_TIMEOUT=超时失败
 */
static IIC_Status_t IIC_Master_Receive(uint16_t dev_addr, uint8_t *data, uint16_t size, uint32_t timeout)
{
	sIIC_Start();                            /* 发送起始信号 */

	sIIC_SendByte(dev_addr | 0x01);          /* 发送从设备地址（读模式：LSB=1） */
	if (!sIIC_WaitAck(timeout))
	{
		sIIC_Stop();
		return IIC_TIMEOUT;
	}

	for (uint16_t i = 0; i < size; i++)
	{
		data[i] = sIIC_ReadByte(i == (size - 1) ? 0 : 1); /* 最后一个字节发 NACK */
	}

	sIIC_Stop();
	return IIC_OK;
}

/**
 * @brief  I2C 写从设备寄存器
 * @param  dev_addr: 从设备 7 位地址
 * @param  mem_addr: 目标寄存器地址（8 位）
 * @param  data:     写入数据缓冲区
 * @param  size:     写入字节数
 * @param  timeout:  等待 ACK 的超时计数值
 * @retval IIC_OK=成功, IIC_TIMEOUT=超时失败
 */
static IIC_Status_t IIC_Mem_Write(uint16_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t size, uint32_t timeout)
{
	sIIC_Start();                            /* 发送起始信号 */

	sIIC_SendByte(dev_addr);                 /* 发送从设备地址（写） */
	if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;

	sIIC_SendByte(mem_addr);                 /* 发送寄存器地址 */
	if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;

	for (uint16_t i = 0; i < size; i++)
	{
		sIIC_SendByte(data[i]);              /* 逐字节发送数据 */
		if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;
	}

	sIIC_Stop();                             /* 发送停止信号 */
	return IIC_OK;
}

/**
 * @brief  I2C 读从设备寄存器（先写寄存器地址，再重新 START 后读取）
 * @param  dev_addr: 从设备 7 位地址
 * @param  mem_addr: 目标寄存器地址（8 位）
 * @param  data:     接收数据缓冲区
 * @param  size:     读取字节数
 * @param  timeout:  等待 ACK 的超时计数值
 * @retval IIC_OK=成功, IIC_TIMEOUT=超时失败
 */
static IIC_Status_t IIC_Mem_Read(uint16_t dev_addr, uint16_t mem_addr, uint8_t *data, uint16_t size, uint32_t timeout)
{
	sIIC_Start();                            /* 发送起始信号 */

	sIIC_SendByte(dev_addr);                 /* 发送从设备地址（写，用于指定寄存器地址） */
	if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;

	sIIC_SendByte(mem_addr);                 /* 发送要读取的寄存器地址 */
	if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;

	sIIC_Start();                            /* 重新发送起始信号（Restart） */
	sIIC_SendByte(dev_addr | 0x01);          /* 发送从设备地址（读模式） */
	if (!sIIC_WaitAck(timeout)) return IIC_TIMEOUT;

	for (uint16_t i = 0; i < size; i++)
	{
		data[i] = sIIC_ReadByte(i == (size - 1) ? 0 : 1); /* 最后一个字节发 NACK */
	}

	sIIC_Stop();                             /* 发送停止信号 */
	return IIC_OK;
}

/**
 * @brief  毫秒延时（封装 HAL_Delay）
 * @param  ms: 延时毫秒数
 */
static void iic_delayms(uint16_t ms)
{
	HAL_Delay(ms);
}

/* 软件 I2C 接口结构体实例 ----------------------------------------------------*/
/* 与硬件 I2C (bsp_iic.c) 使用相同的接口类型，可无缝替换 */

IICInterface_t UserII2Dev = {
	.write     = IIC_Master_Transmit,  /* 主机发送 */
	.read      = IIC_Master_Receive,   /* 主机接收 */
	.write_reg = IIC_Mem_Write,        /* 写寄存器 */
	.read_reg  = IIC_Mem_Read,         /* 读寄存器 */
	.delay_ms  = iic_delayms           /* 毫秒延时 */
};


#if 0 /* 使用示例：MPU6050 初始化（仅供参考） */
/*
void mpu6050_init(void)
{
	#define MPU6050_DEV 0x68
	pIICInterface_t iic = &UserII2Dev;
	uint8_t writebuf = 0;

	writebuf = 0;
	iic->write_reg(MPU6050_DEV << 1, 0x6B, &writebuf, 1, 200); /* 唤醒 MPU6050 */

	writebuf = 0x07;
	iic->write_reg(MPU6050_DEV << 1, 0x19, &writebuf, 1, 200); /* 设置采样率分频 */
	writebuf = 0x06;
	iic->write_reg(MPU6050_DEV << 1, 0x1A, &writebuf, 1, 200); /* 设置 DLPF */
	writebuf = 0x01;
	iic->write_reg(MPU6050_DEV << 1, 0x1C, &writebuf, 1, 200); /* 加速度计量程 ±2g */
	writebuf = 0x18;
	iic->write_reg(MPU6050_DEV << 1, 0x1B, &writebuf, 1, 200); /* 陀螺仪量程 ±2000dps */

	printf("read mpu:%d\r\n", iic->read_reg(0x68 << 1, 0x3B, accel, 6, 200));
	printf("X:%.2f\r\n", (float)(short)(accel[0] << 8 | accel[1]) * 0.00059814453125f);
	printf("Y:%.2f\r\n", (float)(short)(accel[2] << 8 | accel[3]) * 0.00059814453125f);
	printf("Z:%.2f\r\n\r\n", (float)(short)(accel[4] << 8 | accel[5]) * 0.00059814453125f);
}
*/

/* 使用示例：IO 扩展芯片控制（仅供参考） */
/*
void IO_TOP_demo(void* param)
{
	pIICInterface_t iic = &UserII2Dev;
	#define IO_TOP_Dev 0x20              // IO 扩展芯片设备地址

	uint8_t buf[2] = { 0, 0 };
	uint8_t readbuf[2] = { 0xAA, 0xBB };
	uint8_t flag = 0;
	iic->write(IO_TOP_Dev << 1, buf, 2, 200); // 初始化：关闭所有 IO

	for (;;)
	{
		flag = !flag;
		if (flag) buf[0] = 0, buf[1] = 0;
		else      buf[0] = 0xff, buf[1] = 0xff;

		printf("write:%d\r\n", iic->write(IO_TOP_Dev << 1, buf, 2, 200));
		printf("read:%d\r\n",  iic->read(IO_TOP_Dev << 1, readbuf, 2, 200));
		printf("0x%2X    0x%2X\r\n", readbuf[0], readbuf[1]);
	}
}
*/
#endif
