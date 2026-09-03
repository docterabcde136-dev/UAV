/**
 * @file    bsp_iic.h
 * @brief   I2C 通信接口定义（支持硬件/软件 I2C 两种实现）
 */

#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include <stdint.h>

/* I2C 操作状态码 */
typedef enum
{
	IIC_OK      = 0x00U,     /* 操作成功 */
	IIC_ERR     = 0x01U,     /* 操作错误 */
	IIC_BUSY    = 0x02U,     /* 总线忙 */
	IIC_TIMEOUT = 0x03U      /* 操作超时 */
} IIC_Status_t;

/* I2C 操作接口（硬件 I2C 和软件模拟 I2C 共用） */
typedef struct {
	/* 主机发送/接收（不指定寄存器地址） */
	IIC_Status_t (*write)(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	IIC_Status_t (*read)(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

	/* 寄存器读写（指定 8 位寄存器地址） */
	IIC_Status_t (*write_reg)(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	IIC_Status_t (*read_reg)(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

	void (*delay_ms)(uint16_t ms);     /* 毫秒级延时 */
} IICInterface_t, *pIICInterface_t;

extern IICInterface_t UserII2Dev;      /* I2C 接口实例 */

#endif /* __BSP_IIC_H */
