/**
 * @file    bsp_iic.c
 * @brief   硬件 I2C 通信接口封装（基于 STM32 HAL 库）
 * @note    使用 I2C1 外设，将 HAL 库的 I2C 操作函数封装为统一的 IICInterface_t 接口
 *          支持：主机发送、主机接收、寄存器写、寄存器读、毫秒延时
 */

#include "bsp_iic.h"
#include "i2c.h"

/* 硬件 I2C 句柄 --------------------------------------------------------------*/
/* 使用 I2C1 外设（PB6=SCL, PB7=SDA） */
static I2C_HandleTypeDef *usri2c = &hi2c1;

/**
 * @brief  I2C 主机发送数据（不指定寄存器地址）
 * @param  DevAddress: 从设备地址（7 位地址需左移 1 位）
 * @param  pData:      发送数据缓冲区指针
 * @param  Size:       发送字节数
 * @param  Timeout:    超时时间（ms）
 * @retval IIC_Status_t 状态码（IIC_OK / IIC_ERR / IIC_BUSY / IIC_TIMEOUT）
 */
static IIC_Status_t iic_write(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	IIC_Status_t iicstate = IIC_ERR;
	HAL_StatusTypeDef state = HAL_ERROR;
	state = HAL_I2C_Master_Transmit(usri2c, DevAddress, pData, Size, Timeout);
	if (HAL_OK == state)      iicstate = IIC_OK;
	else if (HAL_ERROR == state)   iicstate = IIC_ERR;
	else if (HAL_BUSY == state)    iicstate = IIC_BUSY;
	else if (HAL_TIMEOUT == state) iicstate = IIC_TIMEOUT;
	return iicstate;
}

/**
 * @brief  I2C 主机接收数据（不指定寄存器地址）
 * @param  DevAddress: 从设备地址（7 位地址需左移 1 位）
 * @param  pData:      接收数据缓冲区指针
 * @param  Size:       接收字节数
 * @param  Timeout:    超时时间（ms）
 * @retval IIC_Status_t 状态码
 */
static IIC_Status_t iic_read(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	IIC_Status_t iicstate = IIC_ERR;
	HAL_StatusTypeDef state = HAL_ERROR;
	state = HAL_I2C_Master_Receive(usri2c, DevAddress, pData, Size, Timeout);
	if (HAL_OK == state)      iicstate = IIC_OK;
	else if (HAL_ERROR == state)   iicstate = IIC_ERR;
	else if (HAL_BUSY == state)    iicstate = IIC_BUSY;
	else if (HAL_TIMEOUT == state) iicstate = IIC_TIMEOUT;
	return iicstate;
}

/**
 * @brief  I2C 写从设备寄存器
 * @param  DevAddress:  从设备地址（7 位地址需左移 1 位）
 * @param  MemAddress:  目标寄存器地址（8 位）
 * @param  pData:       写入数据缓冲区指针
 * @param  Size:        写入字节数
 * @param  Timeout:     超时时间（ms）
 * @retval IIC_Status_t 状态码
 */
static IIC_Status_t iic_write_reg(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	IIC_Status_t iicstate = IIC_ERR;
	HAL_StatusTypeDef state = HAL_ERROR;
	state = HAL_I2C_Mem_Write(usri2c, DevAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
	if (HAL_OK == state)      iicstate = IIC_OK;
	else if (HAL_ERROR == state)   iicstate = IIC_ERR;
	else if (HAL_BUSY == state)    iicstate = IIC_BUSY;
	else if (HAL_TIMEOUT == state) iicstate = IIC_TIMEOUT;
	return iicstate;
}

/**
 * @brief  I2C 读从设备寄存器
 * @param  DevAddress:  从设备地址（7 位地址需左移 1 位）
 * @param  MemAddress:  目标寄存器地址（8 位）
 * @param  pData:       接收数据缓冲区指针
 * @param  Size:        读取字节数
 * @param  Timeout:     超时时间（ms）
 * @retval IIC_Status_t 状态码
 */
static IIC_Status_t iic_read_reg(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	IIC_Status_t iicstate = IIC_ERR;
	HAL_StatusTypeDef state = HAL_ERROR;
	state = HAL_I2C_Mem_Read(usri2c, DevAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
	if (HAL_OK == state)      iicstate = IIC_OK;
	else if (HAL_ERROR == state)   iicstate = IIC_ERR;
	else if (HAL_BUSY == state)    iicstate = IIC_BUSY;
	else if (HAL_TIMEOUT == state) iicstate = IIC_TIMEOUT;
	return iicstate;
}

/**
 * @brief  I2C 毫秒延时（封装 HAL_Delay）
 * @param  ms: 延时毫秒数
 */
static void iic_delayms(uint16_t ms)
{
	HAL_Delay(ms);
}

/* I2C 接口结构体实例 ---------------------------------------------------------*/
/* 通过函数指针暴露统一的 I2C 操作接口，上层（如 IMU 驱动）无需直接依赖 HAL */

IICInterface_t UserII2Dev = {
	.write     = iic_write,       /* 主机发送 */
	.read      = iic_read,        /* 主机接收 */
	.write_reg = iic_write_reg,   /* 写寄存器 */
	.read_reg  = iic_read_reg,    /* 读寄存器 */
	.delay_ms  = iic_delayms      /* 毫秒延时 */
};
