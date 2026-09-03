/**
 * ============================================================
 * bsp_iic.c — 硬件I2C驱动 (STM32 HAL封装)
 * ============================================================
 * 功能: 封装 HAL_I2C 库函数, 提供统一接口
 *   - iic_write: 写数据到从设备
 *   - iic_read:  从从设备读数据
 *   - iic_mem_write / iic_mem_read: 寄存器操作
 * 接口: 与软件I2C (bsp_siic) 接口兼容, 可互换使用
 * ============================================================
 */

#include "bsp_iic.h"
#include "i2c.h"

/* 硬件iic */

//使用哪一路IIC接口
static I2C_HandleTypeDef *usri2c = &hi2c1;

//写数据
static IIC_Status_t iic_write(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	IIC_Status_t iicstate = IIC_ERR;
	HAL_StatusTypeDef state = HAL_ERROR;
	state = HAL_I2C_Master_Transmit(usri2c,DevAddress,pData,Size,Timeout);
	if ( HAL_OK == state ) iicstate = IIC_OK;
	else if( HAL_ERROR == state ) iicstate = IIC_ERR;
	else if( HAL_BUSY == state ) iicstate = IIC_BUSY;
	else if( HAL_TIMEOUT == state ) iicstate = IIC_TIMEOUT;
	return iicstate;
}

//读数据
static IIC_Status_t iic_read(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	IIC_Status_t iicstate = IIC_ERR;
	HAL_StatusTypeDef state = HAL_ERROR;
	state = HAL_I2C_Master_Receive(usri2c,DevAddress,pData,Size,Timeout);
	if ( HAL_OK == state ) iicstate = IIC_OK;
	else if( HAL_ERROR == state ) iicstate = IIC_ERR;
	else if( HAL_BUSY == state ) iicstate = IIC_BUSY;
	else if( HAL_TIMEOUT == state ) iicstate = IIC_TIMEOUT;
	return iicstate;
}

//写寄存器
static IIC_Status_t iic_write_reg(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	IIC_Status_t iicstate = IIC_ERR;
	HAL_StatusTypeDef state = HAL_ERROR;
	state = HAL_I2C_Mem_Write(usri2c,DevAddress,MemAddress,I2C_MEMADD_SIZE_8BIT,pData,Size,Timeout);
	if ( HAL_OK == state ) iicstate = IIC_OK;
	else if( HAL_ERROR == state ) iicstate = IIC_ERR;
	else if( HAL_BUSY == state ) iicstate = IIC_BUSY;
	else if( HAL_TIMEOUT == state ) iicstate = IIC_TIMEOUT;
	return iicstate; 
}


//读寄存器
static IIC_Status_t iic_read_reg(uint16_t DevAddress, uint16_t MemAddress,uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	IIC_Status_t iicstate = IIC_ERR;
	HAL_StatusTypeDef state = HAL_ERROR;
	state = HAL_I2C_Mem_Read(usri2c,DevAddress,MemAddress,I2C_MEMADD_SIZE_8BIT,pData,Size,Timeout);
	if ( HAL_OK == state ) iicstate = IIC_OK;
	else if( HAL_ERROR == state ) iicstate = IIC_ERR;
	else if( HAL_BUSY == state ) iicstate = IIC_BUSY;
	else if( HAL_TIMEOUT == state ) iicstate = IIC_TIMEOUT;
	return iicstate;
}

static void iic_delayms(uint16_t ms)
{
	HAL_Delay(ms);
}
	

//挂载驱动
IICInterface_t UserII2Dev = {
	.write = iic_write , 
	.read = iic_read ,
	.write_reg = iic_write_reg ,
	.read_reg = iic_read_reg ,
	.delay_ms = iic_delayms
};


