#ifndef __BSP_sIIC_H
#define __BSP_sIIC_H

#include <stdint.h>

typedef enum 
{
  IIC_OK       = 0x00U,
  IIC_ERR      = 0x01U,
  IIC_BUSY     = 0x02U,
  IIC_TIMEOUT  = 0x03U
} IIC_Status_t;

typedef struct {
	//参数：设备地址，要写入的数据，要写入的数据长度，超时时间
    IIC_Status_t (*write)(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    IIC_Status_t (*read)(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	
	//参数：设备地址，要访问的寄存器地址，要写入的数据，要写入的数据长度，超时时间
    IIC_Status_t (*write_reg)(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	IIC_Status_t (*read_reg)(uint16_t DevAddress, uint16_t MemAddress,  uint8_t *pData, uint16_t Size, uint32_t Timeout);
	
	void (*delay_ms)(uint16_t ms);
}IICInterface_t,*pIICInterface_t;

extern IICInterface_t UserII2Dev;

#endif /* __BSP_sIIC_H */
