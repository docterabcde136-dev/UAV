/**
 * @file    bsp_flash.c
 * @brief   板级 Flash 存储驱动（STM32F405RG 内部 Flash）
 * @note    使用片上 Flash 最后 128KB 扇区（Sector 11, 0x080E0000）存储用户参数
 *          F405RG 总 Flash 容量: 1024KB
 *          扇区 11 起始地址: 0x08000000 + (1024-128)*1024 = 0x080E0000
 *          提供 User_Flash_SaveParam() 和 User_Flash_ReadParam() 两个对外接口
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal_flash.h"
#include <stdio.h>
#include <string.h>

/* Flash 用户数据存储地址（最后 128KB 扇区起始）--------------------------------*/
#define FLASH_SAVE_ADDR 0x080E0000

/* STM32F405RG Flash 扇区地址定义（共 12 个扇区，总 1MB）-----------------------*/
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000)     /* 扇区 0:  16 KB */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000)     /* 扇区 1:  16 KB */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000)     /* 扇区 2:  16 KB */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000)     /* 扇区 3:  16 KB */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000)     /* 扇区 4:  64 KB */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000)     /* 扇区 5:  128 KB */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000)     /* 扇区 6:  128 KB */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000)     /* 扇区 7:  128 KB */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000)     /* 扇区 8:  128 KB */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000)     /* 扇区 9:  128 KB */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000)     /* 扇区 10: 128 KB */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000)     /* 扇区 11: 128 KB */

/**
 * @brief  获取指定地址所在的 Flash 扇区编号
 * @param  addr: Flash 绝对地址
 * @retval 扇区编号（0~11）
 */
static uint8_t stmflash_get_flash_sector(uint32_t addr)
{
	if (addr < ADDR_FLASH_SECTOR_1)  return 0;
	else if (addr < ADDR_FLASH_SECTOR_2)  return 1;
	else if (addr < ADDR_FLASH_SECTOR_3)  return 2;
	else if (addr < ADDR_FLASH_SECTOR_4)  return 3;
	else if (addr < ADDR_FLASH_SECTOR_5)  return 4;
	else if (addr < ADDR_FLASH_SECTOR_6)  return 5;
	else if (addr < ADDR_FLASH_SECTOR_7)  return 6;
	else if (addr < ADDR_FLASH_SECTOR_8)  return 7;
	else if (addr < ADDR_FLASH_SECTOR_9)  return 8;
	else if (addr < ADDR_FLASH_SECTOR_10) return 9;
	else if (addr < ADDR_FLASH_SECTOR_11) return 10;

	return 11;
}

/**
 * @brief  从 Flash 指定地址读取一个 32 位字
 * @param  addr: Flash 绝对地址
 * @retval 读取的 32 位值
 */
static uint32_t read_flash(uint32_t addr)
{
	return *(volatile uint32_t *)addr;
}

/**
 * @brief  用户 Flash 保存参数接口（扇区擦除 + 按字写入）
 * @param  data:    要写入的数据缓冲区指针（32 位对齐）
 * @param  datalen: 写入的字数（每字 4 字节）
 * @retval 1=成功, 0=失败
 * @note   该函数会先擦除 Sector 11 整个扇区，再按字编程写入
 *          擦除前需调用 HAL_FLASH_Unlock() 解锁 Flash 控制器
 */
uint8_t User_Flash_SaveParam(uint32_t* data, uint16_t datalen)
{
	HAL_FLASH_Unlock();                              /* 解锁 Flash 控制器 */

	/* 配置并执行扇区擦除 */
	uint32_t SectorError;
	FLASH_EraseInitTypeDef EraseInitStruct;

	EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS; /* 扇区擦除模式 */
	EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;         /* 电压范围 2.7~3.6V */
	EraseInitStruct.Sector       = stmflash_get_flash_sector(FLASH_SAVE_ADDR);
	EraseInitStruct.NbSectors    = 1;                       /* 擦除 1 个扇区 */
	HAL_StatusTypeDef eraseStatus = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
	if (eraseStatus != HAL_OK)
	{
		printf("Flash erase failed at sector: %lu, error code: %d\r\n",
		       SectorError, HAL_FLASH_GetError());
		HAL_FLASH_Lock();
		return 0;
	}

	/* 按字（32 位）依次写入数据 */
	uint32_t writeaddr = FLASH_SAVE_ADDR;
	for (uint16_t i = 0; i < datalen; i++)
	{
		HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, writeaddr, data[i]);
		if (status != HAL_OK)
		{
			printf("Flash write failed at address: 0x%08X, error: %d\r\n",
			       writeaddr, HAL_FLASH_GetError());
			HAL_FLASH_Lock();
			return 0;
		}
		writeaddr += 4;                              /* 地址递增 4 字节 */
	}

	HAL_FLASH_Lock();                                /* 锁定 Flash 控制器 */
	return 1;
}

/**
 * @brief  用户 Flash 读取参数接口
 * @param  p:       读取数据存放缓冲区指针
 * @param  datalen: 读取的字数（每字 4 字节）
 * @note   直接从 FLASH_SAVE_ADDR 地址开始顺序读取
 *          未写入过的 Flash 地址默认值为 0xFFFFFFFF（擦除态）
 */
void User_Flash_ReadParam(uint32_t* p, uint16_t datalen)
{
	uint16_t i = 0;
	uint32_t addr = FLASH_SAVE_ADDR;
	for (i = 0; i < datalen; i++)
	{
		p[i] = read_flash(addr);
		addr += 4;
	}
}
