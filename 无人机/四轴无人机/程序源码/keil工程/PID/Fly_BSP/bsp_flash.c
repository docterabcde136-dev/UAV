/**
 * ============================================================
 * bsp_flash.c — STM32F4 内部Flash参数存储驱动
 * ============================================================
 * 功能: 使用片内Flash Sector 11 (0x080E0000) 存储用户参数
 *   - User_Flash_SaveParam:  掉电保存 (需先擦除再写入)
 *   - User_Flash_ReadParam:  读取已保存的参数
 * 注意: Flash写入必须在无中断干扰的环境下进行
 * ============================================================
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal_flash.h"

//F405RG 共1024kb FLash. 将数据保存在最后一个扇区11. 0x8000000+(1024-128)*1024 = 0x80E000
#define FLASH_SAVE_ADDR 0x080E0000

/* FLASH 扇区的起始地址 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t )0x08000000)     /* 扇区0起始地址, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t )0x08004000)     /* 扇区1起始地址, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t )0x08008000)     /* 扇区2起始地址, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t )0x0800C000)     /* 扇区3起始地址, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t )0x08010000)     /* 扇区4起始地址, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t )0x08020000)     /* 扇区5起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t )0x08040000)     /* 扇区6起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t )0x08060000)     /* 扇区7起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t )0x08080000)     /* 扇区8起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t )0x080A0000)     /* 扇区9起始地址, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t )0x080C0000)     /* 扇区10起始地址,128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t )0x080E0000)     /* 扇区11起始地址,128 Kbytes */

/**
获取某个地址所在的flash扇区
faddr   : flash地址
0~11, 即addr所在的扇区
*/
static uint8_t stmflash_get_flash_sector(uint32_t addr)
{
    if (addr < ADDR_FLASH_SECTOR_1)return 0;
    else if (addr < ADDR_FLASH_SECTOR_2)return 1;
    else if (addr < ADDR_FLASH_SECTOR_3)return 2;
    else if (addr < ADDR_FLASH_SECTOR_4)return 3;
    else if (addr < ADDR_FLASH_SECTOR_5)return 4;
    else if (addr < ADDR_FLASH_SECTOR_6)return 5;
    else if (addr < ADDR_FLASH_SECTOR_7)return 6;
    else if (addr < ADDR_FLASH_SECTOR_8)return 7;
    else if (addr < ADDR_FLASH_SECTOR_9)return 8;
    else if (addr < ADDR_FLASH_SECTOR_10)return 9;
    else if (addr < ADDR_FLASH_SECTOR_11)return 10;

    return 11;
}

static uint32_t read_flash(uint32_t addr)
{
	return  *(volatile uint32_t *)addr;
}


#include <stdio.h>
#include <string.h>
//对外接口,保存参数到Flash.占用第11扇区空间
uint8_t User_Flash_SaveParam(uint32_t* data,uint16_t datalen)
{
	HAL_FLASH_Unlock();//解锁

    // 擦除目标扇区
    uint32_t SectorError;
    FLASH_EraseInitTypeDef EraseInitStruct;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;//部分擦除
    EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;     //电压区间2.7~3.6V
    EraseInitStruct.Sector = stmflash_get_flash_sector(FLASH_SAVE_ADDR); //擦除的扇区
    EraseInitStruct.NbSectors = 1;    //需要擦除的扇区数量
    HAL_StatusTypeDef eraseStatus = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError); //开始擦除
    if (eraseStatus != HAL_OK) /* Flash错误调试方法 */
    {
        printf("Flash erase failed at sector: %u, error code: %d\r\n", SectorError, HAL_FLASH_GetError());
        HAL_FLASH_Lock();
        return 0;
    }
	
	uint32_t writeaddr = FLASH_SAVE_ADDR;
	for(uint16_t i=0;i<datalen;i++)
	{
		HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,writeaddr,data[i]);	
        if (status != HAL_OK) /* Flash错误调试方法 */
        {
            printf("Flash write failed at address: 0x%08X, error: %d\r\n", writeaddr, HAL_FLASH_GetError());
            HAL_FLASH_Lock();
            return 0;
        }

		writeaddr += 4;
	}
	
	HAL_FLASH_Lock();//上锁
	
	return 1;
}

//对外接口,读取参数到Flash.占用第7扇区空间
void User_Flash_ReadParam(uint32_t* p,uint16_t datalen)
{
	uint16_t i=0;
	uint32_t addr = FLASH_SAVE_ADDR;
	for(i=0;i<datalen;i++)
	{
		p[i] = read_flash(addr);
		addr += 4;
	}
}
