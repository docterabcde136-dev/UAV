/**
 * @file    bsp_datascope.c
 * @brief   DataScope 虚拟示波器串口通信协议驱动
 * @note    通过串口将 float 数据发送到上位机 DataScope 软件进行波形显示
 *          协议帧格式: $ + 4 字节 float 数据 * N 个通道 + 帧尾长度标识
 *          最大支持 10 个通道同时显示
 * @see     http://shop114407458.taobao.com/ （上位机来源）
 */

#include "bsp_datascope.h"
#include "usart.h"

/* DataScope 串口发送缓冲区 ---------------------------------------------------*/
static unsigned char DataScope_OutPut_Buffer[42] = {0}; /* 最多 10 通道 × 4 字节 + 帧头帧尾 */

/**
 * @brief  将 float 值按小端序写入缓冲区的指定位置
 * @param  target: float 变量的地址（通过指针强转为 4 字节）
 * @param  buf:    目标发送缓冲区
 * @param  beg:    写入起始偏移位置
 * @note   直接按字节复制 float 的二进制表示，无需转换
 */
static void Float2Byte(float *target, unsigned char *buf, unsigned char beg)
{
	unsigned char *point;
	point = (unsigned char*)target;        /* 获取 float 的字节级地址 */
	buf[beg]     = point[0];              /* 小端序：低字节在前 */
	buf[beg + 1] = point[1];
	buf[beg + 2] = point[2];
	buf[beg + 3] = point[3];
}

/**
 * @brief  生成 DataScope 协议帧尾并返回总发送长度
 * @param  Channel_Number: 通道数（1~10）
 * @retval 需要发送的总字节数（0 表示参数错误）
 * @note   帧头固定为 '$'，帧尾为对应通道数的长度标识
 *          每个通道占用 4 字节（一个 float）
 */
static unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ((Channel_Number > 10) || (Channel_Number == 0))
	{
		return 0;                          /* 通道数超限 */
	}
	else
	{
		DataScope_OutPut_Buffer[0] = '$';  /* 帧头标识 */

		switch (Channel_Number)
		{
			case 1:  DataScope_OutPut_Buffer[5]  = 5;  return 6;
			case 2:  DataScope_OutPut_Buffer[9]  = 9;  return 10;
			case 3:  DataScope_OutPut_Buffer[13] = 13; return 14;
			case 4:  DataScope_OutPut_Buffer[17] = 17; return 18;
			case 5:  DataScope_OutPut_Buffer[21] = 21; return 22;
			case 6:  DataScope_OutPut_Buffer[25] = 25; return 26;
			case 7:  DataScope_OutPut_Buffer[29] = 29; return 30;
			case 8:  DataScope_OutPut_Buffer[33] = 33; return 34;
			case 9:  DataScope_OutPut_Buffer[37] = 37; return 38;
			case 10: DataScope_OutPut_Buffer[41] = 41; return 42;
		}
	}
	return 0;
}

/**
 * @brief  将单个通道的 float 数据写入发送缓冲区
 * @param  Data:    要发送的 float 数据值
 * @param  Channel: 通道号（1~10）
 * @note   每个通道在缓冲区中有固定的偏移位置
 *         通道 1 → 字节 1~4, 通道 2 → 字节 5~8, ...
 */
static void DataScope_Get_Channel_Data(float Data, unsigned char Channel)
{
	if ((Channel > 10) || (Channel == 0)) return;

	switch (Channel)
	{
		case 1:  Float2Byte(&Data, DataScope_OutPut_Buffer, 1);  break;
		case 2:  Float2Byte(&Data, DataScope_OutPut_Buffer, 5);  break;
		case 3:  Float2Byte(&Data, DataScope_OutPut_Buffer, 9);  break;
		case 4:  Float2Byte(&Data, DataScope_OutPut_Buffer, 13); break;
		case 5:  Float2Byte(&Data, DataScope_OutPut_Buffer, 17); break;
		case 6:  Float2Byte(&Data, DataScope_OutPut_Buffer, 21); break;
		case 7:  Float2Byte(&Data, DataScope_OutPut_Buffer, 25); break;
		case 8:  Float2Byte(&Data, DataScope_OutPut_Buffer, 29); break;
		case 9:  Float2Byte(&Data, DataScope_OutPut_Buffer, 33); break;
		case 10: Float2Byte(&Data, DataScope_OutPut_Buffer, 37); break;
	}
}

/* DataScope 使用的调试串口（UART4） */
UART_HandleTypeDef* DataScope_Seiral = &huart4;

/**
 * @brief  DataScope 上位机数据发送接口
 * @param  showdata: float 数据数组指针
 * @param  showlen:  通道数量（1~10，超出则直接返回）
 * @note   使用 DMA 方式发送以减少 CPU 占用
 *          也可切换为阻塞发送方式（注释掉的 HAL_UART_Transmit）
 */
static void DataScope_ShowData(float* showdata, uint8_t showlen)
{
	if (showlen > 10) return;              /* 数据量超限 */

	for (uint8_t i = 0; i < showlen; i++)
	{
		DataScope_Get_Channel_Data(showdata[i], i + 1); /* 通道从 1 开始编号 */
	}
	uint8_t sendcount = DataScope_Data_Generate(showlen);

	/* 使用 DMA 发送（需在 CubeMX 中配置好 DMA） */
	HAL_UART_Transmit_DMA(DataScope_Seiral, DataScope_OutPut_Buffer, sendcount);
	/* 备选阻塞发送方式: */
	/* HAL_UART_Transmit(DataScope_Seiral, DataScope_OutPut_Buffer, sendcount, 200); */
}

/* DataScope 接口结构体实例 ---------------------------------------------------*/

DataScopeInterface_t UserDataScope = {
	.show = DataScope_ShowData,            /* 发送数据到上位机显示 */
};
