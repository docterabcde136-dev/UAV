/**
 * @file    bsp_opticalflow.c
 * @brief   光流传感器驱动（串口通信）
 * @note    通过 UART4 接收光流模块的位移数据
 *          协议帧格式: 0xFE + DX_L + DX_H + DY_L + DY_H + SUM + Quality + 0xAA（9 字节）
 *          DX/DY: 像素级位移增量（int16_t），Quality > 30 视为有效数据
 *          累加位移增量获得绝对位置
 */

#include "bsp_opticalflow.h"
#include "usart.h"

/* 光流传感器使用的串口（UART4） */
static UART_HandleTypeDef* opticalflow_serial = &huart4;

/* 位移数据 -------------------------------------------------------------------*/
static PositionType_t postiton    = { 0 }; /* 绝对位置（累积位移） */
static PositionType_t postitondot = { 0 }; /* 增量位移（上一帧的增量） */

/* 串口接收缓冲区长度（协议帧为 9 字节） */
#define bufferLen 10
static uint8_t serialbuffer[bufferLen];

/**
 * @brief  启动光流传感器 DMA 空闲中断接收
 * @note   使用 UART Idle DMA 模式，收到完整一帧数据后自动触发回调
 */
static void start_dma_recv(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(opticalflow_serial, serialbuffer, bufferLen);
}

/**
 * @brief  获取当前绝对位置（累积位移）
 * @retval 包含 x, y 的 PositionType_t 结构体
 */
static PositionType_t returnPos(void)
{
	return postiton;
}

/**
 * @brief  获取上一帧位移增量
 * @retval 包含 x, y 增量的 PositionType_t 结构体
 */
static PositionType_t returnPosDot(void)
{
	return postitondot;
}

/**
 * @brief  光流传感器数据解析回调（在 UART Idle 中断中调用）
 * @param  size: 接收到的数据长度
 * @note   帧格式校验：帧头 0xFE + 帧尾 0xAA + 长度 9 + 累加和校验
 *         仅当图像质量 Quality > 30 时认为数据有效
 *         坐标变换：x 轴取反（适配安装方向），y 轴保持
 */
static void Handle_opticalflow_buffer(uint16_t size)
{
	/* 帧格式校验：长度 9、帧头 0xFE、帧尾 0xAA */
	if (size == 9 && serialbuffer[0] == 0xFE && serialbuffer[8] == 0xAA)
	{
		/* 累加和校验（DX_L + DX_H + DY_L + DY_H 的低 8 位应等于 SUM） */
		if (((serialbuffer[2] + serialbuffer[3] + serialbuffer[4] + serialbuffer[5]) & 0xff) == serialbuffer[6])
		{
			/* 图像质量检查：Quality > 30 为有效数据 */
			if (serialbuffer[7] > 30)
			{
				/* 提取位移增量（小端序：低字节在前）并转换坐标 */
				postitondot.x = -((short)(serialbuffer[3] << 8 | serialbuffer[2]));
				postitondot.y =   (short)(serialbuffer[5] << 8 | serialbuffer[4]);

				/* 累加到绝对位置 */
				postiton.x += postitondot.x;
				postiton.y += postitondot.y;
			}
		}
	}
}

/**
 * @brief  清零当前位置（将当前位置设为坐标原点）
 */
static void ClearPostition(void)
{
	postiton.x = 0;
	postiton.y = 0;
}

/* 光流接口结构体实例 ---------------------------------------------------------*/
/* 通过函数指针向上层 APP 暴露统一的光流传感器操作接口 */

OpticalFlowInterface_t UserOpticalFlow = {
	.StartRecvData = start_dma_recv,           /* 启动 DMA 接收 */
	.getpos        = returnPos,                /* 获取绝对位置 */
	.getpos_dot    = returnPosDot,             /* 获取位移增量 */
	.HandleRecvData = Handle_opticalflow_buffer, /* 数据解析回调 */
	.SetZeroPoint  = ClearPostition            /* 设置坐标零点 */
};
