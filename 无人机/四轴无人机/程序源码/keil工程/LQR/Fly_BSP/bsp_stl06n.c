/**
 * @file    bsp_stl06n.c
 * @brief   STL06N 激光雷达数据处理驱动
 * @note    STL06N 雷达: 360° 扫描, 帧头 0x54 0x2C, CRC8 校验
 *          每帧 12 个测距点, 角分辨率约 0.72°
 *          总缓冲区 500 个采样点（360° / 0.72° = 500）
 *          对重复角度取多点平均滤波, 满一圈通过队列发送
 */

#include "bsp_stl06n.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "bsp_RTOSdebug.h"

/* 点云距离阈值: 相邻两点距离差 < 100mm 视为可合并的点 */
#define THRESHOLD_DISTANCE 100          /* 单位 mm */

/* 调试等级: 0=关闭, 1=基本, >1=详细（频率/丢帧统计） */
#define DEBUG_LEVEL 1

/* 外部队列句柄: 用于激光雷达数据传递到处理任务 */
extern QueueHandle_t g_xQueueLidarBuffer;

/* DMA 原始数据缓冲区 */
OriData_STL06N_t DMAbuf_ori_stl06n = { 0 };

/* CRC8 查表（多项式: x^8 + x^5 + x^4 + 1） --------------------------------*/
static const uint8_t CrcTable[256] =
{
	0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
	0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
	0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
	0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
	0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
	0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
	0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
	0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
	0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
	0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
	0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
	0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
	0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
	0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
	0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
	0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
	0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
	0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
	0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
	0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
	0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
	0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

/**
 * @brief  计算 CRC8 校验值（查表法）
 * @param  p:   数据缓冲区指针
 * @param  len: 数据长度
 * @retval 8 位 CRC 校验值
 */
uint8_t CalCRC8(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;
	uint16_t i;
	for (i = 0; i < len; i++)
	{
		crc = CrcTable[(crc ^ *p++) & 0xff];
	}
	return crc;
}

/* 调试工具实例 */
RtosDebugPrivateVar privdebug = { 0 };
pRtosDebugInterface_t debug = &RTOSTaskDebug;

/* STL06N 雷达启动/停止/切换（注释掉，保留备用） -------------------------------*/
/*
void Stl06NLidar_Start(void)
{
	uint8_t StartLidar[8] = { 0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E };
	HAL_UART_Transmit(&huart2, StartLidar, 8, 200);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, DMAbuf_ori_stl06n.Buf, userconfig_STL06N_DMABUF_LEN);
}

void Stl06NLidar_Stop(void)
{
	HAL_UART_DMAStop(&huart2);
	uint8_t StopLidar[8] = { 0x54, 0xA1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x4A };
	HAL_UART_Transmit(&huart2, StopLidar, 8, 200);
}

void Stl06NLidar_Toggle(void)
{
	static uint8_t flag = 0;
	flag = !flag;
	if (flag) Stl06NLidar_Start();
	else      Stl06NLidar_Stop();
}
*/

/**
 * @brief  两点云数据合并（取距离较近者或平均值）
 * @param  a, b: 两个距离值（mm）
 * @retval 合并后的距离值
 * @note   若两点间距 < 阈值(100mm)，取平均值；否则取较小值
 */
static uint16_t process_twoPointCloud(int a, int b) {
	if (abs(a - b) < THRESHOLD_DISTANCE) {
		return roundf((a + b) / 2.0f);    /* 距离接近，取平均值 */
	} else {
		return (a < b) ? a : b;           /* 距离差距大，取较小值（更近） */
	}
}

/**
 * @brief  三点云数据合并（取最接近的两点进行合并）
 * @param  a, b, c: 三个距离值（mm）
 * @retval 合并后的距离值
 * @note   找出三组点对中差值最小的那对，然后调用两点合并
 */
static uint16_t process_threePointCloud(int a, int b, int c) {
	int diff_ab = abs(a - b);
	int diff_ac = abs(a - c);
	int diff_bc = abs(b - c);

	if (diff_bc <= diff_ac && diff_bc <= diff_ab) {
		return process_twoPointCloud(b, c);
	}
	else if (diff_ab <= diff_ac && diff_ab <= diff_bc) {
		return process_twoPointCloud(a, b);
	}
	else if (diff_ac <= diff_ab && diff_ac <= diff_bc) {
		return process_twoPointCloud(a, c);
	}
	else
		return a;
}

/**
 * @brief  点云数组滤波合并（根据填充数量选择最优合并策略）
 * @param  arr:  距离值数组
 * @param  size: 数组中的有效值数量（2 或 3）
 * @retval 合并后的距离值
 */
static uint16_t process_PointCloudarray(uint16_t *arr, uint8_t size) {
	if (size == 2) {
		return process_twoPointCloud(arr[0], arr[1]);
	} else if (size == 3) {
		return process_threePointCloud(arr[0], arr[1], arr[2]);
	} else {
		return arr[0];                   /* 无效数量，返回第一个值 */
	}
}

#include "usart.h"

/**
 * @brief  STL06N 雷达数据帧解析与点云处理（角分辨率 0.72°）
 * @param  recv:      接收到的原始字节缓冲区
 * @param  bufferLen: 缓冲区长度
 * @note   状态机: WaitHead → GetData → CrcCheck
 *          帧头: 0x54 0x2C
 *          角分辨率: 0.72°（36000/500），一帧 12 个测距点
 *          重复角度数据合并滤波 → 满一圈通过队列发送
 */
void Stl06N_BufferHandle(uint8_t* recv, uint8_t bufferLen)
{
	/* 一帧雷达数据 */
	static Stl06NFrameTypeDef stl06n_oneframe = { 0 };

	/* 一整圈（360°）雷达数据 */
	static Stl06NAngleBuffer_t stl06n_onecircle = { 0 };

	const uint8_t stl06frameLen = sizeof(Stl06NFrameTypeDef); /* 一帧字节长度 */
	static uint8_t recvbuffer[sizeof(Stl06NFrameTypeDef)] = { 0 };
	static uint8_t recvcounts = 0;

	/* 接收状态机 */
	enum {
		WaitHead = 0,                    /* 等待帧头 0x54 0x2C */
		GetData  = 1,                    /* 接收数据体 */
		CrcCheck = 2,                    /* CRC8 校验 */
	};
	static uint8_t sm = WaitHead;

	uint8_t ReadyFlag = 0;               /* 数据就绪标志 */

	/* 逐字节处理接收缓冲区 */
	for (uint8_t i = 0; i < bufferLen; i++)
	{
		switch (sm)
		{
			case WaitHead:
				/* 检测帧头 0x54 0x2C */
				if (i + 1 < bufferLen && recv[i] == 0x54 && recv[i + 1] == 0x2C)
				{
					recvbuffer[0] = 0x54;
					recvcounts = 1;
					sm = GetData;
				}
				break;
			case GetData:
				recvbuffer[recvcounts++] = recv[i];
				if (recvcounts == stl06frameLen - 1)
				{
					sm = CrcCheck;       /* 收齐数据体，进入校验阶段 */
				}
				break;
			case CrcCheck:
				/* CRC8 校验 */
				if (CalCRC8(recvbuffer, recvcounts) == recv[i])
				{
					recvbuffer[recvcounts] = recv[i];
					memcpy(&stl06n_oneframe, recvbuffer, sizeof(Stl06NFrameTypeDef));
					ReadyFlag = 1;
				}
				else
					printf("crc8 error!\r\n");

				recvcounts = 0;
				sm = WaitHead;
				break;
		}
	}

	/* 成功解析到一帧数据 */
	if (ReadyFlag == 1)
	{
		uint16_t angleIncr = 0;

		/* 计算角度增量: 一帧 12 个点, 角度跨度约 7.9° */
		if (stl06n_oneframe.start_angle > stl06n_oneframe.end_angle)
			angleIncr = stl06n_oneframe.end_angle + 36000 - stl06n_oneframe.start_angle;
		else
			angleIncr = stl06n_oneframe.end_angle - stl06n_oneframe.start_angle;

		/* 分辨率 = 角度跨度 / 11 (12 个点有 11 个间隔) ≈ 0.72° */
		angleIncr /= 11;

		/* 索引变量 */
		uint16_t index = 0;
		static uint16_t lastindex = 0;
		uint16_t lostindex = 0;

		uint16_t distancelist[5] = { 0 };  /* 重复角度的距离缓存（最多 5 个同角度点） */
		uint8_t  filtersize = 0;           /* 缓存有效个数 */

#if DEBUG_LEVEL > 1
		static uint16_t debug1 = 0;        /* 正常点计数 */
		static uint16_t debug2 = 0;        /* 重复点计数 */
		static uint16_t freqcount = 0;     /* 频率计时 */
#endif

		for (uint8_t j = 0; j < 12; j++)
		{
			/* 当前点角度 */
			uint16_t angle_temp = stl06n_oneframe.start_angle + j * angleIncr;
			if (angle_temp > 36000) angle_temp -= 36000;

			/* 角度 → 数组下标（分辨率 0.72°） */
			index = round(angle_temp / Stl06n_AngleRatio);
			index = index % Stl06n_LidarDistanceBufferLen;

			/* 检查是否完成一整圈扫描 */
			if (index < lastindex)
			{
#if DEBUG_LEVEL > 1
				printf("1: %d\r\n", debug1);
				printf("2: %d\r\n", debug2);
				printf("all:%d\r\n", debug1 + debug2);
				debug1 = 0, debug2 = 0;

				/* 频率计算 */
				uint8_t freq = 0;
				if (stl06n_oneframe.timestamp < freqcount)
					freq = stl06n_oneframe.timestamp + 30000 - freqcount;
				else
					freq = stl06n_oneframe.timestamp - freqcount;
				printf("time:%d\r\n\r\n", freq);
				freqcount = stl06n_oneframe.timestamp;
#endif

				/* 一圈数据完整，通过队列发送到处理任务 */
				xQueueSend(g_xQueueLidarBuffer, &stl06n_onecircle, 0);

				/* 清空一圈数据缓冲区 */
				memset(&stl06n_onecircle, 0, sizeof(Stl06NAngleBuffer_t));

				lostindex = (index + Stl06n_LidarDistanceBufferLen) - lastindex;
			}
			else
				lostindex = index - lastindex;

			/* 正常步进（新角度） */
			if (lostindex != 0)
			{
#if DEBUG_LEVEL > 1
				debug1++;
#endif
				if (filtersize != 0)
				{
					/* 先处理之前的重复点合并 */
					distancelist[filtersize] = stl06n_onecircle.buffer[Stl06n_LidarDistanceBufferLen - lastindex].distance;
					filtersize++;

					stl06n_onecircle.buffer[Stl06n_LidarDistanceBufferLen - lastindex].distance =
					    process_PointCloudarray(distancelist, filtersize);
					filtersize = 0;
				}

				/* 写入新角度数据 */
				stl06n_onecircle.buffer[Stl06n_LidarDistanceBufferLen - index].distance = stl06n_oneframe.point[j].distance;
				stl06n_onecircle.buffer[Stl06n_LidarDistanceBufferLen - index].peak     = stl06n_oneframe.point[j].peak;
			}
			/* 重复角度：缓存起来等待合并处理 */
			else if (lostindex == 0)
			{
#if DEBUG_LEVEL > 1
				debug2++;
#endif
				distancelist[filtersize] = stl06n_oneframe.point[j].distance;
				filtersize++;
			}

			lastindex = index;
		}

		/* 丢帧检测 */
		static uint16_t lasttimestamp = 0;
		uint16_t difftime = 0;
		if (stl06n_oneframe.timestamp < lasttimestamp)
			difftime = stl06n_oneframe.timestamp + 30000 - lasttimestamp;
		else
			difftime = stl06n_oneframe.timestamp - lasttimestamp;

		if (difftime > 3)
		{
			printf("losttime:%d\r\n", difftime);
		}
		lasttimestamp = stl06n_oneframe.timestamp;
	}
}
