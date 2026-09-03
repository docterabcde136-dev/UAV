/**
 * @file    bsp_N10.c
 * @brief   N10 激光雷达数据处理驱动
 * @note    N10 雷达: 360° 扫描, 角分辨率 0.8°, 共 450 个采样点/圈, 帧率 10Hz
 *          每帧含 16 个测距点（起始角度 + 15 个等角度间隔点）
 *          CRC 累加和校验, 角分辨率补偿, 漏点插值填补
 */

#include "bsp_N10.h"
#include "stdio.h"
#include "math.h"

/* 单点激光点云数据结构 -------------------------------------------------------*/
typedef struct {
	uint16_t distance;                   /* 距离值（mm） */
	uint8_t  peak;                       /* 信号强度 */
} N10_OnePointCloud_t;

/* DMA 原始数据缓冲区（只读，由 DMA 填充） */
OriData_N10_t DMABuf_oridata_N10;

/* 一帧 360° 点云数据（450 个采样点 × 0.8° = 360°） */
N10_OnePointCloud_t N10_PointCloud_Data[450];

/**
 * @brief  N10 激光雷达数据帧处理函数
 * @param  oribuf: 原始数据帧缓冲区（58 字节）
 * @note   帧格式: 帧头(5B) + 起始角度(2B) + 16×{距离(2B)+强度(1B)} + 结束角度(2B) + CRC(1B)
 *         对漏点（lostindex≥3）跳过填补（视为不可靠数据）
 *         对重复点取平均，对单漏点做线性插值
 */
void N10_DataHandle(uint8_t* oribuf)
{
	/* CRC 累加和校验（前 57 字节求和 == 第 58 字节） */
	uint8_t CrcVal = 0;
	for (uint8_t i = 0; i < 57; i++) CrcVal += oribuf[i];

	if (CrcVal != oribuf[57])
	{
		/* CRC 校验失败处理（暂无） */
	}

	/* 计算本帧的起始角度、结束角度和角度增量 */
	float startangle, endangle, angleIncr;

	startangle = (float)((uint16_t)(oribuf[5] << 8 | oribuf[6])) / 100.f;
	endangle   = (float)((uint16_t)(oribuf[55] << 8 | oribuf[56])) / 100.f;

	/* 角度增量 = 角度跨度 / 15（16 个点之间有 15 个间隔） */
	if (startangle > endangle)
		angleIncr = (endangle + 360.f - startangle) / 15.0f;
	else
		angleIncr = (endangle - startangle) / 15.0f;

	/* 索引变量 */
	uint16_t index = 0;                  /* 当前角度对应的点云数组下标 */
	uint16_t lostindex = 0;              /* 与上一采样点的下标间距 */
	static uint16_t lastindex = 0;       /* 上一采样点下标（跨帧保持） */

	float tmpanlge;

	for (uint8_t i = 0; i < 16; i++)
	{
		tmpanlge = startangle + (i * angleIncr);
		if (tmpanlge > 360.0f) tmpanlge -= 360.0f;

		/* 角度 → 数组下标: index = angle / 0.8° */
		index = round(tmpanlge / 0.8f);
		index = index % 450;             /* 防止越界 */

		/* 检测是否完成一整圈扫描 */
		if (index < lastindex)
		{
			/* 一圈数据已就绪，可以通知处理 */
			/* 示例: 频率 ≈ 10Hz (N10 标称帧率) */
			// printf("%d\r\n", debug->UpdateFreq(&debugpriv));
			lostindex = (index + 450) - lastindex;
		}
		else
		{
			lostindex = index - lastindex;
		}

		/* 提取当前测距点的距离和强度 */
		N10_OnePointCloud_t TmpPonit = { 0 };
		TmpPonit.distance = oribuf[7 + (i * 3)] << 8 | oribuf[8 + (i * 3)];
		TmpPonit.peak     = oribuf[9 + (i * 3)];

		/* 根据下标间距采用不同的赋值策略 */
		if (1 == lostindex || lostindex >= 3)
		{
			/* 连续点（间距=1）或丢点过多（≥3，不可靠，直接覆盖） */
			N10_PointCloud_Data[index].distance = TmpPonit.distance;
			N10_PointCloud_Data[index].peak     = TmpPonit.peak;
		}
		else if (0 == lostindex)
		{
			/* 重复采样：取平均值 */
			N10_PointCloud_Data[index].distance = (N10_PointCloud_Data[index].distance + TmpPonit.distance) / 2;
			N10_PointCloud_Data[index].peak    = (N10_PointCloud_Data[index].peak     + TmpPonit.peak) / 2;
		}
		else if (2 == lostindex)
		{
			/* 恰好漏 1 个点：线性插值填补漏点 */
			N10_PointCloud_Data[index].distance = TmpPonit.distance;
			N10_PointCloud_Data[index].peak     = TmpPonit.peak;

			N10_PointCloud_Data[(lastindex + 1) % 450].distance =
			    (N10_PointCloud_Data[index].distance + N10_PointCloud_Data[lastindex].distance) / 2;
			N10_PointCloud_Data[(lastindex + 1) % 450].peak =
			    (N10_PointCloud_Data[index].peak     + N10_PointCloud_Data[lastindex].peak) / 2;
		}

		/* 更新上一采样点下标 */
		lastindex = index;
	}
}
