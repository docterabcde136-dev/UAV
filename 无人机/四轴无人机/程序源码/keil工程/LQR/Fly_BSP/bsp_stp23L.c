/**
 * @file    bsp_stp23L.c
 * @brief   STP23L 激光测距模块驱动（串口逐字节解析）
 * @note    每字节通过回调函数 stp23L_getdistance_callback() 处理
 *          协议帧头: 0xAA ×4 + 0x00 0x02 0x00 0x00 0xB8 0x00（共 10 字节）
 *          每帧含 12 个测距点（距离 + 噪声 + 强度 + 置信度 + 积分 + 温度补偿）
 *          输出: 12 点平均距离（单位 m）
 */

#include "bsp_stp23L.h"
#include <stdio.h>
#include <string.h>

/* DMA 原始数据缓冲区（由 DMA 填充） */
OriData_STP23L_t DMABuf_oridata_stp23L;

/* STP23L 高度/距离全局变量（单位 m，只读） */
float g_readonly_distance = 0;

/* 单点激光点云数据结构（1 字节对齐） */
#pragma pack(1)
typedef struct {
	int16_t distance;                    /* 距离（mm） */
	uint16_t noise;                      /* 环境噪声 */
	uint32_t peak;                       /* 信号强度 */
	uint8_t  confidence;                 /* 置信度 */
	uint32_t intg;                       /* 积分值 */
	int16_t reftof;                      /* 温度补偿参考值 */
} LidarPointTypedef;
#pragma pack()

/* 一帧 STP23L 数据（12 个点 + 时间戳） */
#pragma pack(1)
typedef struct {
	LidarPointTypedef PointCloud[12];    /* 12 个测距点 */
	uint32_t timestamp;                  /* 时间戳（ms） */
} STP23L_OneFrame_t;
#pragma pack()

/* 数据接收相关变量 */
static STP23L_OneFrame_t stp23L_frame;                /* 解析后的一帧数据 */
static const uint8_t stp23L_BufLen = sizeof(stp23L_frame);
static uint8_t stp23L_buf[sizeof(STP23L_OneFrame_t)];  /* 原始数据缓冲区 */
static uint8_t stp23L_counts = 0;                       /* 接收字节计数 */

/**
 * @brief  STP23L 距离数据逐字节解析回调
 * @param  recv: 当前接收到的单字节
 * @param  dis:  输出参数，计算后的平均距离（单位 m）
 * @note   状态机流程: Wait_HEAD → Handle_Ddata → END_DATA
 *          帧头: 0xAA ×4 + 0x00 0x02 0x00 0x00 0xB8 0x00（10 字节）
 *          校验码初始值: 0xBA（= 0x02 + 0xB8，排除 4 个 0xAA）
 *          输出为 12 个测距点的算术平均距离值
 */
void stp23L_getdistance_callback(uint8_t recv, float* dis)
{
	static uint8_t checkcode = 0;        /* 累加校验码 */

	/* 状态机定义 */
	enum {
		Wait_HEAD   = 0,                 /* 等待帧头 */
		Handle_Ddata,                    /* 处理数据 */
		END_DATA,                        /* 校验处理 */
	};

	static uint8_t state_machine = Wait_HEAD;

	switch (state_machine)
	{
		case Wait_HEAD:
		{
			static uint8_t headlen = 0;
			/* 固定帧头共 10 字节:
			 * 0xAA 0xAA 0xAA 0xAA 0x00 0x02 0x00 0x00 0xB8 0x00 */
			headlen++;
			if (10 == headlen)
			{
				headlen    = 0;
				state_machine = Handle_Ddata;
				/* 校验码初值 = 0x02 + 0xB8（跳过前 4 个 0xAA） */
				checkcode  = 0xBA;
			}
			break;
		}
		case Handle_Ddata:
		{
			/* 累加校验码（逐字节求和） */
			checkcode += recv;

			/* 存储数据 */
			stp23L_buf[stp23L_counts++] = recv;

			/* 收满一帧数据，进入校验阶段 */
			if (stp23L_BufLen == stp23L_counts)
			{
				stp23L_counts = 0;
				state_machine = END_DATA;
			}
			break;
		}
		case END_DATA:
		{
			if (recv == checkcode)
			{
				/* 校验通过：拷贝数据到结构体 */
				memcpy(&stp23L_frame, stp23L_buf, stp23L_BufLen);

				/* 计算 12 个测距点的平均距离 */
				float alldist = 0;
				for (uint8_t i = 0; i < 12; i++)
					alldist += stp23L_frame.PointCloud[i].distance;
				*dis = alldist / 12.0f;  /* 平均值 */
				*dis /= 1000.0f;          /* mm → m */
			}
			else
			{
				/* 校验失败，丢弃此帧 */
			}

			checkcode = 0;
			state_machine = Wait_HEAD;
			break;
		}
	}
}
