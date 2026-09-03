/**
 * @file    bsp_LC307.c
 * @brief   LC307 / STP23L 激光测距模块驱动（串口通信）
 * @note    使用 USART3 与激光模块通信，协议: 0xAA 0xAB 帧头 + 指令 + XOR 校验
 *          支持 DMA 空闲中断接收数据帧（28 字节）
 *          初始化过程分 3 步: 握手 → 写配置表 → 关闭配置
 *          输出: 光流位移 speed[0]=X方向速度, speed[1]=Y方向速度 (m/s)
 */

#include "bsp_LC307.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* 激光模块失联标志 ------------------------------------------------------------*/
uint8_t g_lost_pos_dev = 0;              /* 1=激光模块通信异常/未响应 */

/* 调试打印开关 ---------------------------------------------------------------*/
#define LC307_DEBUG_LEVEL 1              /* 0=关闭调试打印, 1=开启 */

#if (LC307_DEBUG_LEVEL > 0U)
#define LC307_Log(...) do { printf(__VA_ARGS__); } while (0)
#else
#define LC307_Log(...) do {} while (0)
#endif

/* 串口句柄 -------------------------------------------------------------------*/
UART_HandleTypeDef* OpticalFlowSerial = &huart3;

/* DMA 接收缓冲区（协议帧长度 28 字节） */
#define LC307_BufferLen 28
static uint8_t LC307OriBuffer[LC307_BufferLen];

/* 底层串口收发接口 ------------------------------------------------------------*/

/**
 * @brief  通过串口发送数据到激光模块
 */
static void LC307_SendData(uint8_t *data, uint16_t len)
{
	HAL_UART_Transmit(OpticalFlowSerial, data, len, 50);
}

/**
 * @brief  通过串口从激光模块接收数据
 */
static void LC307_RecvData(uint8_t *data, uint16_t len)
{
	HAL_UART_Receive(OpticalFlowSerial, data, len, 50);
}

/**
 * @brief  启动 DMA 空闲中断接收
 * @note   收到完整一帧或线路空闲后触发 LC307_Callback
 */
static void start_LC307_dma_recv(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(OpticalFlowSerial, LC307OriBuffer, LC307_BufferLen);
}

/**
 * @brief  停止 DMA 接收
 */
static void stop_dma_recv(void)
{
	HAL_UART_DMAStop(OpticalFlowSerial);
}

/* LC307/BF3901 传感器配置表（60Hz 模式）-------------------------------------*/
/* 以 {寄存器地址, 值} 对的形式组织，共 183 对参数 */
const static uint8_t tab_BF3901_60hz[] = {
0x12, 0x80, 0x11, 0x30, 0x1b, 0x06, 0x6b, 0x43, 0x12, 0x20, 0x3a, 0x00, 0x15, 0x02, 0x62, 0x81,
0x08, 0xa0, 0x06, 0x68, 0x2b, 0x20, 0x92, 0x25, 0x27, 0x97, 0x17, 0x01, 0x18, 0x79,
0x19, 0x00, 0x1a, 0xa0, 0x03, 0x00, 0x13, 0x00, 0x01, 0x13, 0x02, 0x20, 0x87, 0x16, 0x8c, 0x01,
0x8d, 0xcc, 0x13, 0x07, 0x33, 0x10, 0x34, 0x1d, 0x35, 0x46, 0x36, 0x40, 0x37, 0xa4,
0x38, 0x7c, 0x65, 0x46, 0x66, 0x46, 0x6e, 0x20, 0x9b, 0xa4, 0x9c, 0x7c, 0xbc, 0x0c, 0xbd, 0xa4,
0xbe, 0x7c, 0x20, 0x09, 0x09, 0x03, 0x72, 0x2f, 0x73, 0x2f, 0x74, 0xa7, 0x75, 0x12,
0x79, 0x8d, 0x7a, 0x00, 0x7e, 0xfa, 0x70, 0x0f, 0x7c, 0x84, 0x7d, 0xba, 0x5b, 0xc2, 0x76, 0x90,
0x7b, 0x55, 0x71, 0x46, 0x77, 0xdd, 0x13, 0x0f, 0x8a, 0x10, 0x8b, 0x20, 0x8e, 0x21,
0x8f, 0x40, 0x94, 0x41, 0x95, 0x7e, 0x96, 0x7f, 0x97, 0xf3, 0x13, 0x07, 0x24, 0x58, 0x97, 0x48,
0x25, 0x08, 0x94, 0xb5, 0x95, 0xc0, 0x80, 0xf4, 0x81, 0xe0, 0x82, 0x1b, 0x83, 0x37,
0x84, 0x39, 0x85, 0x58, 0x86, 0xff, 0x89, 0x15, 0x8a, 0xb8, 0x8b, 0x99, 0x39, 0x98, 0x3f, 0x98,
0x90, 0xa0, 0x91, 0xe0, 0x40, 0x20, 0x41, 0x28, 0x42, 0x26, 0x43, 0x25, 0x44, 0x1f,
0x45, 0x1a, 0x46, 0x16, 0x47, 0x12, 0x48, 0x0f, 0x49, 0x0d, 0x4b, 0x0b, 0x4c, 0x0a, 0x4e, 0x08,
0x4f, 0x06, 0x50, 0x06, 0x5a, 0x56, 0x51, 0x1b, 0x52, 0x04, 0x53, 0x4a, 0x54, 0x26,
0x57, 0x75, 0x58, 0x2b, 0x5a, 0xd6, 0x51, 0x28, 0x52, 0x1e, 0x53, 0x9e, 0x54, 0x70, 0x57, 0x50,
0x58, 0x07, 0x5c, 0x28, 0xb0, 0xe0, 0xb1, 0xc0, 0xb2, 0xb0, 0xb3, 0x4f, 0xb4, 0x63,
0xb4, 0xe3, 0xb1, 0xf0, 0xb2, 0xa0, 0x55, 0x00, 0x56, 0x40, 0x96, 0x50, 0x9a, 0x30, 0x6a, 0x81,
0x23, 0x33, 0xa0, 0xd0, 0xa1, 0x31, 0xa6, 0x04, 0xa2, 0x0f, 0xa3, 0x2b, 0xa4, 0x0f,
0xa5, 0x2b, 0xa7, 0x9a, 0xa8, 0x1c, 0xa9, 0x11, 0xaa, 0x16, 0xab, 0x16, 0xac, 0x3c, 0xad, 0xf0,
0xae, 0x57, 0xc6, 0xaa, 0xd2, 0x78, 0xd0, 0xb4, 0xd1, 0x00, 0xc8, 0x10, 0xc9, 0x12,
0xd3, 0x09, 0xd4, 0x2a, 0xee, 0x4c, 0x7e, 0xfa, 0x74, 0xa7, 0x78, 0x4e, 0x60, 0xe7, 0x61, 0xc8,
0x6d, 0x70, 0x1e, 0x39, 0x98, 0x1a, 0x9d, 0xf0
};

/* 初始化错误码 ---------------------------------------------------------------*/
#define xor_err 1                         /* XOR 校验错误 */
#define ack_err 2                         /* 芯片应答错误 */

/* 模块初始化完成标志 ---------------------------------------------------------*/
static uint8_t LC307_InitFlag = 0;

/**
 * @brief  LC307 激光模块初始化函数
 * @retval 0=初始化成功, 1=XOR 校验错误, 2=芯片应答错误
 * @note   初始化步骤：
 *         Step1: 发送握手命令 0xAA 0xAB 0x96 0x26 0xBC 0x50 0x5C
 *         Step2: 逐对写入配置表（183 对寄存器/值）
 *         Step3: 发送关闭配置命令 0xDD
 *         已在接收 DMA 回调中自动设置 LC307_InitFlag（解析到有效帧 = 已初始化）
 */
uint8_t Opf_LC307_Init(void)
{
	start_LC307_dma_recv();

	/* 等待模块上电稳定（100 × 15ms = 1.5s） */
	uint32_t wait_i = 0;
	for (wait_i = 0; wait_i < 100; wait_i++)
	{
		HAL_Delay(15);
	}

	/* 如果已经收到过有效数据帧，说明模块已完成初始化，直接返回 */
	if (LC307_InitFlag)
	{
		return 0;
	}
	else
	{
		/* 清空接收缓冲区（读 DR 寄存器触发清除） */
		USART3->DR;
	}

	stop_dma_recv();

	/* Step 1: 发送握手协议帧 */
	uint8_t step1_initbuf[7] = { 0xAA, 0xAB, 0x96, 0x26, 0xbc, 0x50, 0x5C };
	uint8_t feedbackbuf[3]   = { 0 };

	LC307_SendData(step1_initbuf, 7);

	/* 检查握手应答 */
	LC307_RecvData(feedbackbuf, 3);
	if ((feedbackbuf[0] ^ feedbackbuf[1]) != feedbackbuf[2])
	{
		g_lost_pos_dev = 1;
		return xor_err;                  /* XOR 校验失败 */
	}
	if (feedbackbuf[1] != 0x00)
	{
		g_lost_pos_dev = 1;
		return ack_err;                  /* 芯片返回错误码 */
	}

	/* Step 2: 逐对写入配置表
	 * 指令格式: 0xBB 0xDC [寄存器地址] [值] [XOR校验(1+2+3字节)] */
	for (uint16_t i = 0; i < sizeof(tab_BF3901_60hz) / sizeof(uint8_t); i += 2)
	{
		/* 组装配置帧 */
		uint8_t buf[5] = { 0xBB, 0xDC, tab_BF3901_60hz[i], tab_BF3901_60hz[i + 1], 0 };
		buf[4] = (buf[1] ^ buf[2] ^ buf[3]);
		LC307_SendData(buf, 5);

		/* 检查每对参数的应答 */
		LC307_RecvData(feedbackbuf, 3);
		if ((feedbackbuf[0] ^ feedbackbuf[1]) != feedbackbuf[2])
		{
			g_lost_pos_dev = 1;
			return xor_err;
		}
		if (feedbackbuf[1] != 0x00)
		{
			g_lost_pos_dev = 1;
			return ack_err;
		}
	}

	/* Step 3: 发送关闭配置命令，使配置生效 */
	uint8_t closecfg = 0xDD;
	LC307_SendData(&closecfg, 1);
	LC307_InitFlag = 1;

	/* 重新启动 DMA 接收（等待数据帧） */
	start_LC307_dma_recv();

	return 0;
}

/**
 * @brief  计算 XOR 校验和
 * @param  data:   数据缓冲区指针
 * @param  length: 数据长度
 * @retval 所有字节的异或结果
 */
static uint8_t XOR_Checksum(const uint8_t *data, uint16_t length)
{
	uint8_t xor_result = 0;
	for (uint16_t i = 0; i < length; i++)
	{
		xor_result ^= data[i];
	}
	return xor_result;
}

/* 激光模块数据帧结构体（1 字节对齐）-------------------------------------------*/
#pragma pack(1)
typedef struct {
	uint8_t  head1;                      /* 帧头 0xFE */
	uint8_t  bufcount;                   /* 数据计数 */
	short    flowX;                      /* X 方向光流位移 */
	short    flowY;                      /* Y 方向光流位移 */
	uint16_t timespan;                   /* 时间间隔 */
	uint16_t distance;                   /* 距离值 */
	uint8_t  quality;                    /* 数据质量 */
	uint8_t  version;                    /* 版本号 */
	uint8_t  XORSum;                     /* XOR 校验和 */
	uint8_t  end;                        /* 帧尾 */
} OpticalFlowFrame_t;
#pragma pack()

/* 接收数据帧缓存 */
static OpticalFlowFrame_t opfRecv_Frame = { 0 };

/* 光流速度输出（m/s） */
static float speed[2] = { 0 };

/**
 * @brief  光流结果回调函数（弱定义，用户可重写）
 * @param  buf: speed[0]=X 方向速度, speed[1]=Y 方向速度 (m/s)
 */
__weak void getOpticalFlowResult_Callback(float* buf)
{
	/* 用户在此处理光流速度数据 */
}

/* 频率调试（注释掉，保留备用） ------------------------------------------------*/
// #include "bsp_RTOSdebug.h"
// static pRtosDebugInterface_t debug = &RTOSTaskDebug;
// static RtosDebugPrivateVar debugpriv = { 0 };
// static uint8_t freq = 0;

/**
 * @brief  LC307 DMA 空闲中断回调函数（数据帧处理入口）
 * @param  size: 接收到的数据长度
 * @note   数据帧格式: 0xFE + 10 字节数据 + XOR 校验
 *          校验通过后：更新 InitFlag、提取光流值、调用用户回调
 *          处理完后自动重启 DMA 接收
 */
void LC307_Callback(uint16_t size)
{
	if (size == 14 && LC307OriBuffer[0] == 0xFE)   /* 帧头校验 */
	{
		/* XOR 校验（对数据区的 10 个字节计算） */
		if (LC307OriBuffer[12] == XOR_Checksum(&LC307OriBuffer[2], 10))
		{
			LC307_InitFlag = 1;                  /* 标记模块已正常工作 */
			memcpy(&opfRecv_Frame, LC307OriBuffer, sizeof(OpticalFlowFrame_t));

			/* 计算速度值（1.0f 为高度补偿系数，单位: m） */
			speed[0] = 1.0f * (float)opfRecv_Frame.flowX;
			speed[1] = 1.0f * (float)opfRecv_Frame.flowY; /* 单位 m/s */

			/* 调用用户回调函数 */
			getOpticalFlowResult_Callback(speed);

			/* 调试用频率测量（注释掉） */
			// freq = debug->UpdateFreq(&debugpriv);
		}
	}
	/* 重新启动 DMA 接收 */
	start_LC307_dma_recv();
}
