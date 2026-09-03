/**
 * ============================================================
 * bsp_stp23L.c — STP23L 激光测距模块驱动
 * ============================================================
 * 功能: 单点激光测距 (UART协议), 用于四轴高度测量
 * 协议: 帧头0x59 0x53, 1字节距离数据, 1字节校验
 * 数据处理: DMA不定长接收 → 队列缓冲 → 校验滤波 → 全局高度变量
 * 接口: STP23LInterface_t { stp23L_getdistance_callback, ... }
 * ============================================================
 */

#include "bsp_stp23L.h"

#include <stdio.h>
#include <string.h>

OriData_STP23L_t DMABuf_oridata_stp23L;

//STP23L高度数据
float g_readonly_distance = 0;

//1个点云数据包里包含的信息。一帧数据共有12个点云数据
#pragma pack(1)
typedef struct { 
    int16_t distance;   //距离
    uint16_t noise;     //环境噪声
    uint32_t peak;      //强度
    uint8_t confidence; //置信度
    uint32_t intg;      //积分次数
    int16_t reftof;     //温度表征值
}LidarPointTypedef; 
#pragma pack()

//1帧完整点云数据
#pragma pack(1)
typedef struct { 
    LidarPointTypedef PointCloud[12];//1帧数据12个点云
    uint32_t timestamp;//1个时间戳
}STP23L_OneFrame_t; 
#pragma pack()


static STP23L_OneFrame_t stp23L_frame;               //定义1帧stp23L数据，用于数据接收
static const uint8_t stp23L_BufLen = sizeof(stp23L_frame); //原始数据的长度
static uint8_t stp23L_buf[ sizeof( stp23L_frame ) ] ;         //存储原始数据的数组
static uint8_t stp23L_counts = 0;                    //接收数据的计数值

//获取距离信息
void stp23L_getdistance_callback(uint8_t recv,float* dis)
{
	static uint8_t checkcode = 0;//校验码
	
    //简单状态机
    enum{
        Wait_HEAD=0       ,//等待正式数据阶段
        Handle_Ddata      ,//处理数据阶段
        END_DATA          ,//数据校验处理阶段
    };
	
	static uint8_t state_machine = Wait_HEAD;//初始化状态机,等待帧头
	
	switch( state_machine )
	{
		case Wait_HEAD:
		{
			static uint8_t headlen = 0;
			//前10字节数据均为已知的帧头：0xAA 0xAA 0xAA 0xAA 0x00 0x02 0x00 0x00 0xB8 0x00
			headlen++;
			if( 10 == headlen  ) 
			{
				headlen=0 , state_machine = Handle_Ddata , checkcode = 0xBA; //checkcode是去除4个0xAA后开始累计,这里的初始值是 0x02+0xB8，后续开始累加
			}
			break;
		}
		case Handle_Ddata:
		{
			/* 数据校验 */
			checkcode+=recv;
			
			/* 接收数据 */
			stp23L_buf[stp23L_counts++] = recv;
			
			/* 数据接收完毕,进入处理阶段 */
			if( stp23L_BufLen == stp23L_counts ) stp23L_counts=0,state_machine = END_DATA;
			
			break;
		}

		case END_DATA:
		{
			if( recv == checkcode )
			{
				/* 数据赋值解码 */
				memcpy(&stp23L_frame,stp23L_buf,stp23L_BufLen);
				
				float  alldist = 0;
				for(uint8_t i=0;i<12;i++) alldist+=stp23L_frame.PointCloud[i].distance;
				*dis = alldist/12.0f; //距离信息赋值	
				*dis /= 1000.0f;      //将单位 mm 转为 m
			}
			else
			{
				//数据校验错误
			}
			
			checkcode = 0;
			state_machine = Wait_HEAD;
			break;
		}
		
	}
}

