/**
 * ============================================================
 * bsp_stl06n.c — STL06N 360°激光雷达驱动
 * ============================================================
 * 功能: 360°旋转激光雷达数据解析 (UART+DMA)
 * 协议: 固定帧格式, 每帧500点, 分辨率0.72°/点
 * 处理:
 *   - 状态机帧解析 (帧头→长度→数据→CRC校验)
 *   - 角度到索引映射
 *   - 相邻帧点云合并 + 中值滤波去噪
 *   - 时间戳帧丢失检测
 * 接口: Stl06N_BufferHandle 回调处理
 * ============================================================
 */

#include "bsp_stl06n.h"

#include "FreeRTOS.h"
#include "queue.h"


extern QueueHandle_t g_xQueueLidarBuffer;

OriData_STL06N_t DMAbuf_ori_stl06n = { 0 };

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


#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "bsp_RTOSdebug.h"

#define DEBUG_LEVEL 1

#define THRESHOLD_DISTANCE 100 //单位mm

// 处理2个相邻的点云
static uint16_t process_twoPointCloud(int a, int b) {
    if (abs(a - b) < THRESHOLD_DISTANCE) {
        return roundf((a + b) / 2.0f);
    } else {
        return (a < b) ? a : b;
    }
}

// 处理3个相邻的点云
static uint16_t process_threePointCloud(int a, int b, int c) {
    int diff_ab = abs(a - b);
    int diff_ac = abs(a - c);
    int diff_bc = abs(b - c);
    
    if( diff_bc<=diff_ac&&diff_bc<=diff_ab ){
        return process_twoPointCloud(b, c);
    }
    else if( diff_ab<=diff_ac&&diff_ab<=diff_bc ){
        return process_twoPointCloud(a, b);
    }
    else if( diff_ac<=diff_ab&&diff_ac<=diff_bc ){
        return process_twoPointCloud(a, c);
    }
	else
		return a;
}

// 点云数据分辨率降低优化
static uint16_t process_PointCloudarray(uint16_t *arr, uint8_t size) {
    if (size == 2) {
        return process_twoPointCloud(arr[0], arr[1]);
    } else if (size == 3) {
        return process_threePointCloud(arr[0], arr[1], arr[2]);
    } else {
        return arr[0]; // 返回一个错误值，表示输入不符合要求
    }
}

RtosDebugPrivateVar privdebug = { 0 };
pRtosDebugInterface_t debug = &RTOSTaskDebug;

#include "usart.h"
//void Stl06NLidar_Start(void)
//{
//	uint8_t StartLidar[8] = { 0x54,0xA0,0x04,0x00,0x00,0x00,0x00,0x5E };
//	HAL_UART_Transmit(&huart2,StartLidar,8,200);
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,DMAbuf_ori_stl06n.Buf,userconfig_STL06N_DMABUF_LEN);
//}

//void Stl06NLidar_Stop(void)
//{
//	HAL_UART_DMAStop(&huart2);
//	uint8_t StopLidar[8] = { 0x54,0xA1,0x04,0x00,0x00,0x00,0x00,0x4A };
//	HAL_UART_Transmit(&huart2,StopLidar,8,200);
//}

//void Stl06NLidar_Toggle(void)
//{
//	static uint8_t flag = 0;
//	flag = !flag;
//	if( flag ) Stl06NLidar_Start();
//	else Stl06NLidar_Stop();
//}

//1.8分辨率
void Stl06N_BufferHandle(uint8_t* recv,uint8_t bufferLen)
{
	//1帧完整雷达数据
	static Stl06NFrameTypeDef stl06n_oneframe = { 0 };
	
	//存放1圈雷达数据的数组
	static Stl06NAngleBuffer_t stl06n_onecircle = { 0 } ;
	
	const uint8_t stl06frameLen = sizeof(Stl06NFrameTypeDef);//1帧数据长度
	static uint8_t recvbuffer[sizeof(Stl06NFrameTypeDef)] = { 0 };
	static uint8_t recvcounts = 0;
	
	//状态机
	enum{
		WaitHead = 0,
		GetData = 1,
		CrcCheck = 2,
	};
	static uint8_t sm = WaitHead;
	
	//数据就绪标志位
	uint8_t ReadyFlag = 0;

	//数据处理过程
	for(uint8_t i=0;i<bufferLen;i++)
	{
		switch( sm )
		{
			case WaitHead:
				//索引到帧头,准备接收数据
				if( i + 1 < bufferLen && recv[i]==0x54 && recv[i+1]==0x2C)
				{
					recvbuffer[0] = 0x54;
					recvcounts = 1;
					sm = GetData;	
				}
				break;
			case GetData:
				recvbuffer[recvcounts++] = recv[i];
				if( recvcounts == stl06frameLen-1 )
				{
					sm = CrcCheck;
				}
				break;
			case CrcCheck:
				//对接收到的数据进行校验,正确则赋值
				if( CalCRC8(recvbuffer,recvcounts)==recv[i] )
				{
					recvbuffer[recvcounts] = recv[i];
					memcpy(&stl06n_oneframe,recvbuffer,sizeof(Stl06NFrameTypeDef));
					ReadyFlag = 1;
				}
				else printf("crc8 error!\r\n");
				
				recvcounts=0; //数据接收完成,开始下一轮接收
				sm = WaitHead;
				break;
		}
	}
	
	//成功解析到1帧数据
	if( ReadyFlag==1 )
	{
		uint16_t angleIncr = 0;
		
		//1帧数据 12个点,角度范围7.9度
		if( stl06n_oneframe.start_angle>stl06n_oneframe.end_angle )
		{
			angleIncr = stl06n_oneframe.end_angle + 36000 - stl06n_oneframe.start_angle;
		}
		else angleIncr = stl06n_oneframe.end_angle-stl06n_oneframe.start_angle;
		
		//分辨率 = 角度范围/点云数量-1 = 790 / 11 = 72 (0.72°)
		angleIncr /= 11;
		
		//数组下标索引
		uint16_t index = 0; 
		static uint16_t lastindex = 0;
		uint16_t lostindex = 0; 
		
		uint16_t distancelist[5] = { 0 };
		uint8_t filtersize = 0;
		
		#if DEBUG_LEVEL > 1
		static uint16_t debug1 = 0;
		static uint16_t debug2 = 0;
		static uint16_t freqcount = 0;
		#endif
		
		for(uint8_t j=0;j<12;j++)
		{
			//当前角度
			uint16_t angle_temp = stl06n_oneframe.start_angle + j*angleIncr;
			if( angle_temp>36000 ) angle_temp-=36000;
			
			//获取索引
			index = round(angle_temp/Stl06n_AngleRatio);//分辨率0.72
			index = index%Stl06n_LidarDistanceBufferLen;
			
			if( index<lastindex )
			{
				#if DEBUG_LEVEL > 1
				printf("1: %d\r\n",debug1);
				printf("2: %d\r\n",debug2);
				printf("all:%d\r\n",debug1+debug2);
				debug1=0,debug2=0;
				
				//频率计算
				uint8_t freq = 0;
				if( stl06n_oneframe.timestamp<freqcount )
					freq = stl06n_oneframe.timestamp+30000-freqcount;
				else freq = stl06n_oneframe.timestamp-freqcount;
				printf("time:%d\r\n\r\n",freq);
				freqcount = stl06n_oneframe.timestamp;
				#endif
				
				xQueueSend(g_xQueueLidarBuffer,&stl06n_onecircle,0);
				
				//数据处理完毕,清空已解析的数据
				memset(&stl06n_onecircle,0,sizeof(Stl06NAngleBuffer_t));
				
				lostindex = (index + Stl06n_LidarDistanceBufferLen) - lastindex;
			}
			else lostindex = index  - lastindex;
			
			//正常索引
			if( lostindex != 0 )
			{
				#if DEBUG_LEVEL > 1
				debug1++;
				#endif
				
				if( filtersize!=0 )
				{
					//所有数据
					distancelist[filtersize] = stl06n_onecircle.buffer[Stl06n_LidarDistanceBufferLen-lastindex].distance;
					filtersize++;
					
					stl06n_onecircle.buffer[Stl06n_LidarDistanceBufferLen-lastindex].distance = process_PointCloudarray(distancelist,filtersize);
					filtersize=0;
				}

				//加载新数据
				stl06n_onecircle.buffer[Stl06n_LidarDistanceBufferLen-index].distance = stl06n_oneframe.point[j].distance;
				stl06n_onecircle.buffer[Stl06n_LidarDistanceBufferLen-index].peak = stl06n_oneframe.point[j].peak;
			}
			
			//重复索引,先将重复的索引内容保存到数组内
			else if( lostindex == 0 )
			{
				#if DEBUG_LEVEL > 1
				debug2++;
				#endif
				
				distancelist[filtersize] = stl06n_oneframe.point[j].distance;
				filtersize++;
			}
			
			//保存索引
			lastindex = index;
		}

		//丢帧检查
		static uint16_t lasttimestamp = 0;
		uint16_t difftime =0 ;
		if( stl06n_oneframe.timestamp<lasttimestamp )
		{
			difftime = stl06n_oneframe.timestamp + 30000 - lasttimestamp;
		}
		else difftime = stl06n_oneframe.timestamp - lasttimestamp;
		
		if( difftime > 3 )
		{
			printf("losttime:%d\r\n",difftime);
		}
		lasttimestamp = stl06n_oneframe.timestamp;
	}
}


//完整分辨率0.72
//void Stl06N_BufferHandle(uint8_t* recv,uint8_t bufferLen)
//{
//	//1帧完整雷达数据
//	static Stl06NFrameTypeDef stl06n_oneframe = { 0 };
//	
//	//存放1圈雷达数据的数组
//	static Stl06NAngleBuffer_t stl06n_onecircle = { 0 } ;
//	
//	const uint8_t stl06frameLen = sizeof(Stl06NFrameTypeDef);//1帧数据长度
//	static uint8_t recvbuffer[stl06frameLen] = { 0 };
//	static uint8_t recvcounts = 0;
//	
//	//状态机
//	enum{
//		WaitHead = 0,
//		GetData = 1,
//		CrcCheck = 2,
//	};
//	static uint8_t sm = WaitHead;
//	
//	//数据就绪标志位
//	uint8_t ReadyFlag = 0;

//	//数据处理过程
//	for(uint8_t i=0;i<bufferLen;i++)
//	{
//		switch( sm )
//		{
//			case WaitHead:
//				//索引到帧头,准备接收数据
//				if( i + 1 < bufferLen && recv[i]==0x54 && recv[i+1]==0x2C)
//				{
//					recvbuffer[0] = 0x54;
//					recvcounts = 1;
//					sm = GetData;	
//				}
//				break;
//			case GetData:
//				recvbuffer[recvcounts++] = recv[i];
//				if( recvcounts == stl06frameLen-1 )
//				{
//					sm = CrcCheck;
//				}
//				break;
//			case CrcCheck:
//				//对接收到的数据进行校验,正确则赋值
//				if( CalCRC8(recvbuffer,recvcounts)==recv[i] )
//				{
//					recvbuffer[recvcounts] = recv[i];
//					memcpy(&stl06n_oneframe,recvbuffer,sizeof(Stl06NFrameTypeDef));
//					ReadyFlag = 1;
//				}
//				else printf("crc8 error!\r\n");
//				
//				recvcounts=0; //数据接收完成,开始下一轮接收
//				sm = WaitHead;
//				break;
//		}
//	}
//	
//	//成功解析到1帧数据
//	if( ReadyFlag==1 )
//	{
//		uint16_t angleIncr = 0;
//		
//		//1帧数据 12个点,角度范围7.9度
//		if( stl06n_oneframe.start_angle>stl06n_oneframe.end_angle )
//		{
//			angleIncr = stl06n_oneframe.end_angle + 36000 - stl06n_oneframe.start_angle;
//		}
//		else angleIncr = stl06n_oneframe.end_angle-stl06n_oneframe.start_angle;
//		
//		//分辨率 = 角度范围/点云数量-1 = 790 / 11 = 72 (0.72°)
//		angleIncr /= 11;
//		
//		//数组下标索引
//		uint16_t index = 0; 
//		static uint16_t lastindex = 0;
//		uint16_t lostindex = 0; 
//		
//		#if DEBUG_LEVEL > 1
//		static uint16_t debug1 = 0;
//		static uint16_t debug2 = 0;
//		static uint16_t debug3 = 0;
//		static uint16_t debug4 = 0;
//		static uint16_t freqcount = 0;
//		#endif
//		
//		for(uint8_t j=0;j<12;j++)
//		{
//			//当前角度
//			uint16_t angle_temp = stl06n_oneframe.start_angle + j*angleIncr;
//			if( angle_temp>36000 ) angle_temp-=36000;
//			
//			//获取索引
//			index = round(angle_temp/Stl06n_AngleRatio);//分辨率0.72
//			index = index%Stl06n_LidarDistanceBufferLen;
//			
//			if( index<lastindex )
//			{
//				#if DEBUG_LEVEL > 1
//				printf("1: %d\r\n",debug1);
//				printf("2: %d\r\n",debug2);
//				printf("3: %d\r\n",debug3);
//				printf("4: %d\r\n",debug4);
//				printf("all:%d\r\n",debug1+debug2+debug3+debug4);
//				debug1=0,debug2=0,debug3=0,debug4=0;
//				
//				//频率计算
//				uint8_t freq = 0;
//				if( stl06n_oneframe.timestamp<freqcount )
//					freq = stl06n_oneframe.timestamp+30000-freqcount;
//				else freq = stl06n_oneframe.timestamp-freqcount;
//				printf("time:%d\r\n\r\n",freq);
//				freqcount = stl06n_oneframe.timestamp;
//				#endif
//				
//				//数据处理完毕,清空已解析的数据
//				memset(&stl06n_onecircle,0,sizeof(Stl06NAngleBuffer_t));
//				
//				lostindex = (index + Stl06n_LidarDistanceBufferLen) - lastindex;
//			}
//			else lostindex = index  - lastindex;
//			
//			//正常索引
//			if( lostindex == 1 )
//			{
//				#if DEBUG_LEVEL > 1
//				debug1++;
//				#endif
//				
//				stl06n_onecircle.buffer[index].distance = stl06n_oneframe.point[j].distance;
//				stl06n_onecircle.buffer[index].peak = stl06n_oneframe.point[j].peak;
//			}
//			//重复索引
//			else if( lostindex == 0 )
//			{
//				#if DEBUG_LEVEL > 1
//				debug2++;
//				#endif
//				stl06n_onecircle.buffer[index].distance += stl06n_oneframe.point[j].distance;
//				stl06n_onecircle.buffer[index].peak+= stl06n_oneframe.point[j].peak;
//				stl06n_onecircle.buffer[index].distance/=2;
//				stl06n_onecircle.buffer[index].peak/=2;
//			}
//			//跳过索引
//			else if( lostindex == 2 )
//			{
//				#if DEBUG_LEVEL > 1
//				debug3++;
//				#endif
//				
//				if( index!=0 ) //索引不为0时才补充,若索引为0,则上一帧的数据已经被处理了,补充无意义
//				{			
//					//补充遗漏的索引值
//					stl06n_onecircle.buffer[index-1].distance = (stl06n_onecircle.buffer[lastindex].distance + stl06n_oneframe.point[j].distance)/2;
//					stl06n_onecircle.buffer[index-1].peak = (stl06n_onecircle.buffer[lastindex].peak + stl06n_oneframe.point[j].peak)/2;
//				}
//				//本次索引赋值
//				stl06n_onecircle.buffer[index].distance = stl06n_oneframe.point[j].distance;
//				stl06n_onecircle.buffer[index].peak = stl06n_oneframe.point[j].peak;	
//			}
//			else
//			{
//				#if DEBUG_LEVEL > 1
//				debug4++;
//				#endif
//			}
//			
//			//保存索引
//			lastindex = index;
//		}

//		//丢帧检查
//		static uint16_t lasttimestamp = 0;
//		uint16_t difftime =0 ;
//		if( stl06n_oneframe.timestamp<lasttimestamp )
//		{
//			difftime = stl06n_oneframe.timestamp + 30000 - lasttimestamp;
//		}
//		else difftime = stl06n_oneframe.timestamp - lasttimestamp;
//		
//		if( difftime > 3 )
//		{
//			printf("losttime:%d\r\n",difftime);
//		}
//		lasttimestamp = stl06n_oneframe.timestamp;
//	}
//}




