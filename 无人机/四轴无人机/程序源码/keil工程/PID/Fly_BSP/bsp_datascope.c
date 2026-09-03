/**
 * ============================================================
 * bsp_datascope.c — DataScope 串口虚拟示波器驱动
 * ============================================================
 * 功能: 通过串口发送数据到 PC端 DataScope V1.0 软件
 * 协议: 自定义帧格式 (帧头+通道数+浮点数据+帧尾)
 * 用法: datascope->show(dataArray, channelCount)
 * 实例: UserDataScope
 * ============================================================
 */

#include "bsp_datascope.h"
#include "usart.h"


static unsigned char DataScope_OutPut_Buffer[42] = {0};	   //串口发送缓冲区

//函数说明：将单精度浮点数据转成4字节数据并存入指定地址
//附加说明：用户无需直接操作此函数
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回
static void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败
static unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
    if ( (Channel_Number > 10) || (Channel_Number == 0) ) {
        return 0;    //通道个数大于10或等于0，直接跳出，不执行函数
    }
    else
    {
        DataScope_OutPut_Buffer[0] = '$';  //帧头

        switch(Channel_Number)
        {
			case 1:
				DataScope_OutPut_Buffer[5]  =  5;
				return  6;
			case 2:
				DataScope_OutPut_Buffer[9]  =  9;
				return 10;
			case 3:
				DataScope_OutPut_Buffer[13] = 13;
				return 14;
			case 4:
				DataScope_OutPut_Buffer[17] = 17;
				return 18;
			case 5:
				DataScope_OutPut_Buffer[21] = 21;
				return 22;
			case 6:
				DataScope_OutPut_Buffer[25] = 25;
				return 26;
			case 7:
				DataScope_OutPut_Buffer[29] = 29;
				return 30;
			case 8:
				DataScope_OutPut_Buffer[33] = 33;
				return 34;
			case 9:
				DataScope_OutPut_Buffer[37] = 37;
				return 38;
			case 10:
				DataScope_OutPut_Buffer[41] = 41;
				return 42;
        }
    }
    return 0;
}



//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
//函数无返回
static void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
    if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
        switch (Channel)
        {
			case 1:
				Float2Byte(&Data,DataScope_OutPut_Buffer,1);
				break;
			case 2:
				Float2Byte(&Data,DataScope_OutPut_Buffer,5);
				break;
			case 3:
				Float2Byte(&Data,DataScope_OutPut_Buffer,9);
				break;
			case 4:
				Float2Byte(&Data,DataScope_OutPut_Buffer,13);
				break;
			case 5:
				Float2Byte(&Data,DataScope_OutPut_Buffer,17);
				break;
			case 6:
				Float2Byte(&Data,DataScope_OutPut_Buffer,21);
				break;
			case 7:
				Float2Byte(&Data,DataScope_OutPut_Buffer,25);
				break;
			case 8:
				Float2Byte(&Data,DataScope_OutPut_Buffer,29);
				break;
			case 9:
				Float2Byte(&Data,DataScope_OutPut_Buffer,33);
				break;
			case 10:
				Float2Byte(&Data,DataScope_OutPut_Buffer,37);
				break;
        }
    }
}

//波形显示使用的串口
UART_HandleTypeDef* DataScope_Seiral = &huart4;

static void DataScope_ShowData(float* showdata,uint8_t showlen)
{
	if( showlen>10 ) return;//超出最大值限制,直接返回
	
	for(uint8_t i=0;i<showlen;i++)
	{
		DataScope_Get_Channel_Data(showdata[i],i+1);//通道从1开始
	}
	uint8_t sendcount = DataScope_Data_Generate(showlen);
	
	/* 注意：使用DMA务必在CubeMX配置后再使用 */
	HAL_UART_Transmit_DMA(DataScope_Seiral,DataScope_OutPut_Buffer,sendcount);
	//HAL_UART_Transmit(DataScope_Seiral,DataScope_OutPut_Buffer,sendcount,200);
}

DataScopeInterface_t UserDataScope = {
	.show = DataScope_ShowData,
};

