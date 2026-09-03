#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"

#include "usart.h"

#include "bsp_buzzer.h"
#include "bsp_stp23L.h"
#include "bsp_stl06n.h"
#include "bsp_LC307.h"
//#include "bsp_N10.h"

#include "balance_task.h"
static void _System_Reset_(uint8_t uart_recv)
{
	extern void ResetSystem(uint8_t isFromISR);
	
	static uint8_t res_buf[5];
	static uint8_t res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//接受到上位机请求的复位字符“reset”，执行软件复位
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			ResetSystem(1);//进行软件复位，复位后执行 BootLoader 程序
		}
	}
}

//历史数据缓存
//static void HistoryCache(uint8_t *array, uint8_t newData,uint16_t BufLen)
//{
//	//新数据在数组头部
////    for (uint8_t i = BufLen; i > 0; i--) {
////        array[i] = array[i - 1];
////    }
////    array[0] = newData;
//	
//	//新数据在数组末尾
//    for (uint16_t i = 0; i < BufLen - 1; i++) {
//        array[i] = array[i + 1];
//    }

//    array[BufLen - 1] = newData;
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if( huart == &huart4 ) //蓝牙使用的串口
	{
		//复位BootLoadr检测指令
		_System_Reset_(uart4_recv);
		
		/* APP写入指令 */
		extern QueueHandle_t g_xQueueBlueTooth_Ori;
		xQueueSendFromISR(g_xQueueBlueTooth_Ori,&uart4_recv,&xHigherPriorityTaskWoken);
		
		HAL_UART_Receive_IT(&huart4,&uart4_recv,1);
	}
	else if( huart == &huart1 )
	{
		_System_Reset_(uart1_recv);
		HAL_UART_Receive_IT(&huart1,&uart1_recv,1);
	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


//DMA传输完成与半完成中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//DMA传输完成中断涉及写入队列操作
	extern QueueHandle_t g_xQueuestp23L_Ori;
	extern QueueHandle_t g_xQueuestl06n_Ori;
	
//	extern QueueHandle_t g_xQueueN10_Ori; 
	
	//DMA搬运数据的存放区
	extern OriData_STP23L_t DMABuf_oridata_stp23L;
	extern OriData_STL06N_t DMAbuf_ori_stl06n;
	
	//检查高优先级任务唤醒
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//检查队列写入情况
	BaseType_t xWriteState = errQUEUE_FULL;
	
	if( huart == &huart5 ) 
	{	
		/* DMA传输半完成,完成,均进入此中断,只需要传输完成的处理 */
		if( HAL_UART_STATE_READY == huart->RxState )
		{
			/* 本次DMA数据接收量 */
			DMABuf_oridata_stp23L.DataLen = Size;
			
			/* 写入队列,检查写入状态 */
			xWriteState = xQueueSendFromISR(g_xQueuestp23L_Ori,&DMABuf_oridata_stp23L,&xHigherPriorityTaskWoken);
			if( errQUEUE_FULL == xWriteState )
			{
				pBuzzeInterface_t tips = &UserBuzzer;
				tips->toggle();
			}
			
			/* 重新使能DMA开启下一次数据的传输 */
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5,DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN);
		}

	}
	
	else if( huart == &huart2 )
	{
		/* DMA传输半完成,完成,均进入此中断,只需要传输完成的处理 */
		if( HAL_UART_STATE_READY == huart->RxState )
		{
			/* 本次DMA数据接收量 */
			DMAbuf_ori_stl06n.DataLen = Size;

			/* 写入队列,检查写入状态 */
			xWriteState = xQueueSendFromISR(g_xQueuestl06n_Ori,&DMAbuf_ori_stl06n,&xHigherPriorityTaskWoken);
			if( errQUEUE_FULL == xWriteState )
			{
				pBuzzeInterface_t tips = &UserBuzzer;
				tips->toggle();
			}
			
			/* 重新使能DMA开启下一次数据的传输 */
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2,DMAbuf_ori_stl06n.Buf,userconfig_STL06N_DMABUF_LEN);
		}
	}
	//光流模块数据
	else if(  huart == &huart3 )
	{
		if( HAL_UART_STATE_READY == huart->RxState )
		{
			LC307_Callback(Size);
		}
		
	}
	else if( huart == &huart4 )
	{
		if( HAL_UART_STATE_READY == huart->RxState )
		{

		}
	}
	
	/* 根据具体情况发起调度 */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
