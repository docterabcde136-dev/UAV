/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//C Lib
#include <stdio.h>

#include "queue.h"        
#include "semphr.h"      
#include "event_groups.h" 

//HAL
#include "usart.h"

//BSP
#include "bsp_stp23L.h"
//#include "bsp_N10.h"
#include "bsp_stl06n.h"

//APP
#include "balance_task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define userconfig_OPEN_STACK_CHECK 0     //开启任务栈大小检查打印
#define userconfig_OPEN_CPU_USAGE_CHECK 0 //开启检查CPU占比打印
#define userconfig_OPEN_CHECK_HEAPSIZE 0  //检查剩余的堆大小
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
xSemaphoreHandle HandleMutex_printf;

//任务句柄
TaskHandle_t sensorhandleTask_Handle; 
TaskHandle_t balanceTask_Handle;
TaskHandle_t g_xTaskHandleCoordTask;

//队列、队列集
QueueHandle_t g_xQueuestp23L_Ori;     //外围设备：stp23L激光测距 原始数据的队列
QueueHandle_t g_xQueueN10_Ori;        //外围设备：N10雷达        原始数据的队列
QueueHandle_t g_xQueuestl06n_Ori;

QueueSetHandle_t g_xQueueSetSensor;   //创建队列集，存放各个传感器队列的句柄
QueueHandle_t g_xQueueFlyControl;     //四轴飞行器控制队列
QueueHandle_t g_xQueueBlueTooth_Ori;  //蓝牙APP原始数据
QueueHandle_t g_xQueueLidarBuffer;    //雷达数据

//事件组
EventGroupHandle_t g_xEventFlyAction; //四轴动作事件组

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//RTOS系统检测任务
#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
void CpuUsageCheckTask(void *param);
#endif

//显示任务
__weak void ShowTask(void* param)
{
	for(;;) vTaskDelay(1);
}

//外围传感器数据处理任务
__weak void SensorHandleTask(void* param)
{
	for(;;) vTaskDelay(1);
}

//平衡主控制任务
__weak void balance_task(void* param)
{
	for(;;) vTaskDelay(1);
}

//蓝牙APP控制任务
__weak void BluetoothAPPControl_task(void* param)
{
	for(;;) vTaskDelay(1);
}

__weak void CoordinateHandleTask(void* param)
{
	for(;;) vTaskDelay(1);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	//使用cpu占比计时器前初始化内容
	TIM6->CNT = 0;
}

__weak unsigned long getRunTimeCounterValue(void)
{
	static unsigned long time = 0 ;
	static uint16_t lasttime = 0;
	static uint16_t nowtime = 0;
	
	nowtime = TIM6->CNT; //获得当前计数值
	
	//如果本次计数值小于上次计数值，说明发生了定时器计数溢出
	if( nowtime < lasttime )
	{
		time += (nowtime + 0xffff - lasttime); //溢出后的时间增量
	}		
	else time += ( nowtime - lasttime ) ; //未发生溢出，增量为本次时间-上次时间
	
	lasttime = nowtime;
	
	return time;
}


/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	printf("%s stack overflow\r\n",pcTaskName);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	printf("malloc failed.check heapsize\r\n");
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	HandleMutex_printf = xSemaphoreCreateMutex();

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	/* 队列创建 */
	g_xQueuestp23L_Ori = xQueueCreate(5,sizeof(OriData_STP23L_t));
	g_xQueuestl06n_Ori    = xQueueCreate(10,sizeof(OriData_STL06N_t));
	g_xQueueFlyControl = xQueueCreate(1,sizeof(FlyControlType_t));
	g_xQueueBlueTooth_Ori = xQueueCreate(50,sizeof(char));
	g_xQueueLidarBuffer = xQueueCreate(5,sizeof(Stl06NAngleBuffer_t));
	
	/* 队列集创建,加入队列集 */
	g_xQueueSetSensor = xQueueCreateSet( 5 + 10 );
	xQueueAddToSet(g_xQueuestp23L_Ori,g_xQueueSetSensor);
	xQueueAddToSet(g_xQueuestl06n_Ori,g_xQueueSetSensor);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
   xTaskCreate(ShowTask,"ShowTask",128*2,NULL,osPriorityNormal,NULL);
   xTaskCreate(SensorHandleTask,"SensorTask",128*3,NULL,osPriorityNormal,&sensorhandleTask_Handle);
   xTaskCreate(balance_task,"BalanceTask",128*4,NULL,osPriorityNormal,&balanceTask_Handle);
   xTaskCreate(BluetoothAPPControl_task,"BTAPPTask",128*2,NULL,osPriorityNormal,NULL);
   xTaskCreate(CoordinateHandleTask,"coordTask",128*16,NULL,osPriorityNormal,&g_xTaskHandleCoordTask);
   
	//调试任务
	#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
	 static uint16_t delaytime = 5000;//打印时间间隔，单位tick
	 xTaskCreate(CpuUsageCheckTask,"DebugTask",128, &delaytime ,osPriorityAboveNormal,NULL);
	#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
	g_xEventFlyAction = xEventGroupCreate();//创建四轴动作事件组的实例
	
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  /* 等待所有任务开始运行后，再使能各个传感器的中断，以便每个数据都能成功与任务联系上 */
	  vTaskDelay(400);
	  
	  /* 启动外围传感器的DMA搬运 */
	  extern OriData_STP23L_t DMABuf_oridata_stp23L;
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart5,DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN);
	  
	  //启动雷达数据搬运
	  extern OriData_STL06N_t DMAbuf_ori_stl06n;
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,DMAbuf_ori_stl06n.Buf,userconfig_STL06N_DMABUF_LEN);
	  
	  vTaskDelete(NULL);//自杀
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
void CpuUsageCheckTask(void *param)
{
	uint16_t* delaytime = (uint16_t*)param;
	static char showbuf[500];
	
	while( 1 )
	{
#if 1 == userconfig_OPEN_CPU_USAGE_CHECK
		//打印CPU占比
		vTaskGetRunTimeStats(showbuf);
		xSemaphoreTake(HandleMutex_printf,portMAX_DELAY);
		printf("TaskName\tUseTime\tCPU\r\n");
		printf("%s\r\n",showbuf);
		xSemaphoreGive(HandleMutex_printf);
		vTaskDelay(*delaytime);
#endif

#if 1 == userconfig_OPEN_STACK_CHECK
		//打印剩余任务栈大小,单位word
		vTaskList(showbuf);
		xSemaphoreTake(HandleMutex_printf,portMAX_DELAY);
		printf("TaskName\tTaskState\tTaskPrio\tStackSize\tTaskNum\r\n");
		printf("%s\r\n",showbuf);
		xSemaphoreGive(HandleMutex_printf);
		vTaskDelay(*delaytime);
#endif
		
#if 1 == userconfig_OPEN_CHECK_HEAPSIZE
		//打印剩余的堆区大小,单位bytes
		xSemaphoreTake(HandleMutex_printf,portMAX_DELAY);
		printf("free heap size : %d bytes\r\n\r\n",xPortGetFreeHeapSize());
		xSemaphoreGive(HandleMutex_printf);
		vTaskDelay(*delaytime);
#endif
	}
}
#endif

/* USER CODE END Application */

