/*
 * FreeRTOS Kernel V10.3.1
 * 版权所有 (C) 2020 Amazon.com, Inc. 或其附属公司。保留所有权利。
 *
 * 特此免费授予任何获得本软件及相关文档文件（以下简称"软件"）副本的人士
 * 不受限制地使用本软件的权利，包括但不限于使用、复制、修改、合并、出版、
 * 分发、再许可和/或销售本软件副本的权利，并允许获得本软件的人士在满足
 * 以下条件的情况下这样做：
 *
 * 上述版权声明和本许可声明应包含在本软件的所有副本或实质性部分中。
 *
 * 本软件按"原样"提供，不提供任何明示或暗示的担保，包括但不限于
 * 适销性、特定用途适用性和非侵权的担保。在任何情况下，作者或版权
 * 持有人均不对任何索赔、损害或其他责任负责，无论是合同行为、侵权
 * 行为还是其他行为，因软件或软件的使用或其他交易而产生、由此产生
 * 或与之相关。
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 个制表符 == 4 个空格！
 */

/* 标准包含文件。 */
#include <stdlib.h>

/* 定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE 可防止 task.h 重新定义
所有 API 函数以使用 MPU 包装器。这应该只在应用程序文件中包含
task.h 时才这样做。 */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#if ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 0 )
	#error 要使 xTimerPendFunctionCall() 函数可用，configUSE_TIMERS 必须设置为 1。
#endif

/* 抑制 Lint e9021、e961 和 e750 作为 MISRA 例外，理由是
MPU 移植需要为上述头文件定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE，
但在此文件中不需要，以便生成正确的特权与非特权链接和放置。 */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e9021 !e961 !e750. */


/* 如果应用程序未配置为包含软件定时器功能，则将跳过整个源文件。
此 #if 在此文件的最后面关闭。如果要包含软件定时器功能，
请确保在 FreeRTOSConfig.h 中将 configUSE_TIMERS 设置为 1。 */
#if ( configUSE_TIMERS == 1 )

/* 杂项定义。 */
#define tmrNO_DELAY		( TickType_t ) 0U

/* 分配给定时器服务任务的名称。可以通过在 FreeRTOSConfig.h 中
定义 trmTIMER_SERVICE_TASK_NAME 来覆盖。 */
#ifndef configTIMER_SERVICE_TASK_NAME
	#define configTIMER_SERVICE_TASK_NAME "Tmr Svc"
#endif

/* 定时器结构体 ucStatus 成员中使用的位定义。 */
#define tmrSTATUS_IS_ACTIVE					( ( uint8_t ) 0x01 )
#define tmrSTATUS_IS_STATICALLY_ALLOCATED	( ( uint8_t ) 0x02 )
#define tmrSTATUS_IS_AUTORELOAD				( ( uint8_t ) 0x04 )

/* 定时器本身的定义。 */
typedef struct tmrTimerControl /* 使用旧的命名约定以防止破坏内核感知调试器。 */
{
	const char				*pcTimerName;		/*<< 文本名称。内核不使用此项，包含它只是为了便于调试。 */ /*lint !e971 未限定的 char 类型仅允许用于字符串和单个字符。 */
	ListItem_t				xTimerListItem;		/*<< 标准链表项，用于所有内核功能的事件管理。 */
	TickType_t				xTimerPeriodInTicks;/*<< 定时器到期的速度和频率。 */
	void 					*pvTimerID;			/*<< 用于标识定时器的 ID。当同一回调用于多个定时器时，这允许识别定时器。 */
	TimerCallbackFunction_t	pxCallbackFunction;	/*<< 定时器到期时将调用的函数。 */
	#if( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t			uxTimerNumber;		/*<< 由 FreeRTOS+Trace 等跟踪工具分配的 ID */
	#endif
	uint8_t 				ucStatus;			/*<< 保存指示定时器是否为静态分配以及是否处于活动状态的位。 */
} xTIMER;

/* 上面保留了旧的 xTIMER 名称，然后在下面 typedef 为新的 Timer_t 名称，
以便使用较旧的内核感知调试器。 */
typedef xTIMER Timer_t;

/* 可以在定时器队列上发送和接收的消息的定义。
可以排队两种类型的消息——操作软件定时器的消息，
以及请求执行非定时器相关回调的消息。这两种消息类型分别定义在
两个独立的结构体中：xTimerParametersType 和 xCallbackParametersType。 */
typedef struct tmrTimerParameters
{
	TickType_t			xMessageValue;		/*<< 由部分命令使用的可选值，例如在更改定时器周期时。 */
	Timer_t *			pxTimer;			/*<< 命令将应用于的定时器。 */
} TimerParameter_t;


typedef struct tmrCallbackParameters
{
	PendedFunction_t	pxCallbackFunction;	/* << 要执行的回调函数。 */
	void *pvParameter1;						/* << 将用作回调函数第一个参数的值。 */
	uint32_t ulParameter2;					/* << 将用作回调函数第二个参数的值。 */
} CallbackParameters_t;

/* 包含两种消息类型的结构体，以及用于确定哪种消息类型有效的标识符。 */
typedef struct tmrTimerQueueMessage
{
	BaseType_t			xMessageID;			/*<< 发送到定时器服务任务的命令。 */
	union
	{
		TimerParameter_t xTimerParameters;

		/* 如果不使用 xCallbackParameters，则不包含它，
		因为它会使结构体（以及定时器队列）变大。 */
		#if ( INCLUDE_xTimerPendFunctionCall == 1 )
			CallbackParameters_t xCallbackParameters;
		#endif /* INCLUDE_xTimerPendFunctionCall */
	} u;
} DaemonTaskMessage_t;

/*lint -save -e956 已使用手动分析和检查来确定哪些静态变量必须声明为 volatile。 */

/* 存储活动定时器的链表。定时器按到期时间顺序引用，
最近到期时间位于链表前面。只允许定时器服务任务访问这些链表。
xActiveTimerList1 和 xActiveTimerList2 可以放在函数作用域中，
但这会破坏一些内核感知调试器，以及依赖移除 static 限定符的调试器。 */
PRIVILEGED_DATA static List_t xActiveTimerList1;
PRIVILEGED_DATA static List_t xActiveTimerList2;
PRIVILEGED_DATA static List_t *pxCurrentTimerList;
PRIVILEGED_DATA static List_t *pxOverflowTimerList;

/* 用于向定时器服务任务发送命令的队列。 */
PRIVILEGED_DATA static QueueHandle_t xTimerQueue = NULL;
PRIVILEGED_DATA static TaskHandle_t xTimerTaskHandle = NULL;

/*lint -restore */

/*-----------------------------------------------------------*/

#if( configSUPPORT_STATIC_ALLOCATION == 1 )

	/* 如果支持静态分配，则应用程序必须提供以下回调函数——
	这使应用程序能够可选地提供将由定时器任务用作任务栈和 TCB 的内存。 */
	extern void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

#endif

/*
 * 如果尚未初始化定时器服务任务使用的基础设施，则初始化它。
 */
static void prvCheckForValidListAndQueue( void ) PRIVILEGED_FUNCTION;

/*
 * 定时器服务任务（守护进程）。定时器功能由该任务控制。
 * 其他任务使用 xTimerQueue 队列与定时器服务任务通信。
 */
static portTASK_FUNCTION_PROTO( prvTimerTask, pvParameters ) PRIVILEGED_FUNCTION;

/*
 * 由定时器服务任务调用，以解释和处理它在定时器队列上接收到的命令。
 */
static void prvProcessReceivedCommands( void ) PRIVILEGED_FUNCTION;

/*
 * 将定时器插入到 xActiveTimerList1 或 xActiveTimerList2 中，
 * 具体取决于到期时间是否导致定时器计数器溢出。
 */
static BaseType_t prvInsertTimerInActiveList( Timer_t * const pxTimer, const TickType_t xNextExpiryTime, const TickType_t xTimeNow, const TickType_t xCommandTime ) PRIVILEGED_FUNCTION;

/*
 * 活动定时器已达到其到期时间。如果是自动重载定时器，则重新加载定时器，
 * 然后调用其回调。
 */
static void prvProcessExpiredTimer( const TickType_t xNextExpireTime, const TickType_t xTimeNow ) PRIVILEGED_FUNCTION;

/*
 * Tick 计数已溢出。在确保当前定时器链表不再引用某些定时器后，
 * 切换定时器链表。
 */
static void prvSwitchTimerLists( void ) PRIVILEGED_FUNCTION;

/*
 * 获取当前 tick 计数，如果自上次调用 prvSampleTimeNow() 以来发生了
 * tick 计数溢出，则将 *pxTimerListsWereSwitched 设置为 pdTRUE。
 */
static TickType_t prvSampleTimeNow( BaseType_t * const pxTimerListsWereSwitched ) PRIVILEGED_FUNCTION;

/*
 * 如果定时器链表包含任何活动定时器，则返回最先到期的定时器的到期时间，
 * 并将 *pxListWasEmpty 设置为 false。如果定时器链表不包含任何定时器，
 * 则返回 0 并将 *pxListWasEmpty 设置为 pdTRUE。
 */
static TickType_t prvGetNextExpireTime( BaseType_t * const pxListWasEmpty ) PRIVILEGED_FUNCTION;

/*
 * 如果定时器已到期，处理它。否则，阻塞定时器服务任务，
 * 直到定时器到期或收到命令。
 */
static void prvProcessTimerOrBlockTask( const TickType_t xNextExpireTime, BaseType_t xListWasEmpty ) PRIVILEGED_FUNCTION;

/*
 * 在静态或动态分配 Timer_t 结构体后调用，以填充结构体的成员。
 */
static void prvInitialiseNewTimer(	const char * const pcTimerName,			/*lint !e971 未限定的 char 类型仅允许用于字符串和单个字符。 */
									const TickType_t xTimerPeriodInTicks,
									const UBaseType_t uxAutoReload,
									void * const pvTimerID,
									TimerCallbackFunction_t pxCallbackFunction,
									Timer_t *pxNewTimer ) PRIVILEGED_FUNCTION;
/*-----------------------------------------------------------*/

BaseType_t xTimerCreateTimerTask( void )
{
BaseType_t xReturn = pdFAIL;

	/* 如果 configUSE_TIMERS 设置为 1，则在调度器启动时调用此函数。
	检查定时器服务任务使用的基础设施是否已创建/初始化。如果已经创建了定时器，
	则初始化已经完成。 */
	prvCheckForValidListAndQueue();

	if( xTimerQueue != NULL )
	{
		#if( configSUPPORT_STATIC_ALLOCATION == 1 )
		{
			StaticTask_t *pxTimerTaskTCBBuffer = NULL;
			StackType_t *pxTimerTaskStackBuffer = NULL;
			uint32_t ulTimerTaskStackSize;

			vApplicationGetTimerTaskMemory( &pxTimerTaskTCBBuffer, &pxTimerTaskStackBuffer, &ulTimerTaskStackSize );
			xTimerTaskHandle = xTaskCreateStatic(	prvTimerTask,
													configTIMER_SERVICE_TASK_NAME,
													ulTimerTaskStackSize,
													NULL,
													( ( UBaseType_t ) configTIMER_TASK_PRIORITY ) | portPRIVILEGE_BIT,
													pxTimerTaskStackBuffer,
													pxTimerTaskTCBBuffer );

			if( xTimerTaskHandle != NULL )
			{
				xReturn = pdPASS;
			}
		}
		#else
		{
			xReturn = xTaskCreate(	prvTimerTask,
									configTIMER_SERVICE_TASK_NAME,
									configTIMER_TASK_STACK_DEPTH,
									NULL,
									( ( UBaseType_t ) configTIMER_TASK_PRIORITY ) | portPRIVILEGE_BIT,
									&xTimerTaskHandle );
		}
		#endif /* configSUPPORT_STATIC_ALLOCATION */
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	configASSERT( xReturn );
	return xReturn;
}
/*-----------------------------------------------------------*/

#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

	TimerHandle_t xTimerCreate(	const char * const pcTimerName,			/*lint !e971 未限定的 char 类型仅允许用于字符串和单个字符。 */
								const TickType_t xTimerPeriodInTicks,
								const UBaseType_t uxAutoReload,
								void * const pvTimerID,
								TimerCallbackFunction_t pxCallbackFunction )
	{
	Timer_t *pxNewTimer;

		pxNewTimer = ( Timer_t * ) pvPortMalloc( sizeof( Timer_t ) ); /*lint !e9087 !e9079 pvPortMalloc() 返回的所有值至少具有 MCU 栈所需的对齐，并且 Timer_t 的第一个成员始终是指向定时器名称的指针。 */

		if( pxNewTimer != NULL )
		{
			/* 目前状态为零，因为定时器不是静态创建的，且尚未启动。
			自动重载位可能在 prvInitialiseNewTimer 中设置。 */
			pxNewTimer->ucStatus = 0x00;
			prvInitialiseNewTimer( pcTimerName, xTimerPeriodInTicks, uxAutoReload, pvTimerID, pxCallbackFunction, pxNewTimer );
		}

		return pxNewTimer;
	}

#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

#if( configSUPPORT_STATIC_ALLOCATION == 1 )

	TimerHandle_t xTimerCreateStatic(	const char * const pcTimerName,		/*lint !e971 未限定的 char 类型仅允许用于字符串和单个字符。 */
										const TickType_t xTimerPeriodInTicks,
										const UBaseType_t uxAutoReload,
										void * const pvTimerID,
										TimerCallbackFunction_t pxCallbackFunction,
										StaticTimer_t *pxTimerBuffer )
	{
	Timer_t *pxNewTimer;

		#if( configASSERT_DEFINED == 1 )
		{
			/* 健全性检查：用于声明 StaticTimer_t 类型变量的结构体大小
			等于实际定时器结构体的大小。 */
			volatile size_t xSize = sizeof( StaticTimer_t );
			configASSERT( xSize == sizeof( Timer_t ) );
			( void ) xSize; /* 在未定义 configASSERT() 时使 lint 保持安静。 */
		}
		#endif /* configASSERT_DEFINED */

		/* 必须提供指向 StaticTimer_t 结构体的指针，使用它。 */
		configASSERT( pxTimerBuffer );
		pxNewTimer = ( Timer_t * ) pxTimerBuffer; /*lint !e740 !e9087 StaticTimer_t 是指向 Timer_t 的指针，因此保证对齐和大小正确（由 assert() 检查），所以这是安全的。 */

		if( pxNewTimer != NULL )
		{
			/* 定时器可以静态或动态创建，因此请注意此定时器是静态创建的，
			以防以后被删除。自动重载位可能在 prvInitialiseNewTimer() 中设置。 */
			pxNewTimer->ucStatus = tmrSTATUS_IS_STATICALLY_ALLOCATED;

			prvInitialiseNewTimer( pcTimerName, xTimerPeriodInTicks, uxAutoReload, pvTimerID, pxCallbackFunction, pxNewTimer );
		}

		return pxNewTimer;
	}

#endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

static void prvInitialiseNewTimer(	const char * const pcTimerName,			/*lint !e971 未限定的 char 类型仅允许用于字符串和单个字符。 */
									const TickType_t xTimerPeriodInTicks,
									const UBaseType_t uxAutoReload,
									void * const pvTimerID,
									TimerCallbackFunction_t pxCallbackFunction,
									Timer_t *pxNewTimer )
{
	/* 0 不是 xTimerPeriodInTicks 的有效值。 */
	configASSERT( ( xTimerPeriodInTicks > 0 ) );

	if( pxNewTimer != NULL )
	{
		/* 确保定时器服务任务使用的基础设施已创建/初始化。 */
		prvCheckForValidListAndQueue();

		/* 使用函数参数初始化定时器结构体成员。 */
		pxNewTimer->pcTimerName = pcTimerName;
		pxNewTimer->xTimerPeriodInTicks = xTimerPeriodInTicks;
		pxNewTimer->pvTimerID = pvTimerID;
		pxNewTimer->pxCallbackFunction = pxCallbackFunction;
		vListInitialiseItem( &( pxNewTimer->xTimerListItem ) );
		if( uxAutoReload != pdFALSE )
		{
			pxNewTimer->ucStatus |= tmrSTATUS_IS_AUTORELOAD;
		}
		traceTIMER_CREATE( pxNewTimer );
	}
}
/*-----------------------------------------------------------*/

BaseType_t xTimerGenericCommand( TimerHandle_t xTimer, const BaseType_t xCommandID, const TickType_t xOptionalValue, BaseType_t * const pxHigherPriorityTaskWoken, const TickType_t xTicksToWait )
{
BaseType_t xReturn = pdFAIL;
DaemonTaskMessage_t xMessage;

	configASSERT( xTimer );

	/* 向定时器服务任务发送消息，以对特定定时器定义执行特定操作。 */
	if( xTimerQueue != NULL )
	{
		/* 向定时器服务任务发送命令以启动 xTimer 定时器。 */
		xMessage.xMessageID = xCommandID;
		xMessage.u.xTimerParameters.xMessageValue = xOptionalValue;
		xMessage.u.xTimerParameters.pxTimer = xTimer;

		if( xCommandID < tmrFIRST_FROM_ISR_COMMAND )
		{
			if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING )
			{
				xReturn = xQueueSendToBack( xTimerQueue, &xMessage, xTicksToWait );
			}
			else
			{
				xReturn = xQueueSendToBack( xTimerQueue, &xMessage, tmrNO_DELAY );
			}
		}
		else
		{
			xReturn = xQueueSendToBackFromISR( xTimerQueue, &xMessage, pxHigherPriorityTaskWoken );
		}

		traceTIMER_COMMAND_SEND( xTimer, xCommandID, xOptionalValue, xReturn );
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

TaskHandle_t xTimerGetTimerDaemonTaskHandle( void )
{
	/* 如果在调度器启动之前调用 xTimerGetTimerDaemonTaskHandle()，
	则 xTimerTaskHandle 将为 NULL。 */
	configASSERT( ( xTimerTaskHandle != NULL ) );
	return xTimerTaskHandle;
}
/*-----------------------------------------------------------*/

TickType_t xTimerGetPeriod( TimerHandle_t xTimer )
{
Timer_t *pxTimer = xTimer;

	configASSERT( xTimer );
	return pxTimer->xTimerPeriodInTicks;
}
/*-----------------------------------------------------------*/

void vTimerSetReloadMode( TimerHandle_t xTimer, const UBaseType_t uxAutoReload )
{
Timer_t * pxTimer =  xTimer;

	configASSERT( xTimer );
	taskENTER_CRITICAL();
	{
		if( uxAutoReload != pdFALSE )
		{
			pxTimer->ucStatus |= tmrSTATUS_IS_AUTORELOAD;
		}
		else
		{
			pxTimer->ucStatus &= ~tmrSTATUS_IS_AUTORELOAD;
		}
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

UBaseType_t uxTimerGetReloadMode( TimerHandle_t xTimer )
{
Timer_t * pxTimer =  xTimer;
UBaseType_t uxReturn;

	configASSERT( xTimer );
	taskENTER_CRITICAL();
	{
		if( ( pxTimer->ucStatus & tmrSTATUS_IS_AUTORELOAD ) == 0 )
		{
			/* 不是自动重载定时器。 */
			uxReturn = ( UBaseType_t ) pdFALSE;
		}
		else
		{
			/* 是自动重载定时器。 */
			uxReturn = ( UBaseType_t ) pdTRUE;
		}
	}
	taskEXIT_CRITICAL();

	return uxReturn;
}
/*-----------------------------------------------------------*/

TickType_t xTimerGetExpiryTime( TimerHandle_t xTimer )
{
Timer_t * pxTimer =  xTimer;
TickType_t xReturn;

	configASSERT( xTimer );
	xReturn = listGET_LIST_ITEM_VALUE( &( pxTimer->xTimerListItem ) );
	return xReturn;
}
/*-----------------------------------------------------------*/

const char * pcTimerGetName( TimerHandle_t xTimer ) /*lint !e971 未限定的 char 类型仅允许用于字符串和单个字符。 */
{
Timer_t *pxTimer = xTimer;

	configASSERT( xTimer );
	return pxTimer->pcTimerName;
}
/*-----------------------------------------------------------*/

static void prvProcessExpiredTimer( const TickType_t xNextExpireTime, const TickType_t xTimeNow )
{
BaseType_t xResult;
Timer_t * const pxTimer = ( Timer_t * ) listGET_OWNER_OF_HEAD_ENTRY( pxCurrentTimerList ); /*lint !e9087 !e9079 使用 void * 是因为此宏也用于任务和协程。由于存储和检索的指针类型相同，对齐已知是没问题的。 */

	/* 从活动定时器链表中移除定时器。已经执行了检查以确保链表不为空。 */
	( void ) uxListRemove( &( pxTimer->xTimerListItem ) );
	traceTIMER_EXPIRED( pxTimer );

	/* 如果定时器是自动重载定时器，则计算下一个到期时间
	并将定时器重新插入到活动定时器链表中。 */
	if( ( pxTimer->ucStatus & tmrSTATUS_IS_AUTORELOAD ) != 0 )
	{
		/* 定时器使用相对于当前时间以外的时间插入到链表中。
		因此它将相对于此任务认为的当前时间插入到正确的链表中。 */
		if( prvInsertTimerInActiveList( pxTimer, ( xNextExpireTime + pxTimer->xTimerPeriodInTicks ), xTimeNow, xNextExpireTime ) != pdFALSE )
		{
			/* 定时器在添加到活动定时器链表之前已到期。
			现在重新加载它。 */
			xResult = xTimerGenericCommand( pxTimer, tmrCOMMAND_START_DONT_TRACE, xNextExpireTime, NULL, tmrNO_DELAY );
			configASSERT( xResult );
			( void ) xResult;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		pxTimer->ucStatus &= ~tmrSTATUS_IS_ACTIVE;
		mtCOVERAGE_TEST_MARKER();
	}

	/* 调用定时器回调。 */
	pxTimer->pxCallbackFunction( ( TimerHandle_t ) pxTimer );
}
/*-----------------------------------------------------------*/

static portTASK_FUNCTION( prvTimerTask, pvParameters )
{
TickType_t xNextExpireTime;
BaseType_t xListWasEmpty;

	/* 仅仅是为了避免编译器警告。 */
	( void ) pvParameters;

	#if( configUSE_DAEMON_TASK_STARTUP_HOOK == 1 )
	{
		extern void vApplicationDaemonTaskStartupHook( void );

		/* 允许应用程序编写者在此任务开始执行时在此任务的上下文中执行一些代码。
		如果应用程序包含在调度器启动后执行有益的初始化代码，这将很有用。 */
		vApplicationDaemonTaskStartupHook();
	}
	#endif /* configUSE_DAEMON_TASK_STARTUP_HOOK */

	for( ;; )
	{
		/* 查询定时器链表以查看它是否包含任何定时器，如果是，
		则获取下一个定时器到期的时间。 */
		xNextExpireTime = prvGetNextExpireTime( &xListWasEmpty );

		/* 如果定时器已到期，处理它。否则，阻塞此任务，
		直到定时器到期或收到命令。 */
		prvProcessTimerOrBlockTask( xNextExpireTime, xListWasEmpty );

		/* 清空命令队列。 */
		prvProcessReceivedCommands();
	}
}
/*-----------------------------------------------------------*/

static void prvProcessTimerOrBlockTask( const TickType_t xNextExpireTime, BaseType_t xListWasEmpty )
{
TickType_t xTimeNow;
BaseType_t xTimerListsWereSwitched;

	vTaskSuspendAll();
	{
		/* 获取当前时间以评估定时器是否已到期。
		如果获取时间导致链表切换，则不处理此定时器，
		因为在 prvSampleTimeNow() 函数中，链表切换时保留在链表中的
		任何定时器都将已被处理。 */
		xTimeNow = prvSampleTimeNow( &xTimerListsWereSwitched );
		if( xTimerListsWereSwitched == pdFALSE )
		{
			/* Tick 计数没有溢出，定时器是否已到期？ */
			if( ( xListWasEmpty == pdFALSE ) && ( xNextExpireTime <= xTimeNow ) )
			{
				( void ) xTaskResumeAll();
				prvProcessExpiredTimer( xNextExpireTime, xTimeNow );
			}
			else
			{
				/* Tick 计数没有溢出，且下一个到期时间尚未到达。
				因此此任务应阻塞以等待下一个到期时间或收到命令——
				以先到者为准。除非在当前定时器链表为空的情况下，
				否则只有 xNextExpireTime > xTimeNow 时才能到达以下行。 */
				if( xListWasEmpty != pdFALSE )
				{
					/* 当前定时器链表为空——溢出链表也为空吗？ */
					xListWasEmpty = listLIST_IS_EMPTY( pxOverflowTimerList );
				}

				vQueueWaitForMessageRestricted( xTimerQueue, ( xNextExpireTime - xTimeNow ), xListWasEmpty );

				if( xTaskResumeAll() == pdFALSE )
				{
					/* 让出 CPU 以等待命令到达或阻塞时间到期。
					如果在退出临界区和此让出之间命令到达，
					则此让出不会导致任务阻塞。 */
					portYIELD_WITHIN_API();
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
		}
		else
		{
			( void ) xTaskResumeAll();
		}
	}
}
/*-----------------------------------------------------------*/

static TickType_t prvGetNextExpireTime( BaseType_t * const pxListWasEmpty )
{
TickType_t xNextExpireTime;

	/* 定时器按到期时间顺序列出，链表头引用最先到期的任务。
	获取具有最近到期时间的定时器将到期的时间。如果没有活动定时器，
	则只需将下一个到期时间设置为 0。这将导致此任务在 tick 计数溢出时
	解除阻塞，此时定时器链表将被切换，下一个到期时间可以重新评估。 */
	*pxListWasEmpty = listLIST_IS_EMPTY( pxCurrentTimerList );
	if( *pxListWasEmpty == pdFALSE )
	{
		xNextExpireTime = listGET_ITEM_VALUE_OF_HEAD_ENTRY( pxCurrentTimerList );
	}
	else
	{
		/* 确保任务在 tick 计数翻转时解除阻塞。 */
		xNextExpireTime = ( TickType_t ) 0U;
	}

	return xNextExpireTime;
}
/*-----------------------------------------------------------*/

static TickType_t prvSampleTimeNow( BaseType_t * const pxTimerListsWereSwitched )
{
TickType_t xTimeNow;
PRIVILEGED_DATA static TickType_t xLastTime = ( TickType_t ) 0U; /*lint !e956 变量仅可由一个任务访问。 */

	xTimeNow = xTaskGetTickCount();

	if( xTimeNow < xLastTime )
	{
		prvSwitchTimerLists();
		*pxTimerListsWereSwitched = pdTRUE;
	}
	else
	{
		*pxTimerListsWereSwitched = pdFALSE;
	}

	xLastTime = xTimeNow;

	return xTimeNow;
}
/*-----------------------------------------------------------*/

static BaseType_t prvInsertTimerInActiveList( Timer_t * const pxTimer, const TickType_t xNextExpiryTime, const TickType_t xTimeNow, const TickType_t xCommandTime )
{
BaseType_t xProcessTimerNow = pdFALSE;

	listSET_LIST_ITEM_VALUE( &( pxTimer->xTimerListItem ), xNextExpiryTime );
	listSET_LIST_ITEM_OWNER( &( pxTimer->xTimerListItem ), pxTimer );

	if( xNextExpiryTime <= xTimeNow )
	{
		/* 在发出启动/重置定时器命令和命令被处理之间，
		到期时间是否已经过了？ */
		if( ( ( TickType_t ) ( xTimeNow - xCommandTime ) ) >= pxTimer->xTimerPeriodInTicks ) /*lint !e961 MISRA 例外，因为强制转换仅对某些移植是冗余的。 */
		{
			/* 发出命令和命令被处理之间的时间实际上超过了定时器周期。 */
			xProcessTimerNow = pdTRUE;
		}
		else
		{
			vListInsert( pxOverflowTimerList, &( pxTimer->xTimerListItem ) );
		}
	}
	else
	{
		if( ( xTimeNow < xCommandTime ) && ( xNextExpiryTime >= xCommandTime ) )
		{
			/* 如果自命令发出以来，tick 计数已溢出但到期时间没有溢出，
			则定时器一定已经过了其到期时间，应立即处理。 */
			xProcessTimerNow = pdTRUE;
		}
		else
		{
			vListInsert( pxCurrentTimerList, &( pxTimer->xTimerListItem ) );
		}
	}

	return xProcessTimerNow;
}
/*-----------------------------------------------------------*/

static void	prvProcessReceivedCommands( void )
{
DaemonTaskMessage_t xMessage;
Timer_t *pxTimer;
BaseType_t xTimerListsWereSwitched, xResult;
TickType_t xTimeNow;

	while( xQueueReceive( xTimerQueue, &xMessage, tmrNO_DELAY ) != pdFAIL ) /*lint !e603 xMessage 不需要初始化，因为它是传出而非传入，并且只有在 xQueueReceive() 返回 pdTRUE 时才使用。 */
	{
		#if ( INCLUDE_xTimerPendFunctionCall == 1 )
		{
			/* 负命令是挂起函数调用而不是定时器命令。 */
			if( xMessage.xMessageID < ( BaseType_t ) 0 )
			{
				const CallbackParameters_t * const pxCallback = &( xMessage.u.xCallbackParameters );

				/* 定时器使用 xCallbackParameters 成员来请求执行回调。
				检查回调不为 NULL。 */
				configASSERT( pxCallback );

				/* 调用函数。 */
				pxCallback->pxCallbackFunction( pxCallback->pvParameter1, pxCallback->ulParameter2 );
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		#endif /* INCLUDE_xTimerPendFunctionCall */

		/* 正命令是定时器命令而不是挂起函数调用。 */
		if( xMessage.xMessageID >= ( BaseType_t ) 0 )
		{
			/* 消息使用 xTimerParameters 成员来操作软件定时器。 */
			pxTimer = xMessage.u.xTimerParameters.pxTimer;

			if( listIS_CONTAINED_WITHIN( NULL, &( pxTimer->xTimerListItem ) ) == pdFALSE ) /*lint !e961. 只有当 NULL 传递给宏时，强制转换才是冗余的。 */
			{
				/* 定时器在链表中，移除它。 */
				( void ) uxListRemove( &( pxTimer->xTimerListItem ) );
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			traceTIMER_COMMAND_RECEIVED( pxTimer, xMessage.xMessageID, xMessage.u.xTimerParameters.xMessageValue );

			/* 在这种情况下不使用 xTimerListsWereSwitched 参数，
			但它必须出现在函数调用中。prvSampleTimeNow() 必须在从 xTimerQueue
			接收消息后调用，这样就没有可能让更高优先级的任务以比定时器守护进程
			任务更晚的时间向消息队列添加消息（因为它在设置 xTimeNow 值之后
			抢占了定时器守护进程任务）。 */
			xTimeNow = prvSampleTimeNow( &xTimerListsWereSwitched );

			switch( xMessage.xMessageID )
			{
				case tmrCOMMAND_START :
				case tmrCOMMAND_START_FROM_ISR :
				case tmrCOMMAND_RESET :
				case tmrCOMMAND_RESET_FROM_ISR :
				case tmrCOMMAND_START_DONT_TRACE :
					/* 启动或重启定时器。 */
					pxTimer->ucStatus |= tmrSTATUS_IS_ACTIVE;
					if( prvInsertTimerInActiveList( pxTimer,  xMessage.u.xTimerParameters.xMessageValue + pxTimer->xTimerPeriodInTicks, xTimeNow, xMessage.u.xTimerParameters.xMessageValue ) != pdFALSE )
					{
						/* 定时器在添加到活动定时器链表之前已到期。
						现在处理它。 */
						pxTimer->pxCallbackFunction( ( TimerHandle_t ) pxTimer );
						traceTIMER_EXPIRED( pxTimer );

						if( ( pxTimer->ucStatus & tmrSTATUS_IS_AUTORELOAD ) != 0 )
						{
							xResult = xTimerGenericCommand( pxTimer, tmrCOMMAND_START_DONT_TRACE, xMessage.u.xTimerParameters.xMessageValue + pxTimer->xTimerPeriodInTicks, NULL, tmrNO_DELAY );
							configASSERT( xResult );
							( void ) xResult;
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
					break;

				case tmrCOMMAND_STOP :
				case tmrCOMMAND_STOP_FROM_ISR :
					/* 定时器已从活动链表中移除。 */
					pxTimer->ucStatus &= ~tmrSTATUS_IS_ACTIVE;
					break;

				case tmrCOMMAND_CHANGE_PERIOD :
				case tmrCOMMAND_CHANGE_PERIOD_FROM_ISR :
					pxTimer->ucStatus |= tmrSTATUS_IS_ACTIVE;
					pxTimer->xTimerPeriodInTicks = xMessage.u.xTimerParameters.xMessageValue;
					configASSERT( ( pxTimer->xTimerPeriodInTicks > 0 ) );

					/* 新周期实际上没有参考点，可以比旧周期长或短。
					因此命令时间设置为当前时间，并且由于周期不能为零，
					下一个到期时间只能在未来，这意味着（与上面的 xTimerStart() 情况不同）
					这里不需要处理失败情况。 */
					( void ) prvInsertTimerInActiveList( pxTimer, ( xTimeNow + pxTimer->xTimerPeriodInTicks ), xTimeNow, xTimeNow );
					break;

				case tmrCOMMAND_DELETE :
					#if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
					{
						/* 定时器已从活动链表中移除，
						如果内存是动态分配的，则只需释放内存。 */
						if( ( pxTimer->ucStatus & tmrSTATUS_IS_STATICALLY_ALLOCATED ) == ( uint8_t ) 0 )
						{
							vPortFree( pxTimer );
						}
						else
						{
							pxTimer->ucStatus &= ~tmrSTATUS_IS_ACTIVE;
						}
					}
					#else
					{
						/* 如果未启用动态分配，则内存不可能是动态分配的。
						因此不需要释放内存——只需将定时器标记为"非活动"。 */
						pxTimer->ucStatus &= ~tmrSTATUS_IS_ACTIVE;
					}
					#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
					break;

				default	:
					/* 不应该到达这里。 */
					break;
			}
		}
	}
}
/*-----------------------------------------------------------*/

static void prvSwitchTimerLists( void )
{
TickType_t xNextExpireTime, xReloadTime;
List_t *pxTemp;
Timer_t *pxTimer;
BaseType_t xResult;

	/* Tick 计数已溢出。必须切换定时器链表。
	如果当前定时器链表中仍有任何被引用的定时器，
	则它们一定已经到期，应在链表切换之前处理。 */
	while( listLIST_IS_EMPTY( pxCurrentTimerList ) == pdFALSE )
	{
		xNextExpireTime = listGET_ITEM_VALUE_OF_HEAD_ENTRY( pxCurrentTimerList );

		/* 从链表中移除定时器。 */
		pxTimer = ( Timer_t * ) listGET_OWNER_OF_HEAD_ENTRY( pxCurrentTimerList ); /*lint !e9087 !e9079 使用 void * 是因为此宏也用于任务和协程。由于存储和检索的指针类型相同，对齐已知是没问题的。 */
		( void ) uxListRemove( &( pxTimer->xTimerListItem ) );
		traceTIMER_EXPIRED( pxTimer );

		/* 执行其回调，如果是自动重载定时器，则发送命令重新启动定时器。
		它不能在这里重新启动，因为链表尚未切换。 */
		pxTimer->pxCallbackFunction( ( TimerHandle_t ) pxTimer );

		if( ( pxTimer->ucStatus & tmrSTATUS_IS_AUTORELOAD ) != 0 )
		{
			/* 计算重载值，如果重载值导致定时器进入相同的定时器链表，
			则它已经到期，定时器应重新插入到当前链表中，以便在此循环中
			再次处理。否则应发送命令重新启动定时器，以确保它仅在链表
			交换后才插入到链表中。 */
			xReloadTime = ( xNextExpireTime + pxTimer->xTimerPeriodInTicks );
			if( xReloadTime > xNextExpireTime )
			{
				listSET_LIST_ITEM_VALUE( &( pxTimer->xTimerListItem ), xReloadTime );
				listSET_LIST_ITEM_OWNER( &( pxTimer->xTimerListItem ), pxTimer );
				vListInsert( pxCurrentTimerList, &( pxTimer->xTimerListItem ) );
			}
			else
			{
				xResult = xTimerGenericCommand( pxTimer, tmrCOMMAND_START_DONT_TRACE, xNextExpireTime, NULL, tmrNO_DELAY );
				configASSERT( xResult );
				( void ) xResult;
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}

	pxTemp = pxCurrentTimerList;
	pxCurrentTimerList = pxOverflowTimerList;
	pxOverflowTimerList = pxTemp;
}
/*-----------------------------------------------------------*/

static void prvCheckForValidListAndQueue( void )
{
	/* 检查引用活动定时器的链表和用于与定时器服务通信的队列
	是否已初始化。 */
	taskENTER_CRITICAL();
	{
		if( xTimerQueue == NULL )
		{
			vListInitialise( &xActiveTimerList1 );
			vListInitialise( &xActiveTimerList2 );
			pxCurrentTimerList = &xActiveTimerList1;
			pxOverflowTimerList = &xActiveTimerList2;

			#if( configSUPPORT_STATIC_ALLOCATION == 1 )
			{
				/* 定时器队列静态分配，以防 configSUPPORT_DYNAMIC_ALLOCATION 为 0。 */
				static StaticQueue_t xStaticTimerQueue; /*lint !e956 可以以这种方式声明，以防止在其他位置添加额外的条件编译保护。 */
				static uint8_t ucStaticTimerQueueStorage[ ( size_t ) configTIMER_QUEUE_LENGTH * sizeof( DaemonTaskMessage_t ) ]; /*lint !e956 可以以这种方式声明，以防止在其他位置添加额外的条件编译保护。 */

				xTimerQueue = xQueueCreateStatic( ( UBaseType_t ) configTIMER_QUEUE_LENGTH, ( UBaseType_t ) sizeof( DaemonTaskMessage_t ), &( ucStaticTimerQueueStorage[ 0 ] ), &xStaticTimerQueue );
			}
			#else
			{
				xTimerQueue = xQueueCreate( ( UBaseType_t ) configTIMER_QUEUE_LENGTH, sizeof( DaemonTaskMessage_t ) );
			}
			#endif

			#if ( configQUEUE_REGISTRY_SIZE > 0 )
			{
				if( xTimerQueue != NULL )
				{
					vQueueAddToRegistry( xTimerQueue, "TmrQ" );
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			#endif /* configQUEUE_REGISTRY_SIZE */
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

BaseType_t xTimerIsTimerActive( TimerHandle_t xTimer )
{
BaseType_t xReturn;
Timer_t *pxTimer = xTimer;

	configASSERT( xTimer );

	/* 定时器在活动定时器链表中吗？ */
	taskENTER_CRITICAL();
	{
		if( ( pxTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) == 0 )
		{
			xReturn = pdFALSE;
		}
		else
		{
			xReturn = pdTRUE;
		}
	}
	taskEXIT_CRITICAL();

	return xReturn;
} /*lint !e818 由于 typedef 的原因，不能是指向 const 的指针。 */
/*-----------------------------------------------------------*/

void *pvTimerGetTimerID( const TimerHandle_t xTimer )
{
Timer_t * const pxTimer = xTimer;
void *pvReturn;

	configASSERT( xTimer );

	taskENTER_CRITICAL();
	{
		pvReturn = pxTimer->pvTimerID;
	}
	taskEXIT_CRITICAL();

	return pvReturn;
}
/*-----------------------------------------------------------*/

void vTimerSetTimerID( TimerHandle_t xTimer, void *pvNewID )
{
Timer_t * const pxTimer = xTimer;

	configASSERT( xTimer );

	taskENTER_CRITICAL();
	{
		pxTimer->pvTimerID = pvNewID;
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

#if( INCLUDE_xTimerPendFunctionCall == 1 )

	BaseType_t xTimerPendFunctionCallFromISR( PendedFunction_t xFunctionToPend, void *pvParameter1, uint32_t ulParameter2, BaseType_t *pxHigherPriorityTaskWoken )
	{
	DaemonTaskMessage_t xMessage;
	BaseType_t xReturn;

		/* 用函数参数完成消息并将其发送到守护进程任务。 */
		xMessage.xMessageID = tmrCOMMAND_EXECUTE_CALLBACK_FROM_ISR;
		xMessage.u.xCallbackParameters.pxCallbackFunction = xFunctionToPend;
		xMessage.u.xCallbackParameters.pvParameter1 = pvParameter1;
		xMessage.u.xCallbackParameters.ulParameter2 = ulParameter2;

		xReturn = xQueueSendFromISR( xTimerQueue, &xMessage, pxHigherPriorityTaskWoken );

		tracePEND_FUNC_CALL_FROM_ISR( xFunctionToPend, pvParameter1, ulParameter2, xReturn );

		return xReturn;
	}

#endif /* INCLUDE_xTimerPendFunctionCall */
/*-----------------------------------------------------------*/

#if( INCLUDE_xTimerPendFunctionCall == 1 )

	BaseType_t xTimerPendFunctionCall( PendedFunction_t xFunctionToPend, void *pvParameter1, uint32_t ulParameter2, TickType_t xTicksToWait )
	{
	DaemonTaskMessage_t xMessage;
	BaseType_t xReturn;

		/* 此函数只能在创建定时器之后或调度器启动之后调用，
		因为在那之前定时器队列不存在。 */
		configASSERT( xTimerQueue );

		/* 用函数参数完成消息并将其发送到守护进程任务。 */
		xMessage.xMessageID = tmrCOMMAND_EXECUTE_CALLBACK;
		xMessage.u.xCallbackParameters.pxCallbackFunction = xFunctionToPend;
		xMessage.u.xCallbackParameters.pvParameter1 = pvParameter1;
		xMessage.u.xCallbackParameters.ulParameter2 = ulParameter2;

		xReturn = xQueueSendToBack( xTimerQueue, &xMessage, xTicksToWait );

		tracePEND_FUNC_CALL( xFunctionToPend, pvParameter1, ulParameter2, xReturn );

		return xReturn;
	}

#endif /* INCLUDE_xTimerPendFunctionCall */
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	UBaseType_t uxTimerGetTimerNumber( TimerHandle_t xTimer )
	{
		return ( ( Timer_t * ) xTimer )->uxTimerNumber;
	}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	void vTimerSetTimerNumber( TimerHandle_t xTimer, UBaseType_t uxTimerNumber )
	{
		( ( Timer_t * ) xTimer )->uxTimerNumber = uxTimerNumber;
	}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/

/* 如果应用程序未配置为包含软件定时器功能，则将跳过整个源文件。
如果要包含软件定时器功能，请确保在 FreeRTOSConfig.h 中将 configUSE_TIMERS 设置为 1。 */
#endif /* configUSE_TIMERS == 1 */
