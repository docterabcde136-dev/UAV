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

/* FreeRTOS 包含文件。 */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"

/* 抑制 Lint e961、e750 和 e9021 作为 MISRA 例外，理由是
MPU 移植需要为上述头文件定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE，
但在此文件中不需要，以便生成正确的特权与非特权链接和放置。 */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e961 !e750 !e9021 请参阅上面的注释。 */

/* 以下位字段在任务的事件列表项值中传递控制信息。
重要的是它们不能与 taskEVENT_LIST_ITEM_VALUE_IN_USE 定义冲突。 */
#if configUSE_16_BIT_TICKS == 1
	#define eventCLEAR_EVENTS_ON_EXIT_BIT	0x0100U
	#define eventUNBLOCKED_DUE_TO_BIT_SET	0x0200U
	#define eventWAIT_FOR_ALL_BITS			0x0400U
	#define eventEVENT_BITS_CONTROL_BYTES	0xff00U
#else
	#define eventCLEAR_EVENTS_ON_EXIT_BIT	0x01000000UL
	#define eventUNBLOCKED_DUE_TO_BIT_SET	0x02000000UL
	#define eventWAIT_FOR_ALL_BITS			0x04000000UL
	#define eventEVENT_BITS_CONTROL_BYTES	0xff000000UL
#endif

typedef struct EventGroupDef_t
{
	EventBits_t uxEventBits;
	List_t xTasksWaitingForBits;		/*< 等待某个位被设置的任务链表。 */

	#if( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t uxEventGroupNumber;
	#endif

	#if( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
		uint8_t ucStaticallyAllocated; /*< 如果事件组是静态分配的，则设置为 pdTRUE，以确保不会尝试释放内存。 */
	#endif
} EventGroup_t;

/*-----------------------------------------------------------*/

/*
 * 测试 uxCurrentEventBits 中设置的位，以查看等待条件是否满足。
 * 等待条件由 xWaitForAllBits 定义。如果 xWaitForAllBits 为 pdTRUE，
 * 则只有当 uxBitsToWaitFor 中设置的所有位也在 uxCurrentEventBits 中设置时，
 * 等待条件才满足。如果 xWaitForAllBits 为 pdFALSE，则只要 uxBitsToWaitFor 中
 * 设置的任何位也在 uxCurrentEventBits 中设置，等待条件就满足。
 */
static BaseType_t prvTestWaitCondition( const EventBits_t uxCurrentEventBits, const EventBits_t uxBitsToWaitFor, const BaseType_t xWaitForAllBits ) PRIVILEGED_FUNCTION;

/*-----------------------------------------------------------*/

#if( configSUPPORT_STATIC_ALLOCATION == 1 )

	EventGroupHandle_t xEventGroupCreateStatic( StaticEventGroup_t *pxEventGroupBuffer )
	{
	EventGroup_t *pxEventBits;

		/* 必须提供一个 StaticEventGroup_t 对象。 */
		configASSERT( pxEventGroupBuffer );

		#if( configASSERT_DEFINED == 1 )
		{
			/* 健全性检查：用于声明 StaticEventGroup_t 类型变量的结构体大小
			等于实际事件组结构体的大小。 */
			volatile size_t xSize = sizeof( StaticEventGroup_t );
			configASSERT( xSize == sizeof( EventGroup_t ) );
		} /*lint !e529 如果定义了 configASSERT()，则 xSize 被引用。 */
		#endif /* configASSERT_DEFINED */

		/* 用户已提供一个静态分配的事件组——使用它。 */
		pxEventBits = ( EventGroup_t * ) pxEventGroupBuffer; /*lint !e740 !e9087 EventGroup_t 和 StaticEventGroup_t 为了数据隐藏目的而被故意别名化，并保证具有相同的大小和对齐要求——由 configASSERT() 检查。 */

		if( pxEventBits != NULL )
		{
			pxEventBits->uxEventBits = 0;
			vListInitialise( &( pxEventBits->xTasksWaitingForBits ) );

			#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
			{
				/* 静态和动态分配都可以使用，因此请注意此事件组是静态创建的，
				以防以后删除事件组。 */
				pxEventBits->ucStaticallyAllocated = pdTRUE;
			}
			#endif /* configSUPPORT_DYNAMIC_ALLOCATION */

			traceEVENT_GROUP_CREATE( pxEventBits );
		}
		else
		{
			/* xEventGroupCreateStatic 只应在 pxEventGroupBuffer 指向
			预分配（编译时分配）的 StaticEventGroup_t 变量时调用。 */
			traceEVENT_GROUP_CREATE_FAILED();
		}

		return pxEventBits;
	}

#endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

	EventGroupHandle_t xEventGroupCreate( void )
	{
	EventGroup_t *pxEventBits;

		/* 分配事件组。MISRA 偏差的理由如下：
		pvPortMalloc() 始终确保返回的内存块按照 MCU 栈的要求对齐。
		在这种情况下，pvPortMalloc() 必须返回一个保证满足 EventGroup_t 结构体
		对齐要求的指针——该要求（如果你追溯下去）是 TickType_t 类型的对齐要求
		（EventBits_t 本身是 TickType_t 类型）。因此，只要栈对齐要求大于或等于
		TickType_t 对齐要求，此强制转换就是安全的。在其他情况下，
		当架构的自然字大小小于 sizeof(TickType_t) 时，TickType_t 变量将通过两次
		或更多次读取操作来访问，对齐要求仅为每次单独读取的要求。 */
		pxEventBits = ( EventGroup_t * ) pvPortMalloc( sizeof( EventGroup_t ) ); /*lint !e9087 !e9079 请参阅上面的注释。 */

		if( pxEventBits != NULL )
		{
			pxEventBits->uxEventBits = 0;
			vListInitialise( &( pxEventBits->xTasksWaitingForBits ) );

			#if( configSUPPORT_STATIC_ALLOCATION == 1 )
			{
				/* 静态和动态分配都可以使用，因此请注意此事件组是动态分配的，
				以防以后删除事件组。 */
				pxEventBits->ucStaticallyAllocated = pdFALSE;
			}
			#endif /* configSUPPORT_STATIC_ALLOCATION */

			traceEVENT_GROUP_CREATE( pxEventBits );
		}
		else
		{
			traceEVENT_GROUP_CREATE_FAILED(); /*lint !e9063 else 分支仅用于允许跟踪，如果未定义跟踪宏，则不生成代码。 */
		}

		return pxEventBits;
	}

#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

EventBits_t xEventGroupSync( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet, const EventBits_t uxBitsToWaitFor, TickType_t xTicksToWait )
{
EventBits_t uxOriginalBitValue, uxReturn;
EventGroup_t *pxEventBits = xEventGroup;
BaseType_t xAlreadyYielded;
BaseType_t xTimeoutOccurred = pdFALSE;

	configASSERT( ( uxBitsToWaitFor & eventEVENT_BITS_CONTROL_BYTES ) == 0 );
	configASSERT( uxBitsToWaitFor != 0 );
	#if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
	{
		configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) && ( xTicksToWait != 0 ) ) );
	}
	#endif

	vTaskSuspendAll();
	{
		uxOriginalBitValue = pxEventBits->uxEventBits;

		( void ) xEventGroupSetBits( xEventGroup, uxBitsToSet );

		if( ( ( uxOriginalBitValue | uxBitsToSet ) & uxBitsToWaitFor ) == uxBitsToWaitFor )
		{
			/* 所有集合点的位现在都已设置——无需阻塞。 */
			uxReturn = ( uxOriginalBitValue | uxBitsToSet );

			/* 集合点始终清除这些位。除非这是集合点中唯一的任务，
			否则它们已经被清除了。 */
			pxEventBits->uxEventBits &= ~uxBitsToWaitFor;

			xTicksToWait = 0;
		}
		else
		{
			if( xTicksToWait != ( TickType_t ) 0 )
			{
				traceEVENT_GROUP_SYNC_BLOCK( xEventGroup, uxBitsToSet, uxBitsToWaitFor );

				/* 将调用任务正在等待的位存储在任务的事件列表项中，
				以便内核在找到匹配时知道。然后进入阻塞状态。 */
				vTaskPlaceOnUnorderedEventList( &( pxEventBits->xTasksWaitingForBits ), ( uxBitsToWaitFor | eventCLEAR_EVENTS_ON_EXIT_BIT | eventWAIT_FOR_ALL_BITS ), xTicksToWait );

				/* 此赋值已过时，因为 uxReturn 将在任务解除阻塞后设置，
				但如果省略此赋值，一些编译器会错误地生成关于 uxReturn
				在未设置的情况下被返回的警告。 */
				uxReturn = 0;
			}
			else
			{
				/* 集合点位未设置，但未指定阻塞时间——
				只需返回当前事件位值。 */
				uxReturn = pxEventBits->uxEventBits;
				xTimeoutOccurred = pdTRUE;
			}
		}
	}
	xAlreadyYielded = xTaskResumeAll();

	if( xTicksToWait != ( TickType_t ) 0 )
	{
		if( xAlreadyYielded == pdFALSE )
		{
			portYIELD_WITHIN_API();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* 任务阻塞以等待其所需的位被设置——此时所需的位要么已设置，
		要么阻塞时间已过期。如果所需的位已设置，它们将被存储在任务的
		事件列表项中，现在应该检索然后清除。 */
		uxReturn = uxTaskResetEventItemValue();

		if( ( uxReturn & eventUNBLOCKED_DUE_TO_BIT_SET ) == ( EventBits_t ) 0 )
		{
			/* 任务超时，只需返回当前事件位值。 */
			taskENTER_CRITICAL();
			{
				uxReturn = pxEventBits->uxEventBits;

				/* 虽然任务是因为在等待的位被设置之前超时来到这里的，
				但有可能在它解除阻塞后，另一个任务已经设置了这些位。
				如果是这种情况，则需要在退出前清除这些位。 */
				if( ( uxReturn & uxBitsToWaitFor ) == uxBitsToWaitFor )
				{
					pxEventBits->uxEventBits &= ~uxBitsToWaitFor;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			taskEXIT_CRITICAL();

			xTimeoutOccurred = pdTRUE;
		}
		else
		{
			/* 任务因为位被设置而解除阻塞。 */
		}

		/* 任务阻塞时可能设置的控制位不应被返回。 */
		uxReturn &= ~eventEVENT_BITS_CONTROL_BYTES;
	}

	traceEVENT_GROUP_SYNC_END( xEventGroup, uxBitsToSet, uxBitsToWaitFor, xTimeoutOccurred );

	/* 防止未使用跟踪宏时的编译器警告。 */
	( void ) xTimeoutOccurred;

	return uxReturn;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupWaitBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait )
{
EventGroup_t *pxEventBits = xEventGroup;
EventBits_t uxReturn, uxControlBits = 0;
BaseType_t xWaitConditionMet, xAlreadyYielded;
BaseType_t xTimeoutOccurred = pdFALSE;

	/* 检查用户没有试图等待内核自身使用的位，
	并且至少请求了一个位。 */
	configASSERT( xEventGroup );
	configASSERT( ( uxBitsToWaitFor & eventEVENT_BITS_CONTROL_BYTES ) == 0 );
	configASSERT( uxBitsToWaitFor != 0 );
	#if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
	{
		configASSERT( !( ( xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED ) && ( xTicksToWait != 0 ) ) );
	}
	#endif

	vTaskSuspendAll();
	{
		const EventBits_t uxCurrentEventBits = pxEventBits->uxEventBits;

		/* 检查等待条件是否已经满足。 */
		xWaitConditionMet = prvTestWaitCondition( uxCurrentEventBits, uxBitsToWaitFor, xWaitForAllBits );

		if( xWaitConditionMet != pdFALSE )
		{
			/* 等待条件已经满足，因此无需阻塞。 */
			uxReturn = uxCurrentEventBits;
			xTicksToWait = ( TickType_t ) 0;

			/* 如果请求，则清除等待位。 */
			if( xClearOnExit != pdFALSE )
			{
				pxEventBits->uxEventBits &= ~uxBitsToWaitFor;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else if( xTicksToWait == ( TickType_t ) 0 )
		{
			/* 等待条件尚未满足，但未指定阻塞时间，
			因此只需返回当前值。 */
			uxReturn = uxCurrentEventBits;
			xTimeoutOccurred = pdTRUE;
		}
		else
		{
			/* 任务将阻塞以等待所需的位被设置。uxControlBits 用于记住
			此 xEventGroupWaitBits() 调用的指定行为——
			以便在事件位解除任务阻塞时使用。 */
			if( xClearOnExit != pdFALSE )
			{
				uxControlBits |= eventCLEAR_EVENTS_ON_EXIT_BIT;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			if( xWaitForAllBits != pdFALSE )
			{
				uxControlBits |= eventWAIT_FOR_ALL_BITS;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			/* 将调用任务正在等待的位存储在任务的事件列表项中，
			以便内核在找到匹配时知道。然后进入阻塞状态。 */
			vTaskPlaceOnUnorderedEventList( &( pxEventBits->xTasksWaitingForBits ), ( uxBitsToWaitFor | uxControlBits ), xTicksToWait );

			/* 这已过时，因为它将在任务解除阻塞后设置，
			但如果不这样做，一些编译器会错误地生成关于变量
			在未设置的情况下被返回的警告。 */
			uxReturn = 0;

			traceEVENT_GROUP_WAIT_BITS_BLOCK( xEventGroup, uxBitsToWaitFor );
		}
	}
	xAlreadyYielded = xTaskResumeAll();

	if( xTicksToWait != ( TickType_t ) 0 )
	{
		if( xAlreadyYielded == pdFALSE )
		{
			portYIELD_WITHIN_API();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* 任务阻塞以等待其所需的位被设置——此时所需的位要么已设置，
		要么阻塞时间已过期。如果所需的位已设置，它们将被存储在任务的
		事件列表项中，现在应该检索然后清除。 */
		uxReturn = uxTaskResetEventItemValue();

		if( ( uxReturn & eventUNBLOCKED_DUE_TO_BIT_SET ) == ( EventBits_t ) 0 )
		{
			taskENTER_CRITICAL();
			{
				/* 任务超时，只需返回当前事件位值。 */
				uxReturn = pxEventBits->uxEventBits;

				/* 事件位可能在此任务离开阻塞状态和再次运行之间被更新。 */
				if( prvTestWaitCondition( uxReturn, uxBitsToWaitFor, xWaitForAllBits ) != pdFALSE )
				{
					if( xClearOnExit != pdFALSE )
					{
						pxEventBits->uxEventBits &= ~uxBitsToWaitFor;
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
				xTimeoutOccurred = pdTRUE;
			}
			taskEXIT_CRITICAL();
		}
		else
		{
			/* 任务因为位被设置而解除阻塞。 */
		}

		/* 任务阻塞了，因此控制位可能已被设置。 */
		uxReturn &= ~eventEVENT_BITS_CONTROL_BYTES;
	}
	traceEVENT_GROUP_WAIT_BITS_END( xEventGroup, uxBitsToWaitFor, xTimeoutOccurred );

	/* 防止未使用跟踪宏时的编译器警告。 */
	( void ) xTimeoutOccurred;

	return uxReturn;
}
/*-----------------------------------------------------------*/

EventBits_t xEventGroupClearBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear )
{
EventGroup_t *pxEventBits = xEventGroup;
EventBits_t uxReturn;

	/* 检查用户没有试图清除内核自身使用的位。 */
	configASSERT( xEventGroup );
	configASSERT( ( uxBitsToClear & eventEVENT_BITS_CONTROL_BYTES ) == 0 );

	taskENTER_CRITICAL();
	{
		traceEVENT_GROUP_CLEAR_BITS( xEventGroup, uxBitsToClear );

		/* 返回的值是位被清除之前的事件组值。 */
		uxReturn = pxEventBits->uxEventBits;

		/* 清除这些位。 */
		pxEventBits->uxEventBits &= ~uxBitsToClear;
	}
	taskEXIT_CRITICAL();

	return uxReturn;
}
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 1 ) )

	BaseType_t xEventGroupClearBitsFromISR( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear )
	{
		BaseType_t xReturn;

		traceEVENT_GROUP_CLEAR_BITS_FROM_ISR( xEventGroup, uxBitsToClear );
		xReturn = xTimerPendFunctionCallFromISR( vEventGroupClearBitsCallback, ( void * ) xEventGroup, ( uint32_t ) uxBitsToClear, NULL ); /*lint !e9087 无法避免强制转换为 void*，因为这是一个不特定于此用例的通用回调函数。回调会转换回原始类型，因此是安全的。 */

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

EventBits_t xEventGroupGetBitsFromISR( EventGroupHandle_t xEventGroup )
{
UBaseType_t uxSavedInterruptStatus;
EventGroup_t const * const pxEventBits = xEventGroup;
EventBits_t uxReturn;

	uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		uxReturn = pxEventBits->uxEventBits;
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

	return uxReturn;
} /*lint !e818 EventGroupHandle_t 是在其他函数中使用的 typedef，因此不能是指向 const 的指针。 */
/*-----------------------------------------------------------*/

EventBits_t xEventGroupSetBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet )
{
ListItem_t *pxListItem, *pxNext;
ListItem_t const *pxListEnd;
List_t const * pxList;
EventBits_t uxBitsToClear = 0, uxBitsWaitedFor, uxControlBits;
EventGroup_t *pxEventBits = xEventGroup;
BaseType_t xMatchFound = pdFALSE;

	/* 检查用户没有试图设置内核自身使用的位。 */
	configASSERT( xEventGroup );
	configASSERT( ( uxBitsToSet & eventEVENT_BITS_CONTROL_BYTES ) == 0 );

	pxList = &( pxEventBits->xTasksWaitingForBits );
	pxListEnd = listGET_END_MARKER( pxList ); /*lint !e826 !e740 !e9087 使用迷你链表结构体作为链表末尾以节省 RAM。此用法已经过检查且有效。 */
	vTaskSuspendAll();
	{
		traceEVENT_GROUP_SET_BITS( xEventGroup, uxBitsToSet );

		pxListItem = listGET_HEAD_ENTRY( pxList );

		/* 设置这些位。 */
		pxEventBits->uxEventBits |= uxBitsToSet;

		/* 查看新的位值是否应该解除任何任务的阻塞。 */
		while( pxListItem != pxListEnd )
		{
			pxNext = listGET_NEXT( pxListItem );
			uxBitsWaitedFor = listGET_LIST_ITEM_VALUE( pxListItem );
			xMatchFound = pdFALSE;

			/* 将等待的位与控制位分开。 */
			uxControlBits = uxBitsWaitedFor & eventEVENT_BITS_CONTROL_BYTES;
			uxBitsWaitedFor &= ~eventEVENT_BITS_CONTROL_BYTES;

			if( ( uxControlBits & eventWAIT_FOR_ALL_BITS ) == ( EventBits_t ) 0 )
			{
				/* 仅等待单个位被设置。 */
				if( ( uxBitsWaitedFor & pxEventBits->uxEventBits ) != ( EventBits_t ) 0 )
				{
					xMatchFound = pdTRUE;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else if( ( uxBitsWaitedFor & pxEventBits->uxEventBits ) == uxBitsWaitedFor )
			{
				/* 所有位都已设置。 */
				xMatchFound = pdTRUE;
			}
			else
			{
				/* 需要所有位都设置，但并非所有位都已设置。 */
			}

			if( xMatchFound != pdFALSE )
			{
				/* 位匹配。退出时是否应清除这些位？ */
				if( ( uxControlBits & eventCLEAR_EVENTS_ON_EXIT_BIT ) != ( EventBits_t ) 0 )
				{
					uxBitsToClear |= uxBitsWaitedFor;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}

				/* 在从事件链表中移除任务之前，将实际的事件标志值存储在
				任务的事件列表项中。设置 eventUNBLOCKED_DUE_TO_BIT_SET 位，
				以便任务知道它是由于所需位匹配而解除阻塞的，
				而不是因为超时。 */
				vTaskRemoveFromUnorderedEventList( pxListItem, pxEventBits->uxEventBits | eventUNBLOCKED_DUE_TO_BIT_SET );
			}

			/* 移动到下一个列表项。注意这里不使用 pxListItem->pxNext，
			因为列表项可能已从事件链表中移除并插入到就绪/挂起读取链表中。 */
			pxListItem = pxNext;
		}

		/* 清除在控制字中设置了 eventCLEAR_EVENTS_ON_EXIT_BIT 位的
		任何匹配的位。 */
		pxEventBits->uxEventBits &= ~uxBitsToClear;
	}
	( void ) xTaskResumeAll();

	return pxEventBits->uxEventBits;
}
/*-----------------------------------------------------------*/

void vEventGroupDelete( EventGroupHandle_t xEventGroup )
{
EventGroup_t *pxEventBits = xEventGroup;
const List_t *pxTasksWaitingForBits = &( pxEventBits->xTasksWaitingForBits );

	vTaskSuspendAll();
	{
		traceEVENT_GROUP_DELETE( xEventGroup );

		while( listCURRENT_LIST_LENGTH( pxTasksWaitingForBits ) > ( UBaseType_t ) 0 )
		{
			/* 解除任务阻塞，返回 0，因为事件链表正在被删除，
			因此不能有任何位被设置。 */
			configASSERT( pxTasksWaitingForBits->xListEnd.pxNext != ( const ListItem_t * ) &( pxTasksWaitingForBits->xListEnd ) );
			vTaskRemoveFromUnorderedEventList( pxTasksWaitingForBits->xListEnd.pxNext, eventUNBLOCKED_DUE_TO_BIT_SET );
		}

		#if( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 0 ) )
		{
			/* 事件组只能动态分配——再次释放它。 */
			vPortFree( pxEventBits );
		}
		#elif( ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
		{
			/* 事件组可能是静态或动态分配的，因此在尝试释放内存之前进行检查。 */
			if( pxEventBits->ucStaticallyAllocated == ( uint8_t ) pdFALSE )
			{
				vPortFree( pxEventBits );
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
	}
	( void ) xTaskResumeAll();
}
/*-----------------------------------------------------------*/

/* 仅供内部使用——执行从中断挂起的"设置位"命令。 */
void vEventGroupSetBitsCallback( void *pvEventGroup, const uint32_t ulBitsToSet )
{
	( void ) xEventGroupSetBits( pvEventGroup, ( EventBits_t ) ulBitsToSet ); /*lint !e9079 无法避免强制转换为 void*，因为这是通用定时器回调原型。回调会转换回原始类型，因此是安全的。 */
}
/*-----------------------------------------------------------*/

/* 仅供内部使用——执行从中断挂起的"清除位"命令。 */
void vEventGroupClearBitsCallback( void *pvEventGroup, const uint32_t ulBitsToClear )
{
	( void ) xEventGroupClearBits( pvEventGroup, ( EventBits_t ) ulBitsToClear ); /*lint !e9079 无法避免强制转换为 void*，因为这是通用定时器回调原型。回调会转换回原始类型，因此是安全的。 */
}
/*-----------------------------------------------------------*/

static BaseType_t prvTestWaitCondition( const EventBits_t uxCurrentEventBits, const EventBits_t uxBitsToWaitFor, const BaseType_t xWaitForAllBits )
{
BaseType_t xWaitConditionMet = pdFALSE;

	if( xWaitForAllBits == pdFALSE )
	{
		/* 任务只需要等待 uxBitsToWaitFor 中的一个位被设置。
		已经有位被设置了吗？ */
		if( ( uxCurrentEventBits & uxBitsToWaitFor ) != ( EventBits_t ) 0 )
		{
			xWaitConditionMet = pdTRUE;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		/* 任务需要等待 uxBitsToWaitFor 中的所有位被设置。
		它们已经被设置了吗？ */
		if( ( uxCurrentEventBits & uxBitsToWaitFor ) == uxBitsToWaitFor )
		{
			xWaitConditionMet = pdTRUE;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}

	return xWaitConditionMet;
}
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 1 ) )

	BaseType_t xEventGroupSetBitsFromISR( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet, BaseType_t *pxHigherPriorityTaskWoken )
	{
	BaseType_t xReturn;

		traceEVENT_GROUP_SET_BITS_FROM_ISR( xEventGroup, uxBitsToSet );
		xReturn = xTimerPendFunctionCallFromISR( vEventGroupSetBitsCallback, ( void * ) xEventGroup, ( uint32_t ) uxBitsToSet, pxHigherPriorityTaskWoken ); /*lint !e9087 无法避免强制转换为 void*，因为这是不特定于此用例的通用回调函数。回调会转换回原始类型，因此是安全的。 */

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

#if (configUSE_TRACE_FACILITY == 1)

	UBaseType_t uxEventGroupGetNumber( void* xEventGroup )
	{
	UBaseType_t xReturn;
	EventGroup_t const *pxEventBits = ( EventGroup_t * ) xEventGroup; /*lint !e9087 !e9079 EventGroupHandle_t 是指向 EventGroup_t 的指针，但为了数据隐藏目的，EventGroupHandle_t 在此文件之外保持不透明。 */

		if( xEventGroup == NULL )
		{
			xReturn = 0;
		}
		else
		{
			xReturn = pxEventBits->uxEventGroupNumber;
		}

		return xReturn;
	}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	void vEventGroupSetNumber( void * xEventGroup, UBaseType_t uxEventGroupNumber )
	{
		( ( EventGroup_t * ) xEventGroup )->uxEventGroupNumber = uxEventGroupNumber; /*lint !e9087 !e9079 EventGroupHandle_t 是指向 EventGroup_t 的指针，但为了数据隐藏目的，EventGroupHandle_t 在此文件之外保持不透明。 */
	}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/
