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

#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

/* 如果不使用协程，则移除整个文件。 */
#if( configUSE_CO_ROUTINES != 0 )

/*
 * 一些内核感知调试器要求查看的数据是全局的，而不是文件作用域的。
 */
#ifdef portREMOVE_STATIC_QUALIFIER
	#define static
#endif


/* 就绪和阻塞协程的链表。 --------------------*/
static List_t pxReadyCoRoutineLists[ configMAX_CO_ROUTINE_PRIORITIES ];	/*< 按优先级排列的就绪协程。 */
static List_t xDelayedCoRoutineList1;									/*< 延迟的协程。 */
static List_t xDelayedCoRoutineList2;									/*< 延迟的协程（使用两个链表——一个用于已溢出当前 tick 计数的延迟）。 */
static List_t * pxDelayedCoRoutineList;									/*< 指向当前正在使用的延迟协程链表。 */
static List_t * pxOverflowDelayedCoRoutineList;							/*< 指向当前用于保存已溢出当前 tick 计数的协程的延迟协程链表。 */
static List_t xPendingReadyCoRoutineList;								/*< 保存已被外部事件就绪的协程。它们不能直接添加到就绪链表中，因为就绪链表不能被中断访问。 */

/* 其他文件私有变量。 --------------------------------*/
CRCB_t * pxCurrentCoRoutine = NULL;
static UBaseType_t uxTopCoRoutineReadyPriority = 0;
static TickType_t xCoRoutineTickCount = 0, xLastTickCount = 0, xPassedTicks = 0;

/* 协程创建时的初始状态。 */
#define corINITIAL_STATE	( 0 )

/*
 * 将由 pxCRCB 表示的协程放入相应优先级的就绪队列中。
 * 它被插入到链表的末尾。
 *
 * 此宏访问协程就绪链表，因此不能在 ISR 中使用。
 */
#define prvAddCoRoutineToReadyQueue( pxCRCB )																		\
{																													\
	if( pxCRCB->uxPriority > uxTopCoRoutineReadyPriority )															\
	{																												\
		uxTopCoRoutineReadyPriority = pxCRCB->uxPriority;															\
	}																												\
	vListInsertEnd( ( List_t * ) &( pxReadyCoRoutineLists[ pxCRCB->uxPriority ] ), &( pxCRCB->xGenericListItem ) );	\
}

/*
 * 用于初始化调度器使用的所有链表的工具函数。
 * 在创建第一个协程时自动调用。
 */
static void prvInitialiseCoRoutineLists( void );

/*
 * 由中断就绪的协程不能直接放入就绪链表中（没有互斥访问）。
 * 相反，它们被放入挂起就绪链表中，以便稍后由协程调度器
 * 移动到就绪链表中。
 */
static void prvCheckPendingReadyList( void );

/*
 * 查看当前延迟的协程链表，看看是否有任何协程需要唤醒。
 *
 * 协程按唤醒时间顺序存储在队列中——
 * 这意味着一旦找到一个定时器尚未到期的协程，
 * 我们就不需要再继续往下查找链表了。
 */
static void prvCheckDelayedList( void );

/*-----------------------------------------------------------*/

BaseType_t xCoRoutineCreate( crCOROUTINE_CODE pxCoRoutineCode, UBaseType_t uxPriority, UBaseType_t uxIndex )
{
BaseType_t xReturn;
CRCB_t *pxCoRoutine;

	/* 分配用于存储协程控制块的内存。 */
	pxCoRoutine = ( CRCB_t * ) pvPortMalloc( sizeof( CRCB_t ) );
	if( pxCoRoutine )
	{
		/* 如果 pxCurrentCoRoutine 为 NULL，则这是第一个要创建的协程，
		需要对协程数据结构进行初始化。 */
		if( pxCurrentCoRoutine == NULL )
		{
			pxCurrentCoRoutine = pxCoRoutine;
			prvInitialiseCoRoutineLists();
		}

		/* 检查优先级是否在允许范围内。 */
		if( uxPriority >= configMAX_CO_ROUTINE_PRIORITIES )
		{
			uxPriority = configMAX_CO_ROUTINE_PRIORITIES - 1;
		}

		/* 根据函数参数填充协程控制块。 */
		pxCoRoutine->uxState = corINITIAL_STATE;
		pxCoRoutine->uxPriority = uxPriority;
		pxCoRoutine->uxIndex = uxIndex;
		pxCoRoutine->pxCoRoutineFunction = pxCoRoutineCode;

		/* 初始化所有其他协程控制块参数。 */
		vListInitialiseItem( &( pxCoRoutine->xGenericListItem ) );
		vListInitialiseItem( &( pxCoRoutine->xEventListItem ) );

		/* 将协程控制块设置为从 ListItem_t 回指的链接。
		这样我们就可以从链表中的通用项找回包含它的 CRCB。 */
		listSET_LIST_ITEM_OWNER( &( pxCoRoutine->xGenericListItem ), pxCoRoutine );
		listSET_LIST_ITEM_OWNER( &( pxCoRoutine->xEventListItem ), pxCoRoutine );

		/* 事件链表始终按优先级排序。 */
		listSET_LIST_ITEM_VALUE( &( pxCoRoutine->xEventListItem ), ( ( TickType_t ) configMAX_CO_ROUTINE_PRIORITIES - ( TickType_t ) uxPriority ) );

		/* 现在协程已经初始化，可以将其添加到正确优先级的
		就绪链表中了。 */
		prvAddCoRoutineToReadyQueue( pxCoRoutine );

		xReturn = pdPASS;
	}
	else
	{
		xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

void vCoRoutineAddToDelayedList( TickType_t xTicksToDelay, List_t *pxEventList )
{
TickType_t xTimeToWake;

	/* 计算唤醒时间——这可能会溢出，但这不是问题。 */
	xTimeToWake = xCoRoutineTickCount + xTicksToDelay;

	/* 在将我们自己添加到阻塞链表之前，必须先将其从就绪链表中移除，
	因为同一个列表项用于两个链表。 */
	( void ) uxListRemove( ( ListItem_t * ) &( pxCurrentCoRoutine->xGenericListItem ) );

	/* 列表项将按唤醒时间顺序插入。 */
	listSET_LIST_ITEM_VALUE( &( pxCurrentCoRoutine->xGenericListItem ), xTimeToWake );

	if( xTimeToWake < xCoRoutineTickCount )
	{
		/* 唤醒时间已溢出。将此项目放入溢出链表。 */
		vListInsert( ( List_t * ) pxOverflowDelayedCoRoutineList, ( ListItem_t * ) &( pxCurrentCoRoutine->xGenericListItem ) );
	}
	else
	{
		/* 唤醒时间没有溢出，因此我们可以使用当前的阻塞链表。 */
		vListInsert( ( List_t * ) pxDelayedCoRoutineList, ( ListItem_t * ) &( pxCurrentCoRoutine->xGenericListItem ) );
	}

	if( pxEventList )
	{
		/* 同时将协程添加到事件链表中。如果这样做，
		则必须在禁用中断的情况下调用此函数。 */
		vListInsert( pxEventList, &( pxCurrentCoRoutine->xEventListItem ) );
	}
}
/*-----------------------------------------------------------*/

static void prvCheckPendingReadyList( void )
{
	/* 是否有任何协程等待移动到就绪链表？这些是由 ISR 就绪的协程。
	ISR 本身不能访问就绪链表。 */
	while( listLIST_IS_EMPTY( &xPendingReadyCoRoutineList ) == pdFALSE )
	{
		CRCB_t *pxUnblockedCRCB;

		/* 挂起就绪链表可以被 ISR 访问。 */
		portDISABLE_INTERRUPTS();
		{
			pxUnblockedCRCB = ( CRCB_t * ) listGET_OWNER_OF_HEAD_ENTRY( (&xPendingReadyCoRoutineList) );
			( void ) uxListRemove( &( pxUnblockedCRCB->xEventListItem ) );
		}
		portENABLE_INTERRUPTS();

		( void ) uxListRemove( &( pxUnblockedCRCB->xGenericListItem ) );
		prvAddCoRoutineToReadyQueue( pxUnblockedCRCB );
	}
}
/*-----------------------------------------------------------*/

static void prvCheckDelayedList( void )
{
CRCB_t *pxCRCB;

	xPassedTicks = xTaskGetTickCount() - xLastTickCount;
	while( xPassedTicks )
	{
		xCoRoutineTickCount++;
		xPassedTicks--;

		/* 如果 tick 计数已溢出，我们需要交换就绪链表。 */
		if( xCoRoutineTickCount == 0 )
		{
			List_t * pxTemp;

			/* Tick 计数已溢出，因此我们需要交换延迟链表。
			如果此时 pxDelayedCoRoutineList 中有任何项目，则存在错误！ */
			pxTemp = pxDelayedCoRoutineList;
			pxDelayedCoRoutineList = pxOverflowDelayedCoRoutineList;
			pxOverflowDelayedCoRoutineList = pxTemp;
		}

		/* 查看此次 tick 是否使某个超时到期。 */
		while( listLIST_IS_EMPTY( pxDelayedCoRoutineList ) == pdFALSE )
		{
			pxCRCB = ( CRCB_t * ) listGET_OWNER_OF_HEAD_ENTRY( pxDelayedCoRoutineList );

			if( xCoRoutineTickCount < listGET_LIST_ITEM_VALUE( &( pxCRCB->xGenericListItem ) ) )
			{
				/* 超时尚未到期。 */
				break;
			}

			portDISABLE_INTERRUPTS();
			{
				/* 事件可能恰好在此临界区之前发生。如果是这种情况，
				那么通用列表项将已被移动到挂起就绪链表，
				以下行仍然有效。同时 pvContainer 参数将已被设置为 NULL，
				因此以下行也仍然有效。 */
				( void ) uxListRemove( &( pxCRCB->xGenericListItem ) );

				/* 协程是否也在等待某个事件？ */
				if( pxCRCB->xEventListItem.pxContainer )
				{
					( void ) uxListRemove( &( pxCRCB->xEventListItem ) );
				}
			}
			portENABLE_INTERRUPTS();

			prvAddCoRoutineToReadyQueue( pxCRCB );
		}
	}

	xLastTickCount = xCoRoutineTickCount;
}
/*-----------------------------------------------------------*/

void vCoRoutineSchedule( void )
{
	/* 查看是否有任何由事件就绪的协程需要移动到就绪链表。 */
	prvCheckPendingReadyList();

	/* 查看是否有任何延迟的协程已超时。 */
	prvCheckDelayedList();

	/* 找到包含就绪协程的最高优先级队列。 */
	while( listLIST_IS_EMPTY( &( pxReadyCoRoutineLists[ uxTopCoRoutineReadyPriority ] ) ) )
	{
		if( uxTopCoRoutineReadyPriority == 0 )
		{
			/* 没有更多协程需要检查。 */
			return;
		}
		--uxTopCoRoutineReadyPriority;
	}

	/* listGET_OWNER_OF_NEXT_ENTRY 遍历链表，因此相同优先级的
	协程可以获得均等的处理器时间份额。 */
	listGET_OWNER_OF_NEXT_ENTRY( pxCurrentCoRoutine, &( pxReadyCoRoutineLists[ uxTopCoRoutineReadyPriority ] ) );

	/* 调用协程。 */
	( pxCurrentCoRoutine->pxCoRoutineFunction )( pxCurrentCoRoutine, pxCurrentCoRoutine->uxIndex );

	return;
}
/*-----------------------------------------------------------*/

static void prvInitialiseCoRoutineLists( void )
{
UBaseType_t uxPriority;

	for( uxPriority = 0; uxPriority < configMAX_CO_ROUTINE_PRIORITIES; uxPriority++ )
	{
		vListInitialise( ( List_t * ) &( pxReadyCoRoutineLists[ uxPriority ] ) );
	}

	vListInitialise( ( List_t * ) &xDelayedCoRoutineList1 );
	vListInitialise( ( List_t * ) &xDelayedCoRoutineList2 );
	vListInitialise( ( List_t * ) &xPendingReadyCoRoutineList );

	/* 开始时 pxDelayedCoRoutineList 使用 list1，
	pxOverflowDelayedCoRoutineList 使用 list2。 */
	pxDelayedCoRoutineList = &xDelayedCoRoutineList1;
	pxOverflowDelayedCoRoutineList = &xDelayedCoRoutineList2;
}
/*-----------------------------------------------------------*/

BaseType_t xCoRoutineRemoveFromEventList( const List_t *pxEventList )
{
CRCB_t *pxUnblockedCRCB;
BaseType_t xReturn;

	/* 此函数从中断中调用。它只能访问事件链表和挂起就绪链表。
	此函数假定已经检查过 pxEventList 不为空。 */
	pxUnblockedCRCB = ( CRCB_t * ) listGET_OWNER_OF_HEAD_ENTRY( pxEventList );
	( void ) uxListRemove( &( pxUnblockedCRCB->xEventListItem ) );
	vListInsertEnd( ( List_t * ) &( xPendingReadyCoRoutineList ), &( pxUnblockedCRCB->xEventListItem ) );

	if( pxUnblockedCRCB->uxPriority >= pxCurrentCoRoutine->uxPriority )
	{
		xReturn = pdTRUE;
	}
	else
	{
		xReturn = pdFALSE;
	}

	return xReturn;
}

#endif /* configUSE_CO_ROUTINES == 0 */
