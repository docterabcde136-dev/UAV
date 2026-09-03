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


#include <stdlib.h>
#include "FreeRTOS.h"
#include "list.h"

/*-----------------------------------------------------------
 * 公开的链表 API，文档见 list.h
 *----------------------------------------------------------*/

void vListInitialise( List_t * const pxList )
{
	/* 链表结构体中包含一个列表项，用于标记链表的末尾。
	要初始化链表，将链表末尾作为唯一的链表条目插入。 */
	pxList->pxIndex = ( ListItem_t * ) &( pxList->xListEnd );			/*lint !e826 !e740 !e9087 使用迷你链表结构体作为链表末尾以节省 RAM。此用法已经过检查且有效。 */

	/* 链表末尾的值是链表中可能的最大值，以确保它始终保持在链表末尾。 */
	pxList->xListEnd.xItemValue = portMAX_DELAY;

	/* 链表末尾的 next 和 previous 指针指向自身，
	这样我们就知道链表何时为空。 */
	pxList->xListEnd.pxNext = ( ListItem_t * ) &( pxList->xListEnd );	/*lint !e826 !e740 !e9087 使用迷你链表结构体作为链表末尾以节省 RAM。此用法已经过检查且有效。 */
	pxList->xListEnd.pxPrevious = ( ListItem_t * ) &( pxList->xListEnd );/*lint !e826 !e740 !e9087 使用迷你链表结构体作为链表末尾以节省 RAM。此用法已经过检查且有效。 */

	pxList->uxNumberOfItems = ( UBaseType_t ) 0U;

	/* 如果 configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES 设置为 1，
	则将已知值写入链表中。 */
	listSET_LIST_INTEGRITY_CHECK_1_VALUE( pxList );
	listSET_LIST_INTEGRITY_CHECK_2_VALUE( pxList );
}
/*-----------------------------------------------------------*/

void vListInitialiseItem( ListItem_t * const pxItem )
{
	/* 确保该列表项没有被记录为在某个链表中。 */
	pxItem->pxContainer = NULL;

	/* 如果 configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES 设置为 1，
	则将已知值写入列表项中。 */
	listSET_FIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem );
	listSET_SECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem );
}
/*-----------------------------------------------------------*/

void vListInsertEnd( List_t * const pxList, ListItem_t * const pxNewListItem )
{
ListItem_t * const pxIndex = pxList->pxIndex;

	/* 仅在同时定义了 configASSERT() 时有效，这些测试可以捕获
	链表数据结构在内存中被覆盖的情况。它们不会捕获由 FreeRTOS
	的错误配置或使用引起的数据错误。 */
	listTEST_LIST_INTEGRITY( pxList );
	listTEST_LIST_ITEM_INTEGRITY( pxNewListItem );

	/* 将新的列表项插入到 pxList 中，但不是对链表进行排序，
	而是使新列表项成为调用 listGET_OWNER_OF_NEXT_ENTRY() 时
	最后被移除的项。 */
	pxNewListItem->pxNext = pxIndex;
	pxNewListItem->pxPrevious = pxIndex->pxPrevious;

	/* 仅在决策覆盖测试期间使用。 */
	mtCOVERAGE_TEST_DELAY();

	pxIndex->pxPrevious->pxNext = pxNewListItem;
	pxIndex->pxPrevious = pxNewListItem;

	/* 记住该项在哪个链表中。 */
	pxNewListItem->pxContainer = pxList;

	( pxList->uxNumberOfItems )++;
}
/*-----------------------------------------------------------*/

void vListInsert( List_t * const pxList, ListItem_t * const pxNewListItem )
{
ListItem_t *pxIterator;
const TickType_t xValueOfInsertion = pxNewListItem->xItemValue;

	/* 仅在同时定义了 configASSERT() 时有效，这些测试可以捕获
	链表数据结构在内存中被覆盖的情况。它们不会捕获由 FreeRTOS
	的错误配置或使用引起的数据错误。 */
	listTEST_LIST_INTEGRITY( pxList );
	listTEST_LIST_ITEM_INTEGRITY( pxNewListItem );

	/* 将新的列表项插入到链表中，按 xItemValue 值排序。

	如果链表中已经包含一个具有相同项值的列表项，则新的列表项
	应放置在其后面。这确保了存储在就绪链表中的 TCB（它们都
	具有相同的 xItemValue 值）能够共享 CPU。但是，如果 xItemValue
	与末尾标记相同，下面的迭代循环将不会结束。因此首先检查该值，
	并在必要时对算法稍作修改。 */
	if( xValueOfInsertion == portMAX_DELAY )
	{
		pxIterator = pxList->xListEnd.pxPrevious;
	}
	else
	{
		/* *** 注意 ***********************************************************
		如果你发现你的应用程序在这里崩溃，那么可能的原因如下所列。
		此外，请参阅 https://www.freertos.org/FAQHelp.html 获取更多提示，
		并确保定义了 configASSERT()！
		https://www.freertos.org/a00110.html#configASSERT

			1) 栈溢出——
			   请参阅 https://www.freertos.org/Stacks-and-stack-overflow-checking.html
			2) 中断优先级分配不正确，特别是在 Cortex-M 部件上，
			   其中数值高的优先级值表示实际中断优先级低，
			   这可能看起来违反直觉。请参阅
			   https://www.freertos.org/RTOS-Cortex-M3-M4.html 以及
			   https://www.freertos.org/a00110.html 上
			   configMAX_SYSCALL_INTERRUPT_PRIORITY 的定义
			3) 在临界区内部或调度器挂起时调用 API 函数，
			   或从中断中调用不以 "FromISR" 结尾的 API 函数。
			4) 在队列或信号量初始化之前或在调度器启动之前
			   使用它们（在调用 vTaskStartScheduler() 之前
			   中断是否已经在触发？）。
		**********************************************************************/

		for( pxIterator = ( ListItem_t * ) &( pxList->xListEnd ); pxIterator->pxNext->xItemValue <= xValueOfInsertion; pxIterator = pxIterator->pxNext ) /*lint !e826 !e740 !e9087 使用迷你链表结构体作为链表末尾以节省 RAM。此用法已经过检查且有效。 *//*lint !e440 迭代器移动到不同的值，不是 xValueOfInsertion。 */
		{
			/* 这里什么都不做，只是迭代到所需的插入位置。 */
		}
	}

	pxNewListItem->pxNext = pxIterator->pxNext;
	pxNewListItem->pxNext->pxPrevious = pxNewListItem;
	pxNewListItem->pxPrevious = pxIterator;
	pxIterator->pxNext = pxNewListItem;

	/* 记住该项在哪个链表中。这允许以后快速移除该项。 */
	pxNewListItem->pxContainer = pxList;

	( pxList->uxNumberOfItems )++;
}
/*-----------------------------------------------------------*/

UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove )
{
/* 列表项知道它在哪个链表中。从列表项获取链表。 */
List_t * const pxList = pxItemToRemove->pxContainer;

	pxItemToRemove->pxNext->pxPrevious = pxItemToRemove->pxPrevious;
	pxItemToRemove->pxPrevious->pxNext = pxItemToRemove->pxNext;

	/* 仅在决策覆盖测试期间使用。 */
	mtCOVERAGE_TEST_DELAY();

	/* 确保索引保持指向一个有效的项。 */
	if( pxList->pxIndex == pxItemToRemove )
	{
		pxList->pxIndex = pxItemToRemove->pxPrevious;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	pxItemToRemove->pxContainer = NULL;
	( pxList->uxNumberOfItems )--;

	return pxList->uxNumberOfItems;
}
/*-----------------------------------------------------------*/
