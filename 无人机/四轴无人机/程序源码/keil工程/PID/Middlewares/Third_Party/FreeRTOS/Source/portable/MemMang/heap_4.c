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

/*
 * pvPortMalloc() 和 vPortFree() 的一个示例实现，它在释放内存块时
 * 合并（聚合）相邻的内存块，从而限制内存碎片。
 *
 * 有关其他实现，请参见 heap_1.c、heap_2.c 和 heap_3.c，
 * 以及 http://www.FreeRTOS.org 的内存管理页面了解更多信息。
 */
#include <stdlib.h>

/* 定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE 可防止 task.h 重新定义
所有 API 函数以使用 MPU 包装器。这应该只在应用程序文件中包含
task.h 时才这样做。 */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
	#error 如果 configSUPPORT_DYNAMIC_ALLOCATION 为 0，则不得使用此文件
#endif

/* 块大小不能变得太小。 */
#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( xHeapStructSize << 1 ) )

/* 假设 8 位字节！ */
#define heapBITS_PER_BYTE		( ( size_t ) 8 )

/* 为堆分配内存。 */
#if( configAPPLICATION_ALLOCATED_HEAP == 1 )
	/* 应用程序编写者已经定义了用于 RTOS 堆的数组
	——可能是为了将其放置在特殊的段或地址中。 */
	extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
	static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif /* configAPPLICATION_ALLOCATED_HEAP */

/* 定义链表结构。这用于按内存地址顺序链接空闲块。 */
typedef struct A_BLOCK_LINK
{
	struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< 链表中的下一个空闲块。 */
	size_t xBlockSize;						/*<< 空闲块的大小。 */
} BlockLink_t;

/*-----------------------------------------------------------*/

/*
 * 将正在释放的内存块插入到空闲内存块链表中的正确位置。
 * 如果被释放的块与其前面和/或后面的内存块相邻，
 * 则将与它们合并。
 */
static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert );

/*
 * 在第一次调用 pvPortMalloc() 时自动调用，
 * 以设置所需的堆结构。
 */
static void prvHeapInit( void );

/*-----------------------------------------------------------*/

/* 放置在每个已分配内存块开头的结构体的大小必须正确字节对齐。 */
static const size_t xHeapStructSize	= ( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

/* 创建一对链表链接来标记链表的开头和结尾。 */
static BlockLink_t xStart, *pxEnd = NULL;

/* 跟踪分配和释放内存的调用次数以及剩余的空闲字节数，
但不反映碎片信息。 */
static size_t xFreeBytesRemaining = 0U;
static size_t xMinimumEverFreeBytesRemaining = 0U;
static size_t xNumberOfSuccessfulAllocations = 0;
static size_t xNumberOfSuccessfulFrees = 0;

/* 设置为 size_t 类型的最高位。当 BlockLink_t 结构体中 xBlockSize
成员的此位被设置时，表示该块属于应用程序。当此位空闲时，
该块仍然是空闲堆空间的一部分。 */
static size_t xBlockAllocatedBit = 0;

/*-----------------------------------------------------------*/

void *pvPortMalloc( size_t xWantedSize )
{
BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
void *pvReturn = NULL;

	vTaskSuspendAll();
	{
		/* 如果这是第一次调用 malloc，则需要对堆进行初始化
		以设置空闲块链表。 */
		if( pxEnd == NULL )
		{
			prvHeapInit();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* 检查请求的块大小是否不太大，以至于最高位被设置。
		BlockLink_t 结构体中块大小成员的最高位用于确定谁拥有该块
		——应用程序还是内核，所以它必须是空闲的。 */
		if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
		{
			/* 增加请求的大小，以便在请求的字节数之外
			还能容纳一个 BlockLink_t 结构体。 */
			if( xWantedSize > 0 )
			{
				xWantedSize += xHeapStructSize;

				/* 确保块始终按所需的字节数对齐。 */
				if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
				{
					/* 需要字节对齐。 */
					xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
					configASSERT( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0 );
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

			if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )
			{
				/* 从起始（最低地址）块开始遍历链表，
				直到找到一个足够大的块。 */
				pxPreviousBlock = &xStart;
				pxBlock = xStart.pxNextFreeBlock;
				while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}

				/* 如果到达了结束标记，则没有找到足够大的块。 */
				if( pxBlock != pxEnd )
				{
					/* 返回指向的内存空间——跳过其开头的
					BlockLink_t 结构体。 */
					pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );

					/* 此块正在被返回使用，因此必须从空闲块
					链表中移除。 */
					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

					/* 如果块比所需的大，可以将其分割成两个。 */
					if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
					{
						/* 此块将被分割成两个。在请求的字节数之后
						创建一个新块。使用 void 强制转换是为了防止
						编译器发出字节对齐警告。 */
						pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
						configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );

						/* 计算从单个块分割出的两个块的大小。 */
						pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
						pxBlock->xBlockSize = xWantedSize;

						/* 将新块插入到空闲块链表中。 */
						prvInsertBlockIntoFreeList( pxNewBlockLink );
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					xFreeBytesRemaining -= pxBlock->xBlockSize;

					if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )
					{
						xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					/* 该块正在被返回——它已被分配并归应用程序
					所有，且没有"下一个"块。 */
					pxBlock->xBlockSize |= xBlockAllocatedBit;
					pxBlock->pxNextFreeBlock = NULL;
					xNumberOfSuccessfulAllocations++;
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
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		traceMALLOC( pvReturn, xWantedSize );
	}
	( void ) xTaskResumeAll();

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	#endif

	configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );
	return pvReturn;
}
/*-----------------------------------------------------------*/

void vPortFree( void *pv )
{
uint8_t *puc = ( uint8_t * ) pv;
BlockLink_t *pxLink;

	if( pv != NULL )
	{
		/* 被释放的内存前面紧挨着一个 BlockLink_t 结构体。 */
		puc -= xHeapStructSize;

		/* 此强制转换是为了防止编译器发出警告。 */
		pxLink = ( void * ) puc;

		/* 检查该块确实已被分配。 */
		configASSERT( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 );
		configASSERT( pxLink->pxNextFreeBlock == NULL );

		if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
		{
			if( pxLink->pxNextFreeBlock == NULL )
			{
				/* 该块正在被返回给堆——它不再被分配。 */
				pxLink->xBlockSize &= ~xBlockAllocatedBit;

				vTaskSuspendAll();
				{
					/* 将此块添加到空闲块链表中。 */
					xFreeBytesRemaining += pxLink->xBlockSize;
					traceFREE( pv, pxLink->xBlockSize );
					prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
					xNumberOfSuccessfulFrees++;
				}
				( void ) xTaskResumeAll();
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
	}
}
/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void )
{
	return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

size_t xPortGetMinimumEverFreeHeapSize( void )
{
	return xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )
{
	/* 这个函数的存在仅仅是为了让链接器保持安静。 */
}
/*-----------------------------------------------------------*/

static void prvHeapInit( void )
{
BlockLink_t *pxFirstFreeBlock;
uint8_t *pucAlignedHeap;
size_t uxAddress;
size_t xTotalHeapSize = configTOTAL_HEAP_SIZE;

	/* 确保堆起始于正确对齐的边界。 */
	uxAddress = ( size_t ) ucHeap;

	if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
	{
		uxAddress += ( portBYTE_ALIGNMENT - 1 );
		uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
		xTotalHeapSize -= uxAddress - ( size_t ) ucHeap;
	}

	pucAlignedHeap = ( uint8_t * ) uxAddress;

	/* xStart 用于保存指向空闲块链表中第一个项的指针。
	使用 void 强制转换是为了防止编译器警告。 */
	xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
	xStart.xBlockSize = ( size_t ) 0;

	/* pxEnd 用于标记空闲块链表的末尾，并插入在堆空间的末尾。 */
	uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
	uxAddress -= xHeapStructSize;
	uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
	pxEnd = ( void * ) uxAddress;
	pxEnd->xBlockSize = 0;
	pxEnd->pxNextFreeBlock = NULL;

	/* 开始时有一个单独的空闲块，其大小占据整个堆空间，
	减去 pxEnd 占用的空间。 */
	pxFirstFreeBlock = ( void * ) pucAlignedHeap;
	pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
	pxFirstFreeBlock->pxNextFreeBlock = pxEnd;

	/* 只有一个块存在——它覆盖了整个可用的堆空间。 */
	xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
	xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

	/* 计算 size_t 变量中最高位的位置。 */
	xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 );
}
/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert )
{
BlockLink_t *pxIterator;
uint8_t *puc;

	/* 遍历链表，直到找到一个地址高于
	正在插入的块的块。 */
	for( pxIterator = &xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
	{
		/* 这里什么都不做，只是迭代到正确的位置。 */
	}

	/* 正在插入的块和它被插入位置后面的块
	是否构成连续的内存块？ */
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	/* 正在插入的块和它被插入位置前面的块
	是否构成连续的内存块？ */
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	{
		if( pxIterator->pxNextFreeBlock != pxEnd )
		{
			/* 将两个块合并成一个大块。 */
			pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		}
		else
		{
			pxBlockToInsert->pxNextFreeBlock = pxEnd;
		}
	}
	else
	{
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}

	/* 如果正在插入的块填补了一个空隙，即与前后的块都合并了，
	那么它的 pxNextFreeBlock 指针已经被设置过了，
	不应在这里再次设置，否则会导致它指向自身。 */
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}
/*-----------------------------------------------------------*/

void vPortGetHeapStats( HeapStats_t *pxHeapStats )
{
BlockLink_t *pxBlock;
size_t xBlocks = 0, xMaxSize = 0, xMinSize = portMAX_DELAY; /* 使用 portMAX_DELAY 作为获取最大值的可移植方式。 */

	vTaskSuspendAll();
	{
		pxBlock = xStart.pxNextFreeBlock;

		/* 如果堆尚未初始化，pxBlock 将为 NULL。
		堆在第一次分配时自动初始化。 */
		if( pxBlock != NULL )
		{
			do
			{
				/* 增加块计数并记录迄今为止看到的最大块。 */
				xBlocks++;

				if( pxBlock->xBlockSize > xMaxSize )
				{
					xMaxSize = pxBlock->xBlockSize;
				}

				if( pxBlock->xBlockSize < xMinSize )
				{
					xMinSize = pxBlock->xBlockSize;
				}

				/* 移动到链表中的下一个块，直到到达最后一个块。 */
				pxBlock = pxBlock->pxNextFreeBlock;
			} while( pxBlock != pxEnd );
		}
	}
	xTaskResumeAll();

	pxHeapStats->xSizeOfLargestFreeBlockInBytes = xMaxSize;
	pxHeapStats->xSizeOfSmallestFreeBlockInBytes = xMinSize;
	pxHeapStats->xNumberOfFreeBlocks = xBlocks;

	taskENTER_CRITICAL();
	{
		pxHeapStats->xAvailableHeapSpaceInBytes = xFreeBytesRemaining;
		pxHeapStats->xNumberOfSuccessfulAllocations = xNumberOfSuccessfulAllocations;
		pxHeapStats->xNumberOfSuccessfulFrees = xNumberOfSuccessfulFrees;
		pxHeapStats->xMinimumEverFreeBytesRemaining = xMinimumEverFreeBytesRemaining;
	}
	taskEXIT_CRITICAL();
}
