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
#include <stdint.h>
#include <string.h>

/* 定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE 可防止 task.h 重新定义
所有 API 函数以使用 MPU 包装器。这应该只在应用程序文件中包含
task.h 时才这样做。 */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* FreeRTOS 包含文件。 */
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#if( configUSE_TASK_NOTIFICATIONS != 1 )
	#error 要构建 stream_buffer.c，configUSE_TASK_NOTIFICATIONS 必须设置为 1
#endif

/* 抑制 Lint e961、e9021 和 e750 作为 MISRA 例外，理由是
MPU 移植需要为上述头文件定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE，
但在此文件中不需要，以便生成正确的特权与非特权链接和放置。 */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e961 !e750 !e9021. */

/* 如果用户没有提供应用程序特定的 Rx 通知宏，
或者 #defined 了通知宏，则提供使用任务通知的默认实现。 */
/*lint -save -e9026 此处允许并需要类似函数的宏，以便它们可以被覆盖。 */
#ifndef sbRECEIVE_COMPLETED
	#define sbRECEIVE_COMPLETED( pxStreamBuffer )										\
		vTaskSuspendAll();																\
		{																				\
			if( ( pxStreamBuffer )->xTaskWaitingToSend != NULL )						\
			{																			\
				( void ) xTaskNotify( ( pxStreamBuffer )->xTaskWaitingToSend,			\
									  ( uint32_t ) 0,									\
									  eNoAction );										\
				( pxStreamBuffer )->xTaskWaitingToSend = NULL;							\
			}																			\
		}																				\
		( void ) xTaskResumeAll();
#endif /* sbRECEIVE_COMPLETED */

#ifndef sbRECEIVE_COMPLETED_FROM_ISR
	#define sbRECEIVE_COMPLETED_FROM_ISR( pxStreamBuffer,								\
										  pxHigherPriorityTaskWoken )					\
	{																					\
	UBaseType_t uxSavedInterruptStatus;													\
																						\
		uxSavedInterruptStatus = ( UBaseType_t ) portSET_INTERRUPT_MASK_FROM_ISR();		\
		{																				\
			if( ( pxStreamBuffer )->xTaskWaitingToSend != NULL )						\
			{																			\
				( void ) xTaskNotifyFromISR( ( pxStreamBuffer )->xTaskWaitingToSend,	\
											 ( uint32_t ) 0,							\
											 eNoAction,									\
											 pxHigherPriorityTaskWoken );				\
				( pxStreamBuffer )->xTaskWaitingToSend = NULL;							\
			}																			\
		}																				\
		portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );					\
	}
#endif /* sbRECEIVE_COMPLETED_FROM_ISR */

/* 如果用户没有提供应用程序特定的 Tx 通知宏，
或者 #defined 了通知宏，则提供使用任务通知的默认实现。 */
#ifndef sbSEND_COMPLETED
	#define sbSEND_COMPLETED( pxStreamBuffer )											\
		vTaskSuspendAll();																\
		{																				\
			if( ( pxStreamBuffer )->xTaskWaitingToReceive != NULL )						\
			{																			\
				( void ) xTaskNotify( ( pxStreamBuffer )->xTaskWaitingToReceive,		\
									  ( uint32_t ) 0,									\
									  eNoAction );										\
				( pxStreamBuffer )->xTaskWaitingToReceive = NULL;						\
			}																			\
		}																				\
		( void ) xTaskResumeAll();
#endif /* sbSEND_COMPLETED */

#ifndef sbSEND_COMPLETE_FROM_ISR
	#define sbSEND_COMPLETE_FROM_ISR( pxStreamBuffer, pxHigherPriorityTaskWoken )		\
	{																					\
	UBaseType_t uxSavedInterruptStatus;													\
																						\
		uxSavedInterruptStatus = ( UBaseType_t ) portSET_INTERRUPT_MASK_FROM_ISR();		\
		{																				\
			if( ( pxStreamBuffer )->xTaskWaitingToReceive != NULL )						\
			{																			\
				( void ) xTaskNotifyFromISR( ( pxStreamBuffer )->xTaskWaitingToReceive,	\
											 ( uint32_t ) 0,							\
											 eNoAction,									\
											 pxHigherPriorityTaskWoken );				\
				( pxStreamBuffer )->xTaskWaitingToReceive = NULL;						\
			}																			\
		}																				\
		portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );					\
	}
#endif /* sbSEND_COMPLETE_FROM_ISR */
/*lint -restore (9026) */

/* 用于保存缓冲区中消息长度的字节数。 */
#define sbBYTES_TO_STORE_MESSAGE_LENGTH ( sizeof( configMESSAGE_BUFFER_LENGTH_TYPE ) )

/* 存储在流缓冲区 ucFlags 字段中的位。 */
#define sbFLAGS_IS_MESSAGE_BUFFER		( ( uint8_t ) 1 ) /* 如果流缓冲区作为消息缓冲区创建则设置，在这种情况下它保存离散消息而不是流。 */
#define sbFLAGS_IS_STATICALLY_ALLOCATED ( ( uint8_t ) 2 ) /* 如果流缓冲区使用静态分配的内存创建则设置。 */

/*-----------------------------------------------------------*/

/* 保存缓冲区状态信息的结构体。 */
typedef struct StreamBufferDef_t /*lint !e9058 风格约定使用标签。 */
{
	volatile size_t xTail;				/* 缓冲区中下一个要读取项的索引。 */
	volatile size_t xHead;				/* 缓冲区中下一个要写入项的索引。 */
	size_t xLength;						/* pucBuffer 指向的缓冲区的长度。 */
	size_t xTriggerLevelBytes;			/* 在解除等待数据的任务的阻塞之前，流缓冲区中必须包含的字节数。 */
	volatile TaskHandle_t xTaskWaitingToReceive; /* 保存等待数据的任务的句柄，如果没有任务等待则为 NULL。 */
	volatile TaskHandle_t xTaskWaitingToSend;	/* 保存等待向已满的消息缓冲区发送数据的任务的句柄。 */
	uint8_t *pucBuffer;					/* 指向缓冲区本身——即存储通过缓冲区传递的数据的 RAM。 */
	uint8_t ucFlags;

	#if ( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t uxStreamBufferNumber;		/* 用于跟踪目的。 */
	#endif
} StreamBuffer_t;

/*
 * 缓冲区中可供读取的字节数。
 */
static size_t prvBytesInBuffer( const StreamBuffer_t * const pxStreamBuffer ) PRIVILEGED_FUNCTION;

/*
 * 将来自 pucData 的 xCount 个字节添加到 pxStreamBuffer 消息缓冲区中。
 * 返回写入的字节数，成功时等于 xCount，如果缓冲区中没有足够的空间
 * 则返回 0（在这种情况下没有数据写入缓冲区）。
 */
static size_t prvWriteBytesToBuffer( StreamBuffer_t * const pxStreamBuffer, const uint8_t *pucData, size_t xCount ) PRIVILEGED_FUNCTION;

/*
 * 如果流缓冲区被用作消息缓冲区，则从缓冲区读取整个消息。
 * 如果流缓冲区被用作流缓冲区，则从缓冲区读取尽可能多的字节。
 * 调用 prvReadBytesFromBuffer() 实际从缓冲区的数据存储区域提取字节。
 */
static size_t prvReadMessageFromBuffer( StreamBuffer_t *pxStreamBuffer,
										void *pvRxData,
										size_t xBufferLengthBytes,
										size_t xBytesAvailable,
										size_t xBytesToStoreMessageLength ) PRIVILEGED_FUNCTION;

/*
 * 如果流缓冲区被用作消息缓冲区，则将整个消息写入缓冲区。
 * 如果流缓冲区被用作流缓冲区，则将尽可能多的字节写入缓冲区。
 * 调用 prvWriteBytestoBuffer() 实际将字节发送到缓冲区的数据存储区域。
 */
static size_t prvWriteMessageToBuffer(  StreamBuffer_t * const pxStreamBuffer,
										const void * pvTxData,
										size_t xDataLengthBytes,
										size_t xSpace,
										size_t xRequiredSpace ) PRIVILEGED_FUNCTION;

/*
 * 从 pxStreamBuffer 消息缓冲区读取 xMaxCount 个字节，
 * 并将它们写入 pucData。
 */
static size_t prvReadBytesFromBuffer( StreamBuffer_t *pxStreamBuffer,
									  uint8_t *pucData,
									  size_t xMaxCount,
									  size_t xBytesAvailable ) PRIVILEGED_FUNCTION;

/*
 * 由 pxStreamBufferCreate() 和 pxStreamBufferCreateStatic() 调用，
 * 以初始化新创建的流缓冲区结构体的成员。
 */
static void prvInitialiseNewStreamBuffer( StreamBuffer_t * const pxStreamBuffer,
										  uint8_t * const pucBuffer,
										  size_t xBufferSizeBytes,
										  size_t xTriggerLevelBytes,
										  uint8_t ucFlags ) PRIVILEGED_FUNCTION;

/*-----------------------------------------------------------*/

#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

	StreamBufferHandle_t xStreamBufferGenericCreate( size_t xBufferSizeBytes, size_t xTriggerLevelBytes, BaseType_t xIsMessageBuffer )
	{
	uint8_t *pucAllocatedMemory;
	uint8_t ucFlags;

		/* 如果流缓冲区将用作消息缓冲区（即它将保存离散消息，
		并带有说明下一条消息有多大的少量元数据），请检查缓冲区是否
		足够大以容纳至少一条消息。 */
		if( xIsMessageBuffer == pdTRUE )
		{
			/* 是消息缓冲区但不是静态分配的。 */
			ucFlags = sbFLAGS_IS_MESSAGE_BUFFER;
			configASSERT( xBufferSizeBytes > sbBYTES_TO_STORE_MESSAGE_LENGTH );
		}
		else
		{
			/* 不是消息缓冲区且不是静态分配的。 */
			ucFlags = 0;
			configASSERT( xBufferSizeBytes > 0 );
		}
		configASSERT( xTriggerLevelBytes <= xBufferSizeBytes );

		/* 触发级别为 0 会导致等待任务即使在缓冲区为空时也解除阻塞。 */
		if( xTriggerLevelBytes == ( size_t ) 0 )
		{
			xTriggerLevelBytes = ( size_t ) 1;
		}

		/* 流缓冲区需要一个 StreamBuffer_t 结构体和一个缓冲区。
		两者在单次调用 pvPortMalloc() 中分配。StreamBuffer_t 结构体
		放置在分配的内存的开始处，缓冲区紧随其后。请求的大小会递增，
		以便返回用户期望的空闲空间——这是实现的一个怪癖，
		意味着否则空闲空间将比逻辑上预期的少一个字节。 */
		xBufferSizeBytes++;
		pucAllocatedMemory = ( uint8_t * ) pvPortMalloc( xBufferSizeBytes + sizeof( StreamBuffer_t ) ); /*lint !e9079 malloc() 只返回 void*。 */

		if( pucAllocatedMemory != NULL )
		{
			prvInitialiseNewStreamBuffer( ( StreamBuffer_t * ) pucAllocatedMemory, /* 结构体位于分配的内存的开始处。 */ /*lint !e9087 安全转换，因为分配的内存是对齐的。 */ /*lint !e826 区域不会太小，只要 malloc() 按预期行为并返回对齐的缓冲区，对齐就是有保证的。 */
										   pucAllocatedMemory + sizeof( StreamBuffer_t ),  /* 存储区域紧随其后。 */ /*lint !e9016 对 uint8_t 指针来说，索引超出结构体是有效的，而且存储区域没有对齐要求。 */
										   xBufferSizeBytes,
										   xTriggerLevelBytes,
										   ucFlags );

			traceSTREAM_BUFFER_CREATE( ( ( StreamBuffer_t * ) pucAllocatedMemory ), xIsMessageBuffer );
		}
		else
		{
			traceSTREAM_BUFFER_CREATE_FAILED( xIsMessageBuffer );
		}

		return ( StreamBufferHandle_t ) pucAllocatedMemory; /*lint !e9087 !e826 安全转换，因为分配的内存是对齐的。 */
	}

#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

#if( configSUPPORT_STATIC_ALLOCATION == 1 )

	StreamBufferHandle_t xStreamBufferGenericCreateStatic( size_t xBufferSizeBytes,
														   size_t xTriggerLevelBytes,
														   BaseType_t xIsMessageBuffer,
														   uint8_t * const pucStreamBufferStorageArea,
														   StaticStreamBuffer_t * const pxStaticStreamBuffer )
	{
	StreamBuffer_t * const pxStreamBuffer = ( StreamBuffer_t * ) pxStaticStreamBuffer; /*lint !e740 !e9087 安全转换，因为 StaticStreamBuffer_t 是不透明的 Streambuffer_t。 */
	StreamBufferHandle_t xReturn;
	uint8_t ucFlags;

		configASSERT( pucStreamBufferStorageArea );
		configASSERT( pxStaticStreamBuffer );
		configASSERT( xTriggerLevelBytes <= xBufferSizeBytes );

		/* 触发级别为 0 会导致等待任务即使在缓冲区为空时也解除阻塞。 */
		if( xTriggerLevelBytes == ( size_t ) 0 )
		{
			xTriggerLevelBytes = ( size_t ) 1;
		}

		if( xIsMessageBuffer != pdFALSE )
		{
			/* 静态分配的消息缓冲区。 */
			ucFlags = sbFLAGS_IS_MESSAGE_BUFFER | sbFLAGS_IS_STATICALLY_ALLOCATED;
		}
		else
		{
			/* 静态分配的流缓冲区。 */
			ucFlags = sbFLAGS_IS_STATICALLY_ALLOCATED;
		}

		/* 如果流缓冲区将用作消息缓冲区（即它将保存离散消息，
		并带有说明下一条消息有多大的少量元数据），请检查缓冲区是否
		足够大以容纳至少一条消息。 */
		configASSERT( xBufferSizeBytes > sbBYTES_TO_STORE_MESSAGE_LENGTH );

		#if( configASSERT_DEFINED == 1 )
		{
			/* 健全性检查：用于声明 StaticStreamBuffer_t 类型变量的结构体大小
			等于实际消息缓冲区结构体的大小。 */
			volatile size_t xSize = sizeof( StaticStreamBuffer_t );
			configASSERT( xSize == sizeof( StreamBuffer_t ) );
		} /*lint !e529 如果定义了 configASSERT()，则 xSize 被引用。 */
		#endif /* configASSERT_DEFINED */

		if( ( pucStreamBufferStorageArea != NULL ) && ( pxStaticStreamBuffer != NULL ) )
		{
			prvInitialiseNewStreamBuffer( pxStreamBuffer,
										  pucStreamBufferStorageArea,
										  xBufferSizeBytes,
										  xTriggerLevelBytes,
										  ucFlags );

			/* 记住这是静态分配的，以防以后被删除。 */
			pxStreamBuffer->ucFlags |= sbFLAGS_IS_STATICALLY_ALLOCATED;

			traceSTREAM_BUFFER_CREATE( pxStreamBuffer, xIsMessageBuffer );

			xReturn = ( StreamBufferHandle_t ) pxStaticStreamBuffer; /*lint !e9087 数据隐藏需要强制转换为不透明类型。 */
		}
		else
		{
			xReturn = NULL;
			traceSTREAM_BUFFER_CREATE_STATIC_FAILED( xReturn, xIsMessageBuffer );
		}

		return xReturn;
	}

#endif /* ( configSUPPORT_STATIC_ALLOCATION == 1 ) */
/*-----------------------------------------------------------*/

void vStreamBufferDelete( StreamBufferHandle_t xStreamBuffer )
{
StreamBuffer_t * pxStreamBuffer = xStreamBuffer;

	configASSERT( pxStreamBuffer );

	traceSTREAM_BUFFER_DELETE( xStreamBuffer );

	if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_STATICALLY_ALLOCATED ) == ( uint8_t ) pdFALSE )
	{
		#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
		{
			/* 结构体和缓冲区都是通过单次调用 pvPortMalloc() 分配的，
			因此只需要一次调用 vPortFree()。 */
			vPortFree( ( void * ) pxStreamBuffer ); /*lint !e9087 标准 free() 语义需要 void*，而且 pxStreamBuffer 是由 pvPortMalloc() 分配的。 */
		}
		#else
		{
			/* 不应该可能到达这里，ucFlags 一定已损坏。
			强制触发 assert。 */
			configASSERT( xStreamBuffer == ( StreamBufferHandle_t ) ~0 );
		}
		#endif
	}
	else
	{
		/* 结构体和缓冲区不是动态分配的，不能被释放——
		只是清除结构体，以便将来使用时触发 assert。 */
		( void ) memset( pxStreamBuffer, 0x00, sizeof( StreamBuffer_t ) );
	}
}
/*-----------------------------------------------------------*/

BaseType_t xStreamBufferReset( StreamBufferHandle_t xStreamBuffer )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
BaseType_t xReturn = pdFAIL;

#if( configUSE_TRACE_FACILITY == 1 )
	UBaseType_t uxStreamBufferNumber;
#endif

	configASSERT( pxStreamBuffer );

	#if( configUSE_TRACE_FACILITY == 1 )
	{
		/* 存储流缓冲区编号，以便在重置后可以恢复。 */
		uxStreamBufferNumber = pxStreamBuffer->uxStreamBufferNumber;
	}
	#endif

	/* 只有在没有任务阻塞在消息缓冲区上时才能重置它。 */
	taskENTER_CRITICAL();
	{
		if( pxStreamBuffer->xTaskWaitingToReceive == NULL )
		{
			if( pxStreamBuffer->xTaskWaitingToSend == NULL )
			{
				prvInitialiseNewStreamBuffer( pxStreamBuffer,
											  pxStreamBuffer->pucBuffer,
											  pxStreamBuffer->xLength,
											  pxStreamBuffer->xTriggerLevelBytes,
											  pxStreamBuffer->ucFlags );
				xReturn = pdPASS;

				#if( configUSE_TRACE_FACILITY == 1 )
				{
					pxStreamBuffer->uxStreamBufferNumber = uxStreamBufferNumber;
				}
				#endif

				traceSTREAM_BUFFER_RESET( xStreamBuffer );
			}
		}
	}
	taskEXIT_CRITICAL();

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xStreamBufferSetTriggerLevel( StreamBufferHandle_t xStreamBuffer, size_t xTriggerLevel )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
BaseType_t xReturn;

	configASSERT( pxStreamBuffer );

	/* 触发级别为 0 是无效的。 */
	if( xTriggerLevel == ( size_t ) 0 )
	{
		xTriggerLevel = ( size_t ) 1;
	}

	/* 触发级别是在解除等待数据的任务的阻塞之前，
	流缓冲区中必须包含的字节数。 */
	if( xTriggerLevel <= pxStreamBuffer->xLength )
	{
		pxStreamBuffer->xTriggerLevelBytes = xTriggerLevel;
		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFALSE;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

size_t xStreamBufferSpacesAvailable( StreamBufferHandle_t xStreamBuffer )
{
const StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
size_t xSpace;

	configASSERT( pxStreamBuffer );

	xSpace = pxStreamBuffer->xLength + pxStreamBuffer->xTail;
	xSpace -= pxStreamBuffer->xHead;
	xSpace -= ( size_t ) 1;

	if( xSpace >= pxStreamBuffer->xLength )
	{
		xSpace -= pxStreamBuffer->xLength;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	return xSpace;
}
/*-----------------------------------------------------------*/

size_t xStreamBufferBytesAvailable( StreamBufferHandle_t xStreamBuffer )
{
const StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
size_t xReturn;

	configASSERT( pxStreamBuffer );

	xReturn = prvBytesInBuffer( pxStreamBuffer );
	return xReturn;
}
/*-----------------------------------------------------------*/

size_t xStreamBufferSend( StreamBufferHandle_t xStreamBuffer,
						  const void *pvTxData,
						  size_t xDataLengthBytes,
						  TickType_t xTicksToWait )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
size_t xReturn, xSpace = 0;
size_t xRequiredSpace = xDataLengthBytes;
TimeOut_t xTimeOut;

	configASSERT( pvTxData );
	configASSERT( pxStreamBuffer );

	/* 此发送函数用于写入消息缓冲区和流缓冲区。
	如果这是消息缓冲区，则所需空间必须增加存储消息长度
	所需的字节数。 */
	if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_MESSAGE_BUFFER ) != ( uint8_t ) 0 )
	{
		xRequiredSpace += sbBYTES_TO_STORE_MESSAGE_LENGTH;

		/* 溢出？ */
		configASSERT( xRequiredSpace > xDataLengthBytes );
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	if( xTicksToWait != ( TickType_t ) 0 )
	{
		vTaskSetTimeOutState( &xTimeOut );

		do
		{
			/* 等待消息缓冲区中有所需的空闲字节数。 */
			taskENTER_CRITICAL();
			{
				xSpace = xStreamBufferSpacesAvailable( pxStreamBuffer );

				if( xSpace < xRequiredSpace )
				{
					/* 清除通知状态，因为将要等待空间。 */
					( void ) xTaskNotifyStateClear( NULL );

					/* 应该只有一个写入者。 */
					configASSERT( pxStreamBuffer->xTaskWaitingToSend == NULL );
					pxStreamBuffer->xTaskWaitingToSend = xTaskGetCurrentTaskHandle();
				}
				else
				{
					taskEXIT_CRITICAL();
					break;
				}
			}
			taskEXIT_CRITICAL();

			traceBLOCKING_ON_STREAM_BUFFER_SEND( xStreamBuffer );
			( void ) xTaskNotifyWait( ( uint32_t ) 0, ( uint32_t ) 0, NULL, xTicksToWait );
			pxStreamBuffer->xTaskWaitingToSend = NULL;

		} while( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == pdFALSE );
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	if( xSpace == ( size_t ) 0 )
	{
		xSpace = xStreamBufferSpacesAvailable( pxStreamBuffer );
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	xReturn = prvWriteMessageToBuffer( pxStreamBuffer, pvTxData, xDataLengthBytes, xSpace, xRequiredSpace );

	if( xReturn > ( size_t ) 0 )
	{
		traceSTREAM_BUFFER_SEND( xStreamBuffer, xReturn );

		/* 是否有任务在等待数据？ */
		if( prvBytesInBuffer( pxStreamBuffer ) >= pxStreamBuffer->xTriggerLevelBytes )
		{
			sbSEND_COMPLETED( pxStreamBuffer );
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
		traceSTREAM_BUFFER_SEND_FAILED( xStreamBuffer );
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

size_t xStreamBufferSendFromISR( StreamBufferHandle_t xStreamBuffer,
								 const void *pvTxData,
								 size_t xDataLengthBytes,
								 BaseType_t * const pxHigherPriorityTaskWoken )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
size_t xReturn, xSpace;
size_t xRequiredSpace = xDataLengthBytes;

	configASSERT( pvTxData );
	configASSERT( pxStreamBuffer );

	/* 此发送函数用于写入消息缓冲区和流缓冲区。
	如果这是消息缓冲区，则所需空间必须增加存储消息长度
	所需的字节数。 */
	if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_MESSAGE_BUFFER ) != ( uint8_t ) 0 )
	{
		xRequiredSpace += sbBYTES_TO_STORE_MESSAGE_LENGTH;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	xSpace = xStreamBufferSpacesAvailable( pxStreamBuffer );
	xReturn = prvWriteMessageToBuffer( pxStreamBuffer, pvTxData, xDataLengthBytes, xSpace, xRequiredSpace );

	if( xReturn > ( size_t ) 0 )
	{
		/* 是否有任务在等待数据？ */
		if( prvBytesInBuffer( pxStreamBuffer ) >= pxStreamBuffer->xTriggerLevelBytes )
		{
			sbSEND_COMPLETE_FROM_ISR( pxStreamBuffer, pxHigherPriorityTaskWoken );
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

	traceSTREAM_BUFFER_SEND_FROM_ISR( xStreamBuffer, xReturn );

	return xReturn;
}
/*-----------------------------------------------------------*/

static size_t prvWriteMessageToBuffer( StreamBuffer_t * const pxStreamBuffer,
									   const void * pvTxData,
									   size_t xDataLengthBytes,
									   size_t xSpace,
									   size_t xRequiredSpace )
{
	BaseType_t xShouldWrite;
	size_t xReturn;

	if( xSpace == ( size_t ) 0 )
	{
		/* 无论是流缓冲区还是消息缓冲区，都没有空间可写。 */
		xShouldWrite = pdFALSE;
	}
	else if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_MESSAGE_BUFFER ) == ( uint8_t ) 0 )
	{
		/* 这是流缓冲区，而不是消息缓冲区，因此写入的是字节流
		而不是离散消息。尽可能多地写入字节。 */
		xShouldWrite = pdTRUE;
		xDataLengthBytes = configMIN( xDataLengthBytes, xSpace );
	}
	else if( xSpace >= xRequiredSpace )
	{
		/* 这是消息缓冲区，而不是流缓冲区，并且有足够的空间
		将消息长度和消息本身都写入缓冲区。首先写入数据的长度，
		数据本身将在本函数稍后写入。 */
		xShouldWrite = pdTRUE;
		( void ) prvWriteBytesToBuffer( pxStreamBuffer, ( const uint8_t * ) &( xDataLengthBytes ), sbBYTES_TO_STORE_MESSAGE_LENGTH );
	}
	else
	{
		/* 有可用空间，但不够。 */
		xShouldWrite = pdFALSE;
	}

	if( xShouldWrite != pdFALSE )
	{
		/* 写入数据本身。 */
		xReturn = prvWriteBytesToBuffer( pxStreamBuffer, ( const uint8_t * ) pvTxData, xDataLengthBytes ); /*lint !e9079 存储缓冲区实现为 uint8_t 以便于调整大小、对齐和访问。 */
	}
	else
	{
		xReturn = 0;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

size_t xStreamBufferReceive( StreamBufferHandle_t xStreamBuffer,
							 void *pvRxData,
							 size_t xBufferLengthBytes,
							 TickType_t xTicksToWait )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
size_t xReceivedLength = 0, xBytesAvailable, xBytesToStoreMessageLength;

	configASSERT( pvRxData );
	configASSERT( pxStreamBuffer );

	/* 此接收函数同时由存储离散消息的消息缓冲区和存储连续字节流的
	流缓冲区使用。离散消息包含额外的 sbBYTES_TO_STORE_MESSAGE_LENGTH 字节，
	用于保存消息的长度。 */
	if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_MESSAGE_BUFFER ) != ( uint8_t ) 0 )
	{
		xBytesToStoreMessageLength = sbBYTES_TO_STORE_MESSAGE_LENGTH;
	}
	else
	{
		xBytesToStoreMessageLength = 0;
	}

	if( xTicksToWait != ( TickType_t ) 0 )
	{
		/* 检查是否有数据并清除通知状态必须原子性地执行。 */
		taskENTER_CRITICAL();
		{
			xBytesAvailable = prvBytesInBuffer( pxStreamBuffer );

			/* 如果此函数由消息缓冲区读取调用，则 xBytesToStoreMessageLength
			保存用于保存下一条离散消息长度的字节数。如果此函数由
			流缓冲区读取调用，则 xBytesToStoreMessageLength 将为 0。 */
			if( xBytesAvailable <= xBytesToStoreMessageLength )
			{
				/* 清除通知状态，因为将要等待数据。 */
				( void ) xTaskNotifyStateClear( NULL );

				/* 应该只有一个读取者。 */
				configASSERT( pxStreamBuffer->xTaskWaitingToReceive == NULL );
				pxStreamBuffer->xTaskWaitingToReceive = xTaskGetCurrentTaskHandle();
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		taskEXIT_CRITICAL();

		if( xBytesAvailable <= xBytesToStoreMessageLength )
		{
			/* 等待数据可用。 */
			traceBLOCKING_ON_STREAM_BUFFER_RECEIVE( xStreamBuffer );
			( void ) xTaskNotifyWait( ( uint32_t ) 0, ( uint32_t ) 0, NULL, xTicksToWait );
			pxStreamBuffer->xTaskWaitingToReceive = NULL;

			/* 阻塞后重新检查可用数据。 */
			xBytesAvailable = prvBytesInBuffer( pxStreamBuffer );
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		xBytesAvailable = prvBytesInBuffer( pxStreamBuffer );
	}

	/* 无论是接收离散消息（其中 xBytesToStoreMessageLength 保存
	用于存储消息长度的字节数）还是字节流（其中 xBytesToStoreMessageLength
	为零），可用字节数必须大于 xBytesToStoreMessageLength
	才能从缓冲区读取字节。 */
	if( xBytesAvailable > xBytesToStoreMessageLength )
	{
		xReceivedLength = prvReadMessageFromBuffer( pxStreamBuffer, pvRxData, xBufferLengthBytes, xBytesAvailable, xBytesToStoreMessageLength );

		/* 是否有任务在等待缓冲区中的空间？ */
		if( xReceivedLength != ( size_t ) 0 )
		{
			traceSTREAM_BUFFER_RECEIVE( xStreamBuffer, xReceivedLength );
			sbRECEIVE_COMPLETED( pxStreamBuffer );
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		traceSTREAM_BUFFER_RECEIVE_FAILED( xStreamBuffer );
		mtCOVERAGE_TEST_MARKER();
	}

	return xReceivedLength;
}
/*-----------------------------------------------------------*/

size_t xStreamBufferNextMessageLengthBytes( StreamBufferHandle_t xStreamBuffer )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
size_t xReturn, xBytesAvailable, xOriginalTail;
configMESSAGE_BUFFER_LENGTH_TYPE xTempReturn;

	configASSERT( pxStreamBuffer );

	/* 确保流缓冲区被用作消息缓冲区。 */
	if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_MESSAGE_BUFFER ) != ( uint8_t ) 0 )
	{
		xBytesAvailable = prvBytesInBuffer( pxStreamBuffer );
		if( xBytesAvailable > sbBYTES_TO_STORE_MESSAGE_LENGTH )
		{
			/* 可用字节数大于保存下一条消息长度所需的字节数，
			因此另一条消息可用。返回其长度而不从缓冲区中移除长度字节。
			存储一份 tail 的副本，以便将缓冲区恢复到先前状态，
			因为消息实际上并未从缓冲区中移除。 */
			xOriginalTail = pxStreamBuffer->xTail;
			( void ) prvReadBytesFromBuffer( pxStreamBuffer, ( uint8_t * ) &xTempReturn, sbBYTES_TO_STORE_MESSAGE_LENGTH, xBytesAvailable );
			xReturn = ( size_t ) xTempReturn;
			pxStreamBuffer->xTail = xOriginalTail;
		}
		else
		{
			/* 消息缓冲区中的最小字节数是 (sbBYTES_TO_STORE_MESSAGE_LENGTH + 1)，
			因此如果 xBytesAvailable 小于 sbBYTES_TO_STORE_MESSAGE_LENGTH，
			唯一有效的其他值是 0。 */
			configASSERT( xBytesAvailable == 0 );
			xReturn = 0;
		}
	}
	else
	{
		xReturn = 0;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

size_t xStreamBufferReceiveFromISR( StreamBufferHandle_t xStreamBuffer,
									void *pvRxData,
									size_t xBufferLengthBytes,
									BaseType_t * const pxHigherPriorityTaskWoken )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
size_t xReceivedLength = 0, xBytesAvailable, xBytesToStoreMessageLength;

	configASSERT( pvRxData );
	configASSERT( pxStreamBuffer );

	/* 此接收函数同时由存储离散消息的消息缓冲区和存储连续字节流的
	流缓冲区使用。离散消息包含额外的 sbBYTES_TO_STORE_MESSAGE_LENGTH 字节，
	用于保存消息的长度。 */
	if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_MESSAGE_BUFFER ) != ( uint8_t ) 0 )
	{
		xBytesToStoreMessageLength = sbBYTES_TO_STORE_MESSAGE_LENGTH;
	}
	else
	{
		xBytesToStoreMessageLength = 0;
	}

	xBytesAvailable = prvBytesInBuffer( pxStreamBuffer );

	/* 无论是接收离散消息（其中 xBytesToStoreMessageLength 保存
	用于存储消息长度的字节数）还是字节流（其中 xBytesToStoreMessageLength
	为零），可用字节数必须大于 xBytesToStoreMessageLength
	才能从缓冲区读取字节。 */
	if( xBytesAvailable > xBytesToStoreMessageLength )
	{
		xReceivedLength = prvReadMessageFromBuffer( pxStreamBuffer, pvRxData, xBufferLengthBytes, xBytesAvailable, xBytesToStoreMessageLength );

		/* 是否有任务在等待缓冲区中的空间？ */
		if( xReceivedLength != ( size_t ) 0 )
		{
			sbRECEIVE_COMPLETED_FROM_ISR( pxStreamBuffer, pxHigherPriorityTaskWoken );
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

	traceSTREAM_BUFFER_RECEIVE_FROM_ISR( xStreamBuffer, xReceivedLength );

	return xReceivedLength;
}
/*-----------------------------------------------------------*/

static size_t prvReadMessageFromBuffer( StreamBuffer_t *pxStreamBuffer,
										void *pvRxData,
										size_t xBufferLengthBytes,
										size_t xBytesAvailable,
										size_t xBytesToStoreMessageLength )
{
size_t xOriginalTail, xReceivedLength, xNextMessageLength;
configMESSAGE_BUFFER_LENGTH_TYPE xTempNextMessageLength;

	if( xBytesToStoreMessageLength != ( size_t ) 0 )
	{
		/* 正在接收离散消息。首先接收消息的长度。
		存储一份 tail 的副本，以便在消息长度对提供的缓冲区来说太大时，
		可以将缓冲区恢复到先前状态。 */
		xOriginalTail = pxStreamBuffer->xTail;
		( void ) prvReadBytesFromBuffer( pxStreamBuffer, ( uint8_t * ) &xTempNextMessageLength, xBytesToStoreMessageLength, xBytesAvailable );
		xNextMessageLength = ( size_t ) xTempNextMessageLength;

		/* 将可用字节数减少刚刚读出的字节数。 */
		xBytesAvailable -= xBytesToStoreMessageLength;

		/* 检查用户提供的缓冲区中是否有足够的空间。 */
		if( xNextMessageLength > xBufferLengthBytes )
		{
			/* 用户提供的空间不足以读取消息，
			因此将缓冲区恢复到先前状态（这样消息的长度会再次在缓冲区中）。 */
			pxStreamBuffer->xTail = xOriginalTail;
			xNextMessageLength = 0;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		/* 正在接收字节流（而不是离散消息），
		因此尽可能多地读取字节。 */
		xNextMessageLength = xBufferLengthBytes;
	}

	/* 读取实际数据。 */
	xReceivedLength = prvReadBytesFromBuffer( pxStreamBuffer, ( uint8_t * ) pvRxData, xNextMessageLength, xBytesAvailable ); /*lint !e9079 数据存储区域实现为 uint8_t 数组，以便于调整大小、索引和对齐。 */

	return xReceivedLength;
}
/*-----------------------------------------------------------*/

BaseType_t xStreamBufferIsEmpty( StreamBufferHandle_t xStreamBuffer )
{
const StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
BaseType_t xReturn;
size_t xTail;

	configASSERT( pxStreamBuffer );

	/* 如果没有可用字节则为 true。 */
	xTail = pxStreamBuffer->xTail;
	if( pxStreamBuffer->xHead == xTail )
	{
		xReturn = pdTRUE;
	}
	else
	{
		xReturn = pdFALSE;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xStreamBufferIsFull( StreamBufferHandle_t xStreamBuffer )
{
BaseType_t xReturn;
size_t xBytesToStoreMessageLength;
const StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;

	configASSERT( pxStreamBuffer );

	/* 此通用版本的接收函数同时由存储离散消息的消息缓冲区和存储连续字节流的
	流缓冲区使用。离散消息包含额外的 sbBYTES_TO_STORE_MESSAGE_LENGTH 字节，
	用于保存消息的长度。 */
	if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_MESSAGE_BUFFER ) != ( uint8_t ) 0 )
	{
		xBytesToStoreMessageLength = sbBYTES_TO_STORE_MESSAGE_LENGTH;
	}
	else
	{
		xBytesToStoreMessageLength = 0;
	}

	/* 如果可用空间等于零则为 true。 */
	if( xStreamBufferSpacesAvailable( xStreamBuffer ) <= xBytesToStoreMessageLength )
	{
		xReturn = pdTRUE;
	}
	else
	{
		xReturn = pdFALSE;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xStreamBufferSendCompletedFromISR( StreamBufferHandle_t xStreamBuffer, BaseType_t *pxHigherPriorityTaskWoken )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
BaseType_t xReturn;
UBaseType_t uxSavedInterruptStatus;

	configASSERT( pxStreamBuffer );

	uxSavedInterruptStatus = ( UBaseType_t ) portSET_INTERRUPT_MASK_FROM_ISR();
	{
		if( ( pxStreamBuffer )->xTaskWaitingToReceive != NULL )
		{
			( void ) xTaskNotifyFromISR( ( pxStreamBuffer )->xTaskWaitingToReceive,
										 ( uint32_t ) 0,
										 eNoAction,
										 pxHigherPriorityTaskWoken );
			( pxStreamBuffer )->xTaskWaitingToReceive = NULL;
			xReturn = pdTRUE;
		}
		else
		{
			xReturn = pdFALSE;
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xStreamBufferReceiveCompletedFromISR( StreamBufferHandle_t xStreamBuffer, BaseType_t *pxHigherPriorityTaskWoken )
{
StreamBuffer_t * const pxStreamBuffer = xStreamBuffer;
BaseType_t xReturn;
UBaseType_t uxSavedInterruptStatus;

	configASSERT( pxStreamBuffer );

	uxSavedInterruptStatus = ( UBaseType_t ) portSET_INTERRUPT_MASK_FROM_ISR();
	{
		if( ( pxStreamBuffer )->xTaskWaitingToSend != NULL )
		{
			( void ) xTaskNotifyFromISR( ( pxStreamBuffer )->xTaskWaitingToSend,
										 ( uint32_t ) 0,
										 eNoAction,
										 pxHigherPriorityTaskWoken );
			( pxStreamBuffer )->xTaskWaitingToSend = NULL;
			xReturn = pdTRUE;
		}
		else
		{
			xReturn = pdFALSE;
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );

	return xReturn;
}
/*-----------------------------------------------------------*/

static size_t prvWriteBytesToBuffer( StreamBuffer_t * const pxStreamBuffer, const uint8_t *pucData, size_t xCount )
{
size_t xNextHead, xFirstLength;

	configASSERT( xCount > ( size_t ) 0 );

	xNextHead = pxStreamBuffer->xHead;

	/* 计算第一次写入可以添加的字节数——这可能小于需要添加的总字节数，
	如果缓冲区将回绕到开头。 */
	xFirstLength = configMIN( pxStreamBuffer->xLength - xNextHead, xCount );

	/* 尽可能多地写入第一次写入可以写的字节。 */
	configASSERT( ( xNextHead + xFirstLength ) <= pxStreamBuffer->xLength );
	( void ) memcpy( ( void* ) ( &( pxStreamBuffer->pucBuffer[ xNextHead ] ) ), ( const void * ) pucData, xFirstLength ); /*lint !e9087 memcpy() 需要 void *。 */

	/* 如果写入的字节数少于第一次写入可以写的数量…… */
	if( xCount > xFirstLength )
	{
		/* ……则将剩余字节写入缓冲区的开头。 */
		configASSERT( ( xCount - xFirstLength ) <= pxStreamBuffer->xLength );
		( void ) memcpy( ( void * ) pxStreamBuffer->pucBuffer, ( const void * ) &( pucData[ xFirstLength ] ), xCount - xFirstLength ); /*lint !e9087 memcpy() 需要 void *。 */
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	xNextHead += xCount;
	if( xNextHead >= pxStreamBuffer->xLength )
	{
		xNextHead -= pxStreamBuffer->xLength;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	pxStreamBuffer->xHead = xNextHead;

	return xCount;
}
/*-----------------------------------------------------------*/

static size_t prvReadBytesFromBuffer( StreamBuffer_t *pxStreamBuffer, uint8_t *pucData, size_t xMaxCount, size_t xBytesAvailable )
{
size_t xCount, xFirstLength, xNextTail;

	/* 使用所需字节数和可用字节数中的较小值。 */
	xCount = configMIN( xBytesAvailable, xMaxCount );

	if( xCount > ( size_t ) 0 )
	{
		xNextTail = pxStreamBuffer->xTail;

		/* 计算可以读取的字节数——如果数据回绕到缓冲区的开头，
		则可能小于所需数量。 */
		xFirstLength = configMIN( pxStreamBuffer->xLength - xNextTail, xCount );

		/* 获取第一次读取可能获得的字节数。
		断言检查读取和写入的边界。 */
		configASSERT( xFirstLength <= xMaxCount );
		configASSERT( ( xNextTail + xFirstLength ) <= pxStreamBuffer->xLength );
		( void ) memcpy( ( void * ) pucData, ( const void * ) &( pxStreamBuffer->pucBuffer[ xNextTail ] ), xFirstLength ); /*lint !e9087 memcpy() 需要 void *。 */

		/* 如果所需的总字节数大于第一次读取可以读取的数量…… */
		if( xCount > xFirstLength )
		{
			/*……则从缓冲区的开头读取剩余字节。 */
			configASSERT( xCount <= xMaxCount );
			( void ) memcpy( ( void * ) &( pucData[ xFirstLength ] ), ( void * ) ( pxStreamBuffer->pucBuffer ), xCount - xFirstLength ); /*lint !e9087 memcpy() 需要 void *。 */
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* 移动 tail 指针以有效地从缓冲区中移除已读取的数据。 */
		xNextTail += xCount;

		if( xNextTail >= pxStreamBuffer->xLength )
		{
			xNextTail -= pxStreamBuffer->xLength;
		}

		pxStreamBuffer->xTail = xNextTail;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	return xCount;
}
/*-----------------------------------------------------------*/

static size_t prvBytesInBuffer( const StreamBuffer_t * const pxStreamBuffer )
{
/* 返回 xTail 和 xHead 之间的距离。 */
size_t xCount;

	xCount = pxStreamBuffer->xLength + pxStreamBuffer->xHead;
	xCount -= pxStreamBuffer->xTail;
	if ( xCount >= pxStreamBuffer->xLength )
	{
		xCount -= pxStreamBuffer->xLength;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	return xCount;
}
/*-----------------------------------------------------------*/

static void prvInitialiseNewStreamBuffer( StreamBuffer_t * const pxStreamBuffer,
										  uint8_t * const pucBuffer,
										  size_t xBufferSizeBytes,
										  size_t xTriggerLevelBytes,
										  uint8_t ucFlags )
{
	/* 这里的 assert 故意写入整个缓冲区，以确保可以向其写入而不会生成异常，
	并将缓冲区设置为已知值以帮助开发/调试。 */
	#if( configASSERT_DEFINED == 1 )
	{
		/* 写入的值只需要在查看内存时可识别即可。
		不要使用 0xA5，因为那是栈填充值，可能会导致混淆
		实际观察到的是什么。 */
		const BaseType_t xWriteValue = 0x55;
		configASSERT( memset( pucBuffer, ( int ) xWriteValue, xBufferSizeBytes ) == pucBuffer );
	} /*lint !e529 !e438 xWriteValue 仅在定义了 configASSERT() 时使用。 */
	#endif

	( void ) memset( ( void * ) pxStreamBuffer, 0x00, sizeof( StreamBuffer_t ) ); /*lint !e9087 memset() 需要 void *。 */
	pxStreamBuffer->pucBuffer = pucBuffer;
	pxStreamBuffer->xLength = xBufferSizeBytes;
	pxStreamBuffer->xTriggerLevelBytes = xTriggerLevelBytes;
	pxStreamBuffer->ucFlags = ucFlags;
}

#if ( configUSE_TRACE_FACILITY == 1 )

	UBaseType_t uxStreamBufferGetStreamBufferNumber( StreamBufferHandle_t xStreamBuffer )
	{
		return xStreamBuffer->uxStreamBufferNumber;
	}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	void vStreamBufferSetStreamBufferNumber( StreamBufferHandle_t xStreamBuffer, UBaseType_t uxStreamBufferNumber )
	{
		xStreamBuffer->uxStreamBufferNumber = uxStreamBufferNumber;
	}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

	uint8_t ucStreamBufferGetStreamBufferType( StreamBufferHandle_t xStreamBuffer )
	{
		return ( xStreamBuffer->ucFlags & sbFLAGS_IS_MESSAGE_BUFFER );
	}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/
