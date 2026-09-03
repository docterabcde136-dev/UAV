/* --------------------------------------------------------------------------
 * 版权所有 (c) 2013-2020 Arm Limited。保留所有权利。
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 根据 Apache 许可证 2.0 版（"许可证"）授权；
 * 除非遵守许可证，否则您不得使用此文件。
 * 您可以在以下网址获取许可证副本：
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * 除非适用法律要求或书面同意，根据许可证分发的软件
 * 按"原样"分发，不提供任何明示或暗示的担保或条件。
 * 请参阅许可证了解管理权限和限制的具体语言。
 *
 *      名称:    cmsis_os2.c
 *      用途:    FreeRTOS 的 CMSIS RTOS2 包装器
 *
 *---------------------------------------------------------------------------*/

#include <string.h>

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "cmsis_compiler.h"             // 编译器无关定义

#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core

#include "freertos_mpool.h"             // osMemoryPool 定义
#include "freertos_os2.h"               // 配置检查和设置

/*---------------------------------------------------------------------------*/
#ifndef __ARM_ARCH_6M__
  #define __ARM_ARCH_6M__         0
#endif
#ifndef __ARM_ARCH_7M__
  #define __ARM_ARCH_7M__         0
#endif
#ifndef __ARM_ARCH_7EM__
  #define __ARM_ARCH_7EM__        0
#endif
#ifndef __ARM_ARCH_8M_MAIN__
  #define __ARM_ARCH_8M_MAIN__    0
#endif
#ifndef __ARM_ARCH_7A__
  #define __ARM_ARCH_7A__         0
#endif

#if   ((__ARM_ARCH_7M__      == 1U) || \
       (__ARM_ARCH_7EM__     == 1U) || \
       (__ARM_ARCH_8M_MAIN__ == 1U))
#define IS_IRQ_MASKED()           ((__get_PRIMASK() != 0U) || (__get_BASEPRI() != 0U))
#elif  (__ARM_ARCH_6M__      == 1U)
#define IS_IRQ_MASKED()           (__get_PRIMASK() != 0U)
#elif (__ARM_ARCH_7A__       == 1U)
/* CPSR 掩码位 */
#define CPSR_MASKBIT_I            0x80U

#define IS_IRQ_MASKED()           ((__get_CPSR() & CPSR_MASKBIT_I) != 0U)
#else
#define IS_IRQ_MASKED()           (__get_PRIMASK() != 0U)
#endif

#if    (__ARM_ARCH_7A__      == 1U)
/* CPSR 模式位掩码 */
#define CPSR_MODE_USER            0x10U
#define CPSR_MODE_SYSTEM          0x1FU

#define IS_IRQ_MODE()             ((__get_mode() != CPSR_MODE_USER) && (__get_mode() != CPSR_MODE_SYSTEM))
#else
#define IS_IRQ_MODE()             (__get_IPSR() != 0U)
#endif

#define IS_IRQ()                  IS_IRQ_MODE()

#define SVCall_IRQ_NBR            (IRQn_Type) -5	/* 添加 SVCall_IRQ_NBR 是因为 SV_Call 处理函数名称在 CM0 和所有其他 CMx 上不同 */

/* 限制 */
#define MAX_BITS_TASK_NOTIFY      31U
#define MAX_BITS_EVENT_GROUPS     24U

#define THREAD_FLAGS_INVALID_BITS (~((1UL << MAX_BITS_TASK_NOTIFY)  - 1U))
#define EVENT_FLAGS_INVALID_BITS  (~((1UL << MAX_BITS_EVENT_GROUPS) - 1U))

/* 内核版本和标识字符串定义（主版本.次版本.修订版: mmnnnrrrr 十进制） */
#define KERNEL_VERSION            (((uint32_t)tskKERNEL_VERSION_MAJOR * 10000000UL) | \
                                   ((uint32_t)tskKERNEL_VERSION_MINOR *    10000UL) | \
                                   ((uint32_t)tskKERNEL_VERSION_BUILD *        1UL))

#define KERNEL_ID                 ("FreeRTOS " tskKERNEL_VERSION_NUMBER)

/* 定时器回调信息结构体定义 */
typedef struct {
  osTimerFunc_t func;
  void         *arg;
} TimerCallback_t;

/* 内核初始化状态 */
static osKernelState_t KernelState = osKernelInactive;

/*
  heap_5 变体使用的堆区域定义

  如果堆区域已在应用程序中定义且调用了 vPortDefineHeapRegions，
  则在 FreeRTOSConfig.h 中将 configAPPLICATION_ALLOCATED_HEAP 定义为非零值。

  否则 osKernelInitialize 将使用 configHEAP_5_REGIONS 定义作为参数
  调用 vPortDefineHeapRegions。可以通过在全局或 FreeRTOSConfig.h 中
  定义 configHEAP_5_REGIONS 来覆盖它。
*/
#if defined(USE_FreeRTOS_HEAP_5)
#if (configAPPLICATION_ALLOCATED_HEAP == 0)
  /*
    应用程序未定义 FreeRTOS 堆。
    默认提供大小为 configTOTAL_HEAP_SIZE（在 FreeRTOSConfig.h 中定义）的单个区域。
    定义 configHEAP_5_REGIONS 以提供自定义的 HeapRegion_t 数组。
  */
  #define HEAP_5_REGION_SETUP   1

  #ifndef configHEAP_5_REGIONS
    #define configHEAP_5_REGIONS xHeapRegions

    static uint8_t ucHeap[configTOTAL_HEAP_SIZE];

    static HeapRegion_t xHeapRegions[] = {
      { ucHeap, configTOTAL_HEAP_SIZE },
      { NULL,   0                     }
    };
  #else
    /* 提供全局定义以覆盖默认堆数组 */
    extern HeapRegion_t configHEAP_5_REGIONS[];
  #endif
#else
  /*
    应用程序已经定义了用于 FreeRTOS 堆的数组，
    并调用了 vPortDefineHeapRegions 来初始化堆。
  */
  #define HEAP_5_REGION_SETUP   0
#endif /* configAPPLICATION_ALLOCATED_HEAP */
#endif /* USE_FreeRTOS_HEAP_5 */

#if defined(SysTick)
#undef SysTick_Handler

/* CMSIS SysTick 中断处理函数原型 */
extern void SysTick_Handler     (void);
/* FreeRTOS tick 定时器中断处理函数原型 */
extern void xPortSysTickHandler (void);

/*
  同时清除溢出标志的 SysTick 处理函数实现。
*/
#if (USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION == 0)
void SysTick_Handler (void) {
  /* 清除溢出标志 */
  SysTick->CTRL;

  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    /* 调用 tick 处理函数 */
    xPortSysTickHandler();
  }
}
#endif
#endif /* SysTick */

/*
  将 SVC 设置为复位值。
*/
__STATIC_INLINE void SVC_Setup (void) {
#if (__ARM_ARCH_7A__ == 0U)
  /* 服务调用中断可能在内核启动之前被配置，           */
  /* 当其优先级低于或等于 BASEPRI 时，svc 指令        */
  /* 会导致 Hard Fault。                              */
  NVIC_SetPriority (SVCall_IRQ_NBR, 0U);
#endif
}

/*
  用于从中断中检索信号量计数的函数宏
*/
#ifndef uxSemaphoreGetCountFromISR
#define uxSemaphoreGetCountFromISR( xSemaphore ) uxQueueMessagesWaitingFromISR( ( QueueHandle_t ) ( xSemaphore ) )
#endif

/* 获取 OS Tick 计数值 */
static uint32_t OS_Tick_GetCount (void);
/* 获取 OS Tick 溢出状态 */
static uint32_t OS_Tick_GetOverflow (void);
/* 获取 OS Tick 间隔 */
static uint32_t OS_Tick_GetInterval (void);
/*---------------------------------------------------------------------------*/

osStatus_t osKernelInitialize (void) {
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else {
    if (KernelState == osKernelInactive) {
      #if defined(USE_TRACE_EVENT_RECORDER)
        EvrFreeRTOSSetup(0U);
      #endif
      #if defined(USE_FreeRTOS_HEAP_5) && (HEAP_5_REGION_SETUP == 1)
        vPortDefineHeapRegions (configHEAP_5_REGIONS);
      #endif
      KernelState = osKernelReady;
      stat = osOK;
    } else {
      stat = osError;
    }
  }

  return (stat);
}

osStatus_t osKernelGetInfo (osVersion_t *version, char *id_buf, uint32_t id_size) {

  if (version != NULL) {
    /* 版本编码为主版本.次版本.修订版: mmnnnrrrr 十进制 */
    version->api    = KERNEL_VERSION;
    version->kernel = KERNEL_VERSION;
  }

  if ((id_buf != NULL) && (id_size != 0U)) {
    if (id_size > sizeof(KERNEL_ID)) {
      id_size = sizeof(KERNEL_ID);
    }
    memcpy(id_buf, KERNEL_ID, id_size);
  }

  return (osOK);
}

osKernelState_t osKernelGetState (void) {
  osKernelState_t state;

  switch (xTaskGetSchedulerState()) {
    case taskSCHEDULER_RUNNING:
      state = osKernelRunning;
      break;

    case taskSCHEDULER_SUSPENDED:
      state = osKernelLocked;
      break;

    case taskSCHEDULER_NOT_STARTED:
    default:
      if (KernelState == osKernelReady) {
        state = osKernelReady;
      } else {
        state = osKernelInactive;
      }
      break;
  }

  return (state);
}

osStatus_t osKernelStart (void) {
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else {
    if (KernelState == osKernelReady) {
      /* 确保 SVC 优先级处于复位值 */
      SVC_Setup();
      /* 更改状态以启用 IRQ 屏蔽检查 */
      KernelState = osKernelRunning;
      /* 启动内核调度器 */
      vTaskStartScheduler();
      stat = osOK;
    } else {
      stat = osError;
    }
  }

  return (stat);
}

int32_t osKernelLock (void) {
  int32_t lock;

  if (IS_IRQ()) {
    lock = (int32_t)osErrorISR;
  }
  else {
    switch (xTaskGetSchedulerState()) {
      case taskSCHEDULER_SUSPENDED:
        lock = 1;
        break;

      case taskSCHEDULER_RUNNING:
        vTaskSuspendAll();
        lock = 0;
        break;

      case taskSCHEDULER_NOT_STARTED:
      default:
        lock = (int32_t)osError;
        break;
    }
  }

  return (lock);
}

int32_t osKernelUnlock (void) {
  int32_t lock;

  if (IS_IRQ()) {
    lock = (int32_t)osErrorISR;
  }
  else {
    switch (xTaskGetSchedulerState()) {
      case taskSCHEDULER_SUSPENDED:
        lock = 1;

        if (xTaskResumeAll() != pdTRUE) {
          if (xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) {
            lock = (int32_t)osError;
          }
        }
        break;

      case taskSCHEDULER_RUNNING:
        lock = 0;
        break;

      case taskSCHEDULER_NOT_STARTED:
      default:
        lock = (int32_t)osError;
        break;
    }
  }

  return (lock);
}

int32_t osKernelRestoreLock (int32_t lock) {

  if (IS_IRQ()) {
    lock = (int32_t)osErrorISR;
  }
  else {
    switch (xTaskGetSchedulerState()) {
      case taskSCHEDULER_SUSPENDED:
      case taskSCHEDULER_RUNNING:
        if (lock == 1) {
          vTaskSuspendAll();
        }
        else {
          if (lock != 0) {
            lock = (int32_t)osError;
          }
          else {
            if (xTaskResumeAll() != pdTRUE) {
              if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
                lock = (int32_t)osError;
              }
            }
          }
        }
        break;

      case taskSCHEDULER_NOT_STARTED:
      default:
        lock = (int32_t)osError;
        break;
    }
  }

  return (lock);
}

uint32_t osKernelGetTickCount (void) {
  TickType_t ticks;

  if (IS_IRQ()) {
    ticks = xTaskGetTickCountFromISR();
  } else {
    ticks = xTaskGetTickCount();
  }

  return (ticks);
}

uint32_t osKernelGetTickFreq (void) {
  return (configTICK_RATE_HZ);
}

/* 获取 OS Tick 计数值 */
static uint32_t OS_Tick_GetCount (void) {
  uint32_t load = SysTick->LOAD;
  return  (load - SysTick->VAL);
}

/* 获取 OS Tick 溢出状态 */
static uint32_t OS_Tick_GetOverflow (void) {
  return ((SysTick->CTRL >> 16) & 1U);
}

/* 获取 OS Tick 间隔 */
static uint32_t OS_Tick_GetInterval (void) {
  return (SysTick->LOAD + 1U);
}

uint32_t osKernelGetSysTimerCount (void) {
  uint32_t irqmask = IS_IRQ_MASKED();
  TickType_t ticks;
  uint32_t val;

  __disable_irq();

  ticks = xTaskGetTickCount();
  val   = OS_Tick_GetCount();

  if (OS_Tick_GetOverflow() != 0U) {
    val = OS_Tick_GetCount();
    ticks++;
  }
  val += ticks * OS_Tick_GetInterval();

  if (irqmask == 0U) {
    __enable_irq();
  }

  return (val);
}

uint32_t osKernelGetSysTimerFreq (void) {
  return (configCPU_CLOCK_HZ);
}

/*---------------------------------------------------------------------------*/

osThreadId_t osThreadNew (osThreadFunc_t func, void *argument, const osThreadAttr_t *attr) {
  const char *name;
  uint32_t stack;
  TaskHandle_t hTask;
  UBaseType_t prio;
  int32_t mem;

  hTask = NULL;

  if (!IS_IRQ() && (func != NULL)) {
    stack = configMINIMAL_STACK_SIZE;
    prio  = (UBaseType_t)osPriorityNormal;

    name = NULL;
    mem  = -1;

    if (attr != NULL) {
      if (attr->name != NULL) {
        name = attr->name;
      }
      if (attr->priority != osPriorityNone) {
        prio = (UBaseType_t)attr->priority;
      }

      if ((prio < osPriorityIdle) || (prio > osPriorityISR) || ((attr->attr_bits & osThreadJoinable) == osThreadJoinable)) {
        return (NULL);
      }

      if (attr->stack_size > 0U) {
        /* 在 FreeRTOS 中栈不是以字节为单位，而是以 sizeof(StackType_t) 为单位，在 ARM 移植中为 4。 */
        /* 因此栈大小应为 4 字节对齐，以避免除法引起的副作用                       */
        stack = attr->stack_size / sizeof(StackType_t);
      }

      if ((attr->cb_mem    != NULL) && (attr->cb_size    >= sizeof(StaticTask_t)) &&
          (attr->stack_mem != NULL) && (attr->stack_size >  0U)) {
        mem = 1;
      }
      else {
        if ((attr->cb_mem == NULL) && (attr->cb_size == 0U) && (attr->stack_mem == NULL)) {
          mem = 0;
        }
      }
    }
    else {
      mem = 0;
    }

    if (mem == 1) {
      #if (configSUPPORT_STATIC_ALLOCATION == 1)
        hTask = xTaskCreateStatic ((TaskFunction_t)func, name, stack, argument, prio, (StackType_t  *)attr->stack_mem,
                                                                                      (StaticTask_t *)attr->cb_mem);
      #endif
    }
    else {
      if (mem == 0) {
        #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
          if (xTaskCreate ((TaskFunction_t)func, name, (uint16_t)stack, argument, prio, &hTask) != pdPASS) {
            hTask = NULL;
          }
        #endif
      }
    }
  }

  return ((osThreadId_t)hTask);
}

const char *osThreadGetName (osThreadId_t thread_id) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  const char *name;

  if (IS_IRQ() || (hTask == NULL)) {
    name = NULL;
  } else {
    name = pcTaskGetName (hTask);
  }

  return (name);
}

osThreadId_t osThreadGetId (void) {
  osThreadId_t id;

  id = (osThreadId_t)xTaskGetCurrentTaskHandle();

  return (id);
}

osThreadState_t osThreadGetState (osThreadId_t thread_id) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  osThreadState_t state;

  if (IS_IRQ() || (hTask == NULL)) {
    state = osThreadError;
  }
  else {
    switch (eTaskGetState (hTask)) {
      case eRunning:   state = osThreadRunning;    break;
      case eReady:     state = osThreadReady;      break;
      case eBlocked:
      case eSuspended: state = osThreadBlocked;    break;
      case eDeleted:   state = osThreadTerminated; break;
      case eInvalid:
      default:         state = osThreadError;      break;
    }
  }

  return (state);
}

uint32_t osThreadGetStackSpace (osThreadId_t thread_id) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  uint32_t sz;

  if (IS_IRQ() || (hTask == NULL)) {
    sz = 0U;
  } else {
    sz = (uint32_t)(uxTaskGetStackHighWaterMark(hTask) * sizeof(StackType_t));
  }

  return (sz);
}

osStatus_t osThreadSetPriority (osThreadId_t thread_id, osPriority_t priority) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if ((hTask == NULL) || (priority < osPriorityIdle) || (priority > osPriorityISR)) {
    stat = osErrorParameter;
  }
  else {
    stat = osOK;
    vTaskPrioritySet (hTask, (UBaseType_t)priority);
  }

  return (stat);
}

osPriority_t osThreadGetPriority (osThreadId_t thread_id) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  osPriority_t prio;

  if (IS_IRQ() || (hTask == NULL)) {
    prio = osPriorityError;
  } else {
    prio = (osPriority_t)((int32_t)uxTaskPriorityGet (hTask));
  }

  return (prio);
}

osStatus_t osThreadYield (void) {
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  } else {
    stat = osOK;
    taskYIELD();
  }

  return (stat);
}

#if (configUSE_OS2_THREAD_SUSPEND_RESUME == 1)
osStatus_t osThreadSuspend (osThreadId_t thread_id) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hTask == NULL) {
    stat = osErrorParameter;
  }
  else {
    stat = osOK;
    vTaskSuspend (hTask);
  }

  return (stat);
}

osStatus_t osThreadResume (osThreadId_t thread_id) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hTask == NULL) {
    stat = osErrorParameter;
  }
  else {
    stat = osOK;
    vTaskResume (hTask);
  }

  return (stat);
}
#endif /* (configUSE_OS2_THREAD_SUSPEND_RESUME == 1) */

__NO_RETURN void osThreadExit (void) {
#ifndef USE_FreeRTOS_HEAP_1
  vTaskDelete (NULL);
#endif
  for (;;);
}

osStatus_t osThreadTerminate (osThreadId_t thread_id) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  osStatus_t stat;
#ifndef USE_FreeRTOS_HEAP_1
  eTaskState tstate;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hTask == NULL) {
    stat = osErrorParameter;
  }
  else {
    tstate = eTaskGetState (hTask);

    if (tstate != eDeleted) {
      stat = osOK;
      vTaskDelete (hTask);
    } else {
      stat = osErrorResource;
    }
  }
#else
  stat = osError;
#endif

  return (stat);
}

uint32_t osThreadGetCount (void) {
  uint32_t count;

  if (IS_IRQ()) {
    count = 0U;
  } else {
    count = uxTaskGetNumberOfTasks();
  }

  return (count);
}

#if (configUSE_OS2_THREAD_ENUMERATE == 1)
uint32_t osThreadEnumerate (osThreadId_t *thread_array, uint32_t array_items) {
  uint32_t i, count;
  TaskStatus_t *task;

  if (IS_IRQ() || (thread_array == NULL) || (array_items == 0U)) {
    count = 0U;
  } else {
    vTaskSuspendAll();

    count = uxTaskGetNumberOfTasks();
    task  = pvPortMalloc (count * sizeof(TaskStatus_t));

    if (task != NULL) {
      count = uxTaskGetSystemState (task, count, NULL);

      for (i = 0U; (i < count) && (i < array_items); i++) {
        thread_array[i] = (osThreadId_t)task[i].xHandle;
      }
      count = i;
    }
    (void)xTaskResumeAll();

    vPortFree (task);
  }

  return (count);
}
#endif /* (configUSE_OS2_THREAD_ENUMERATE == 1) */

#if (configUSE_OS2_THREAD_FLAGS == 1)
uint32_t osThreadFlagsSet (osThreadId_t thread_id, uint32_t flags) {
  TaskHandle_t hTask = (TaskHandle_t)thread_id;
  uint32_t rflags;
  BaseType_t yield;

  if ((hTask == NULL) || ((flags & THREAD_FLAGS_INVALID_BITS) != 0U)) {
    rflags = (uint32_t)osErrorParameter;
  }
  else {
    rflags = (uint32_t)osError;

    if (IS_IRQ()) {
      yield = pdFALSE;

      (void)xTaskNotifyFromISR (hTask, flags, eSetBits, &yield);
      (void)xTaskNotifyAndQueryFromISR (hTask, 0, eNoAction, &rflags, NULL);

      portYIELD_FROM_ISR (yield);
    }
    else {
      (void)xTaskNotify (hTask, flags, eSetBits);
      (void)xTaskNotifyAndQuery (hTask, 0, eNoAction, &rflags);
    }
  }
  /* 返回设置后的标志 */
  return (rflags);
}

uint32_t osThreadFlagsClear (uint32_t flags) {
  TaskHandle_t hTask;
  uint32_t rflags, cflags;

  if (IS_IRQ()) {
    rflags = (uint32_t)osErrorISR;
  }
  else if ((flags & THREAD_FLAGS_INVALID_BITS) != 0U) {
    rflags = (uint32_t)osErrorParameter;
  }
  else {
    hTask = xTaskGetCurrentTaskHandle();

    if (xTaskNotifyAndQuery (hTask, 0, eNoAction, &cflags) == pdPASS) {
      rflags = cflags;
      cflags &= ~flags;

      if (xTaskNotify (hTask, cflags, eSetValueWithOverwrite) != pdPASS) {
        rflags = (uint32_t)osError;
      }
    }
    else {
      rflags = (uint32_t)osError;
    }
  }

  /* 返回清除前的标志 */
  return (rflags);
}

uint32_t osThreadFlagsGet (void) {
  TaskHandle_t hTask;
  uint32_t rflags;

  if (IS_IRQ()) {
    rflags = (uint32_t)osErrorISR;
  }
  else {
    hTask = xTaskGetCurrentTaskHandle();

    if (xTaskNotifyAndQuery (hTask, 0, eNoAction, &rflags) != pdPASS) {
      rflags = (uint32_t)osError;
    }
  }

  return (rflags);
}

uint32_t osThreadFlagsWait (uint32_t flags, uint32_t options, uint32_t timeout) {
  uint32_t rflags, nval;
  uint32_t clear;
  TickType_t t0, td, tout;
  BaseType_t rval;

  if (IS_IRQ()) {
    rflags = (uint32_t)osErrorISR;
  }
  else if ((flags & THREAD_FLAGS_INVALID_BITS) != 0U) {
    rflags = (uint32_t)osErrorParameter;
  }
  else {
    if ((options & osFlagsNoClear) == osFlagsNoClear) {
      clear = 0U;
    } else {
      clear = flags;
    }

    rflags = 0U;
    tout   = timeout;

    t0 = xTaskGetTickCount();
    do {
      rval = xTaskNotifyWait (0, clear, &nval, tout);

      if (rval == pdPASS) {
        rflags &= flags;
        rflags |= nval;

        if ((options & osFlagsWaitAll) == osFlagsWaitAll) {
          if ((flags & rflags) == flags) {
            break;
          } else {
            if (timeout == 0U) {
              rflags = (uint32_t)osErrorResource;
              break;
            }
          }
        }
        else {
          if ((flags & rflags) != 0) {
            break;
          } else {
            if (timeout == 0U) {
              rflags = (uint32_t)osErrorResource;
              break;
            }
          }
        }

        /* 更新超时 */
        td = xTaskGetTickCount() - t0;

        if (td > tout) {
          tout  = 0;
        } else {
          tout -= td;
        }
      }
      else {
        if (timeout == 0) {
          rflags = (uint32_t)osErrorResource;
        } else {
          rflags = (uint32_t)osErrorTimeout;
        }
      }
    }
    while (rval != pdFAIL);
  }

  /* 返回清除前的标志 */
  return (rflags);
}
#endif /* (configUSE_OS2_THREAD_FLAGS == 1) */

osStatus_t osDelay (uint32_t ticks) {
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else {
    stat = osOK;

    if (ticks != 0U) {
      vTaskDelay(ticks);
    }
  }

  return (stat);
}

osStatus_t osDelayUntil (uint32_t ticks) {
  TickType_t tcnt, delay;
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else {
    stat = osOK;
    tcnt = xTaskGetTickCount();

    /* 确定剩余的延迟 tick 数 */
    delay = (TickType_t)ticks - tcnt;

    /* 检查目标 tick 是否尚未过期 */
    if((delay != 0U) && (0 == (delay >> (8 * sizeof(TickType_t) - 1)))) {
      vTaskDelayUntil (&tcnt, delay);
    }
    else
    {
      /* 无延迟或已经过期 */
      stat = osErrorParameter;
    }
  }

  return (stat);
}

/*---------------------------------------------------------------------------*/
#if (configUSE_OS2_TIMER == 1)

static void TimerCallback (TimerHandle_t hTimer) {
  TimerCallback_t *callb;

  callb = (TimerCallback_t *)pvTimerGetTimerID (hTimer);

  if (callb != NULL) {
    callb->func (callb->arg);
  }
}

osTimerId_t osTimerNew (osTimerFunc_t func, osTimerType_t type, void *argument, const osTimerAttr_t *attr) {
  const char *name;
  TimerHandle_t hTimer;
  TimerCallback_t *callb;
  UBaseType_t reload;
  int32_t mem;

  hTimer = NULL;

  if (!IS_IRQ() && (func != NULL)) {
    /* 分配内存以存储回调函数和参数 */
    callb = pvPortMalloc (sizeof(TimerCallback_t));

    if (callb != NULL) {
      callb->func = func;
      callb->arg  = argument;

      if (type == osTimerOnce) {
        reload = pdFALSE;
      } else {
        reload = pdTRUE;
      }

      mem  = -1;
      name = NULL;

      if (attr != NULL) {
        if (attr->name != NULL) {
          name = attr->name;
        }

        if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticTimer_t))) {
          mem = 1;
        }
        else {
          if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
            mem = 0;
          }
        }
      }
      else {
        mem = 0;
      }

      if (mem == 1) {
        #if (configSUPPORT_STATIC_ALLOCATION == 1)
          hTimer = xTimerCreateStatic (name, 1, reload, callb, TimerCallback, (StaticTimer_t *)attr->cb_mem);
        #endif
      }
      else {
        if (mem == 0) {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            hTimer = xTimerCreate (name, 1, reload, callb, TimerCallback);
          #endif
        }
      }

      if ((hTimer == NULL) && (callb != NULL)) {
        vPortFree (callb);
      }
    }
  }

  return ((osTimerId_t)hTimer);
}

const char *osTimerGetName (osTimerId_t timer_id) {
  TimerHandle_t hTimer = (TimerHandle_t)timer_id;
  const char *p;

  if (IS_IRQ() || (hTimer == NULL)) {
    p = NULL;
  } else {
    p = pcTimerGetName (hTimer);
  }

  return (p);
}

osStatus_t osTimerStart (osTimerId_t timer_id, uint32_t ticks) {
  TimerHandle_t hTimer = (TimerHandle_t)timer_id;
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hTimer == NULL) {
    stat = osErrorParameter;
  }
  else {
    if (xTimerChangePeriod (hTimer, ticks, 0) == pdPASS) {
      stat = osOK;
    } else {
      stat = osErrorResource;
    }
  }

  return (stat);
}

osStatus_t osTimerStop (osTimerId_t timer_id) {
  TimerHandle_t hTimer = (TimerHandle_t)timer_id;
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hTimer == NULL) {
    stat = osErrorParameter;
  }
  else {
    if (xTimerIsTimerActive (hTimer) == pdFALSE) {
      stat = osErrorResource;
    }
    else {
      if (xTimerStop (hTimer, 0) == pdPASS) {
        stat = osOK;
      } else {
        stat = osError;
      }
    }
  }

  return (stat);
}

uint32_t osTimerIsRunning (osTimerId_t timer_id) {
  TimerHandle_t hTimer = (TimerHandle_t)timer_id;
  uint32_t running;

  if (IS_IRQ() || (hTimer == NULL)) {
    running = 0U;
  } else {
    running = (uint32_t)xTimerIsTimerActive (hTimer);
  }

  return (running);
}

osStatus_t osTimerDelete (osTimerId_t timer_id) {
  TimerHandle_t hTimer = (TimerHandle_t)timer_id;
  osStatus_t stat;
#ifndef USE_FreeRTOS_HEAP_1
  TimerCallback_t *callb;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hTimer == NULL) {
    stat = osErrorParameter;
  }
  else {
    callb = (TimerCallback_t *)pvTimerGetTimerID (hTimer);

    if (xTimerDelete (hTimer, 0) == pdPASS) {
      vPortFree (callb);
      stat = osOK;
    } else {
      stat = osErrorResource;
    }
  }
#else
  stat = osError;
#endif

  return (stat);
}
#endif /* (configUSE_OS2_TIMER == 1) */

/*---------------------------------------------------------------------------*/

osEventFlagsId_t osEventFlagsNew (const osEventFlagsAttr_t *attr) {
  EventGroupHandle_t hEventGroup;
  int32_t mem;

  hEventGroup = NULL;

  if (!IS_IRQ()) {
    mem = -1;

    if (attr != NULL) {
      if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticEventGroup_t))) {
        mem = 1;
      }
      else {
        if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
          mem = 0;
        }
      }
    }
    else {
      mem = 0;
    }

    if (mem == 1) {
      #if (configSUPPORT_STATIC_ALLOCATION == 1)
      hEventGroup = xEventGroupCreateStatic (attr->cb_mem);
      #endif
    }
    else {
      if (mem == 0) {
        #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
          hEventGroup = xEventGroupCreate();
        #endif
      }
    }
  }

  return ((osEventFlagsId_t)hEventGroup);
}

uint32_t osEventFlagsSet (osEventFlagsId_t ef_id, uint32_t flags) {
  EventGroupHandle_t hEventGroup = (EventGroupHandle_t)ef_id;
  uint32_t rflags;
  BaseType_t yield;

  if ((hEventGroup == NULL) || ((flags & EVENT_FLAGS_INVALID_BITS) != 0U)) {
    rflags = (uint32_t)osErrorParameter;
  }
  else if (IS_IRQ()) {
  #if (configUSE_OS2_EVENTFLAGS_FROM_ISR == 0)
    (void)yield;
    /* 启用定时器和 xTimerPendFunctionCall 函数以支持从中断调用 osEventFlagsSet */
    rflags = (uint32_t)osErrorResource;
  #else
    yield = pdFALSE;

    if (xEventGroupSetBitsFromISR (hEventGroup, (EventBits_t)flags, &yield) == pdFAIL) {
      rflags = (uint32_t)osErrorResource;
    } else {
      rflags = flags;
      portYIELD_FROM_ISR (yield);
    }
  #endif
  }
  else {
    rflags = xEventGroupSetBits (hEventGroup, (EventBits_t)flags);
  }

  return (rflags);
}

uint32_t osEventFlagsClear (osEventFlagsId_t ef_id, uint32_t flags) {
  EventGroupHandle_t hEventGroup = (EventGroupHandle_t)ef_id;
  uint32_t rflags;

  if ((hEventGroup == NULL) || ((flags & EVENT_FLAGS_INVALID_BITS) != 0U)) {
    rflags = (uint32_t)osErrorParameter;
  }
  else if (IS_IRQ()) {
  #if (configUSE_OS2_EVENTFLAGS_FROM_ISR == 0)
    /* 启用定时器和 xTimerPendFunctionCall 函数以支持从中断调用 osEventFlagsSet */
    rflags = (uint32_t)osErrorResource;
  #else
    rflags = xEventGroupGetBitsFromISR (hEventGroup);

    if (xEventGroupClearBitsFromISR (hEventGroup, (EventBits_t)flags) == pdFAIL) {
      rflags = (uint32_t)osErrorResource;
    }
  #endif
  }
  else {
    rflags = xEventGroupClearBits (hEventGroup, (EventBits_t)flags);
  }

  return (rflags);
}

uint32_t osEventFlagsGet (osEventFlagsId_t ef_id) {
  EventGroupHandle_t hEventGroup = (EventGroupHandle_t)ef_id;
  uint32_t rflags;

  if (ef_id == NULL) {
    rflags = 0U;
  }
  else if (IS_IRQ()) {
    rflags = xEventGroupGetBitsFromISR (hEventGroup);
  }
  else {
    rflags = xEventGroupGetBits (hEventGroup);
  }

  return (rflags);
}

uint32_t osEventFlagsWait (osEventFlagsId_t ef_id, uint32_t flags, uint32_t options, uint32_t timeout) {
  EventGroupHandle_t hEventGroup = (EventGroupHandle_t)ef_id;
  BaseType_t wait_all;
  BaseType_t exit_clr;
  uint32_t rflags;

  if ((hEventGroup == NULL) || ((flags & EVENT_FLAGS_INVALID_BITS) != 0U)) {
    rflags = (uint32_t)osErrorParameter;
  }
  else if (IS_IRQ()) {
    rflags = (uint32_t)osErrorISR;
  }
  else {
    if (options & osFlagsWaitAll) {
      wait_all = pdTRUE;
    } else {
      wait_all = pdFAIL;
    }

    if (options & osFlagsNoClear) {
      exit_clr = pdFAIL;
    } else {
      exit_clr = pdTRUE;
    }

    rflags = xEventGroupWaitBits (hEventGroup, (EventBits_t)flags, exit_clr, wait_all, (TickType_t)timeout);

    if (options & osFlagsWaitAll) {
      if ((flags & rflags) != flags) {
        if (timeout > 0U) {
          rflags = (uint32_t)osErrorTimeout;
        } else {
          rflags = (uint32_t)osErrorResource;
        }
      }
    }
    else {
      if ((flags & rflags) == 0U) {
        if (timeout > 0U) {
          rflags = (uint32_t)osErrorTimeout;
        } else {
          rflags = (uint32_t)osErrorResource;
        }
      }
    }
  }

  return (rflags);
}

osStatus_t osEventFlagsDelete (osEventFlagsId_t ef_id) {
  EventGroupHandle_t hEventGroup = (EventGroupHandle_t)ef_id;
  osStatus_t stat;

#ifndef USE_FreeRTOS_HEAP_1
  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hEventGroup == NULL) {
    stat = osErrorParameter;
  }
  else {
    stat = osOK;
    vEventGroupDelete (hEventGroup);
  }
#else
  stat = osError;
#endif

  return (stat);
}

/*---------------------------------------------------------------------------*/
#if (configUSE_OS2_MUTEX == 1)

osMutexId_t osMutexNew (const osMutexAttr_t *attr) {
  SemaphoreHandle_t hMutex;
  uint32_t type;
  uint32_t rmtx;
  int32_t  mem;
  #if (configQUEUE_REGISTRY_SIZE > 0)
  const char *name;
  #endif

  hMutex = NULL;

  if (!IS_IRQ()) {
    if (attr != NULL) {
      type = attr->attr_bits;
    } else {
      type = 0U;
    }

    if ((type & osMutexRecursive) == osMutexRecursive) {
      rmtx = 1U;
    } else {
      rmtx = 0U;
    }

    if ((type & osMutexRobust) != osMutexRobust) {
      mem = -1;

      if (attr != NULL) {
        if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticSemaphore_t))) {
          mem = 1;
        }
        else {
          if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
            mem = 0;
          }
        }
      }
      else {
        mem = 0;
      }

      if (mem == 1) {
        #if (configSUPPORT_STATIC_ALLOCATION == 1)
          if (rmtx != 0U) {
            #if (configUSE_RECURSIVE_MUTEXES == 1)
            hMutex = xSemaphoreCreateRecursiveMutexStatic (attr->cb_mem);
            #endif
          }
          else {
            hMutex = xSemaphoreCreateMutexStatic (attr->cb_mem);
          }
        #endif
      }
      else {
        if (mem == 0) {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            if (rmtx != 0U) {
              #if (configUSE_RECURSIVE_MUTEXES == 1)
              hMutex = xSemaphoreCreateRecursiveMutex ();
              #endif
            } else {
              hMutex = xSemaphoreCreateMutex ();
            }
          #endif
        }
      }

      #if (configQUEUE_REGISTRY_SIZE > 0)
      if (hMutex != NULL) {
        if (attr != NULL) {
          name = attr->name;
        } else {
          name = NULL;
        }
        vQueueAddToRegistry (hMutex, name);
      }
      #endif

      if ((hMutex != NULL) && (rmtx != 0U)) {
        hMutex = (SemaphoreHandle_t)((uint32_t)hMutex | 1U);
      }
    }
  }

  return ((osMutexId_t)hMutex);
}

osStatus_t osMutexAcquire (osMutexId_t mutex_id, uint32_t timeout) {
  SemaphoreHandle_t hMutex;
  osStatus_t stat;
  uint32_t rmtx;

  hMutex = (SemaphoreHandle_t)((uint32_t)mutex_id & ~1U);

  rmtx = (uint32_t)mutex_id & 1U;

  stat = osOK;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hMutex == NULL) {
    stat = osErrorParameter;
  }
  else {
    if (rmtx != 0U) {
      #if (configUSE_RECURSIVE_MUTEXES == 1)
      if (xSemaphoreTakeRecursive (hMutex, timeout) != pdPASS) {
        if (timeout != 0U) {
          stat = osErrorTimeout;
        } else {
          stat = osErrorResource;
        }
      }
      #endif
    }
    else {
      if (xSemaphoreTake (hMutex, timeout) != pdPASS) {
        if (timeout != 0U) {
          stat = osErrorTimeout;
        } else {
          stat = osErrorResource;
        }
      }
    }
  }

  return (stat);
}

osStatus_t osMutexRelease (osMutexId_t mutex_id) {
  SemaphoreHandle_t hMutex;
  osStatus_t stat;
  uint32_t rmtx;

  hMutex = (SemaphoreHandle_t)((uint32_t)mutex_id & ~1U);

  rmtx = (uint32_t)mutex_id & 1U;

  stat = osOK;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hMutex == NULL) {
    stat = osErrorParameter;
  }
  else {
    if (rmtx != 0U) {
      #if (configUSE_RECURSIVE_MUTEXES == 1)
      if (xSemaphoreGiveRecursive (hMutex) != pdPASS) {
        stat = osErrorResource;
      }
      #endif
    }
    else {
      if (xSemaphoreGive (hMutex) != pdPASS) {
        stat = osErrorResource;
      }
    }
  }

  return (stat);
}

osThreadId_t osMutexGetOwner (osMutexId_t mutex_id) {
  SemaphoreHandle_t hMutex;
  osThreadId_t owner;

  hMutex = (SemaphoreHandle_t)((uint32_t)mutex_id & ~1U);

  if (IS_IRQ() || (hMutex == NULL)) {
    owner = NULL;
  } else {
    owner = (osThreadId_t)xSemaphoreGetMutexHolder (hMutex);
  }

  return (owner);
}

osStatus_t osMutexDelete (osMutexId_t mutex_id) {
  osStatus_t stat;
#ifndef USE_FreeRTOS_HEAP_1
  SemaphoreHandle_t hMutex;

  hMutex = (SemaphoreHandle_t)((uint32_t)mutex_id & ~1U);

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hMutex == NULL) {
    stat = osErrorParameter;
  }
  else {
    #if (configQUEUE_REGISTRY_SIZE > 0)
    vQueueUnregisterQueue (hMutex);
    #endif
    stat = osOK;
    vSemaphoreDelete (hMutex);
  }
#else
  stat = osError;
#endif

  return (stat);
}
#endif /* (configUSE_OS2_MUTEX == 1) */

/*---------------------------------------------------------------------------*/

osSemaphoreId_t osSemaphoreNew (uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr) {
  SemaphoreHandle_t hSemaphore;
  int32_t mem;
  #if (configQUEUE_REGISTRY_SIZE > 0)
  const char *name;
  #endif

  hSemaphore = NULL;

  if (!IS_IRQ() && (max_count > 0U) && (initial_count <= max_count)) {
    mem = -1;

    if (attr != NULL) {
      if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticSemaphore_t))) {
        mem = 1;
      }
      else {
        if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
          mem = 0;
        }
      }
    }
    else {
      mem = 0;
    }

    if (mem != -1) {
      if (max_count == 1U) {
        if (mem == 1) {
          #if (configSUPPORT_STATIC_ALLOCATION == 1)
            hSemaphore = xSemaphoreCreateBinaryStatic ((StaticSemaphore_t *)attr->cb_mem);
          #endif
        }
        else {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            hSemaphore = xSemaphoreCreateBinary();
          #endif
        }

        if ((hSemaphore != NULL) && (initial_count != 0U)) {
          if (xSemaphoreGive (hSemaphore) != pdPASS) {
            vSemaphoreDelete (hSemaphore);
            hSemaphore = NULL;
          }
        }
      }
      else {
        if (mem == 1) {
          #if (configSUPPORT_STATIC_ALLOCATION == 1)
            hSemaphore = xSemaphoreCreateCountingStatic (max_count, initial_count, (StaticSemaphore_t *)attr->cb_mem);
          #endif
        }
        else {
          #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
            hSemaphore = xSemaphoreCreateCounting (max_count, initial_count);
          #endif
        }
      }

      #if (configQUEUE_REGISTRY_SIZE > 0)
      if (hSemaphore != NULL) {
        if (attr != NULL) {
          name = attr->name;
        } else {
          name = NULL;
        }
        vQueueAddToRegistry (hSemaphore, name);
      }
      #endif
    }
  }

  return ((osSemaphoreId_t)hSemaphore);
}

osStatus_t osSemaphoreAcquire (osSemaphoreId_t semaphore_id, uint32_t timeout) {
  SemaphoreHandle_t hSemaphore = (SemaphoreHandle_t)semaphore_id;
  osStatus_t stat;
  BaseType_t yield;

  stat = osOK;

  if (hSemaphore == NULL) {
    stat = osErrorParameter;
  }
  else if (IS_IRQ()) {
    if (timeout != 0U) {
      stat = osErrorParameter;
    }
    else {
      yield = pdFALSE;

      if (xSemaphoreTakeFromISR (hSemaphore, &yield) != pdPASS) {
        stat = osErrorResource;
      } else {
        portYIELD_FROM_ISR (yield);
      }
    }
  }
  else {
    if (xSemaphoreTake (hSemaphore, (TickType_t)timeout) != pdPASS) {
      if (timeout != 0U) {
        stat = osErrorTimeout;
      } else {
        stat = osErrorResource;
      }
    }
  }

  return (stat);
}

osStatus_t osSemaphoreRelease (osSemaphoreId_t semaphore_id) {
  SemaphoreHandle_t hSemaphore = (SemaphoreHandle_t)semaphore_id;
  osStatus_t stat;
  BaseType_t yield;

  stat = osOK;

  if (hSemaphore == NULL) {
    stat = osErrorParameter;
  }
  else if (IS_IRQ()) {
    yield = pdFALSE;

    if (xSemaphoreGiveFromISR (hSemaphore, &yield) != pdTRUE) {
      stat = osErrorResource;
    } else {
      portYIELD_FROM_ISR (yield);
    }
  }
  else {
    if (xSemaphoreGive (hSemaphore) != pdPASS) {
      stat = osErrorResource;
    }
  }

  return (stat);
}

uint32_t osSemaphoreGetCount (osSemaphoreId_t semaphore_id) {
  SemaphoreHandle_t hSemaphore = (SemaphoreHandle_t)semaphore_id;
  uint32_t count;

  if (hSemaphore == NULL) {
    count = 0U;
  }
  else if (IS_IRQ()) {
    count = uxQueueMessagesWaitingFromISR (hSemaphore);
  } else {
    count = (uint32_t)uxSemaphoreGetCount (hSemaphore);
  }

  return (count);
}

osStatus_t osSemaphoreDelete (osSemaphoreId_t semaphore_id) {
  SemaphoreHandle_t hSemaphore = (SemaphoreHandle_t)semaphore_id;
  osStatus_t stat;

#ifndef USE_FreeRTOS_HEAP_1
  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hSemaphore == NULL) {
    stat = osErrorParameter;
  }
  else {
    #if (configQUEUE_REGISTRY_SIZE > 0)
    vQueueUnregisterQueue (hSemaphore);
    #endif

    stat = osOK;
    vSemaphoreDelete (hSemaphore);
  }
#else
  stat = osError;
#endif

  return (stat);
}

/*---------------------------------------------------------------------------*/

osMessageQueueId_t osMessageQueueNew (uint32_t msg_count, uint32_t msg_size, const osMessageQueueAttr_t *attr) {
  QueueHandle_t hQueue;
  int32_t mem;
  #if (configQUEUE_REGISTRY_SIZE > 0)
  const char *name;
  #endif

  hQueue = NULL;

  if (!IS_IRQ() && (msg_count > 0U) && (msg_size > 0U)) {
    mem = -1;

    if (attr != NULL) {
      if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(StaticQueue_t)) &&
          (attr->mq_mem != NULL) && (attr->mq_size >= (msg_count * msg_size))) {
        mem = 1;
      }
      else {
        if ((attr->cb_mem == NULL) && (attr->cb_size == 0U) &&
            (attr->mq_mem == NULL) && (attr->mq_size == 0U)) {
          mem = 0;
        }
      }
    }
    else {
      mem = 0;
    }

    if (mem == 1) {
      #if (configSUPPORT_STATIC_ALLOCATION == 1)
        hQueue = xQueueCreateStatic (msg_count, msg_size, attr->mq_mem, attr->cb_mem);
      #endif
    }
    else {
      if (mem == 0) {
        #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
          hQueue = xQueueCreate (msg_count, msg_size);
        #endif
      }
    }

    #if (configQUEUE_REGISTRY_SIZE > 0)
    if (hQueue != NULL) {
      if (attr != NULL) {
        name = attr->name;
      } else {
        name = NULL;
      }
      vQueueAddToRegistry (hQueue, name);
    }
    #endif

  }

  return ((osMessageQueueId_t)hQueue);
}

osStatus_t osMessageQueuePut (osMessageQueueId_t mq_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout) {
  QueueHandle_t hQueue = (QueueHandle_t)mq_id;
  osStatus_t stat;
  BaseType_t yield;

  (void)msg_prio; /* 消息优先级被忽略 */

  stat = osOK;

  if (IS_IRQ()) {
    if ((hQueue == NULL) || (msg_ptr == NULL) || (timeout != 0U)) {
      stat = osErrorParameter;
    }
    else {
      yield = pdFALSE;

      if (xQueueSendToBackFromISR (hQueue, msg_ptr, &yield) != pdTRUE) {
        stat = osErrorResource;
      } else {
        portYIELD_FROM_ISR (yield);
      }
    }
  }
  else {
    if ((hQueue == NULL) || (msg_ptr == NULL)) {
      stat = osErrorParameter;
    }
    else {
      if (xQueueSendToBack (hQueue, msg_ptr, (TickType_t)timeout) != pdPASS) {
        if (timeout != 0U) {
          stat = osErrorTimeout;
        } else {
          stat = osErrorResource;
        }
      }
    }
  }

  return (stat);
}

osStatus_t osMessageQueueGet (osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout) {
  QueueHandle_t hQueue = (QueueHandle_t)mq_id;
  osStatus_t stat;
  BaseType_t yield;

  (void)msg_prio; /* 消息优先级被忽略 */

  stat = osOK;

  if (IS_IRQ()) {
    if ((hQueue == NULL) || (msg_ptr == NULL) || (timeout != 0U)) {
      stat = osErrorParameter;
    }
    else {
      yield = pdFALSE;

      if (xQueueReceiveFromISR (hQueue, msg_ptr, &yield) != pdPASS) {
        stat = osErrorResource;
      } else {
        portYIELD_FROM_ISR (yield);
      }
    }
  }
  else {
    if ((hQueue == NULL) || (msg_ptr == NULL)) {
      stat = osErrorParameter;
    }
    else {
      if (xQueueReceive (hQueue, msg_ptr, (TickType_t)timeout) != pdPASS) {
        if (timeout != 0U) {
          stat = osErrorTimeout;
        } else {
          stat = osErrorResource;
        }
      }
    }
  }

  return (stat);
}

uint32_t osMessageQueueGetCapacity (osMessageQueueId_t mq_id) {
  StaticQueue_t *mq = (StaticQueue_t *)mq_id;
  uint32_t capacity;

  if (mq == NULL) {
    capacity = 0U;
  } else {
    /* capacity = pxQueue->uxLength */
    capacity = mq->uxDummy4[1];
  }

  return (capacity);
}

uint32_t osMessageQueueGetMsgSize (osMessageQueueId_t mq_id) {
  StaticQueue_t *mq = (StaticQueue_t *)mq_id;
  uint32_t size;

  if (mq == NULL) {
    size = 0U;
  } else {
    /* size = pxQueue->uxItemSize */
    size = mq->uxDummy4[2];
  }

  return (size);
}

uint32_t osMessageQueueGetCount (osMessageQueueId_t mq_id) {
  QueueHandle_t hQueue = (QueueHandle_t)mq_id;
  UBaseType_t count;

  if (hQueue == NULL) {
    count = 0U;
  }
  else if (IS_IRQ()) {
    count = uxQueueMessagesWaitingFromISR (hQueue);
  }
  else {
    count = uxQueueMessagesWaiting (hQueue);
  }

  return ((uint32_t)count);
}

uint32_t osMessageQueueGetSpace (osMessageQueueId_t mq_id) {
  StaticQueue_t *mq = (StaticQueue_t *)mq_id;
  uint32_t space;
  uint32_t isrm;

  if (mq == NULL) {
    space = 0U;
  }
  else if (IS_IRQ()) {
    isrm = taskENTER_CRITICAL_FROM_ISR();

    /* space = pxQueue->uxLength - pxQueue->uxMessagesWaiting; */
    space = mq->uxDummy4[1] - mq->uxDummy4[0];

    taskEXIT_CRITICAL_FROM_ISR(isrm);
  }
  else {
    space = (uint32_t)uxQueueSpacesAvailable ((QueueHandle_t)mq);
  }

  return (space);
}

osStatus_t osMessageQueueReset (osMessageQueueId_t mq_id) {
  QueueHandle_t hQueue = (QueueHandle_t)mq_id;
  osStatus_t stat;

  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hQueue == NULL) {
    stat = osErrorParameter;
  }
  else {
    stat = osOK;
    (void)xQueueReset (hQueue);
  }

  return (stat);
}

osStatus_t osMessageQueueDelete (osMessageQueueId_t mq_id) {
  QueueHandle_t hQueue = (QueueHandle_t)mq_id;
  osStatus_t stat;

#ifndef USE_FreeRTOS_HEAP_1
  if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else if (hQueue == NULL) {
    stat = osErrorParameter;
  }
  else {
    #if (configQUEUE_REGISTRY_SIZE > 0)
    vQueueUnregisterQueue (hQueue);
    #endif

    stat = osOK;
    vQueueDelete (hQueue);
  }
#else
  stat = osError;
#endif

  return (stat);
}

/*---------------------------------------------------------------------------*/
#ifdef FREERTOS_MPOOL_H_

/* 静态内存池函数 */
static void  FreeBlock   (MemPool_t *mp, void *block);
static void *AllocBlock  (MemPool_t *mp);
static void *CreateBlock (MemPool_t *mp);

osMemoryPoolId_t osMemoryPoolNew (uint32_t block_count, uint32_t block_size, const osMemoryPoolAttr_t *attr) {
  MemPool_t *mp;
  const char *name;
  int32_t mem_cb, mem_mp;
  uint32_t sz;

  if (IS_IRQ()) {
    mp = NULL;
  }
  else if ((block_count == 0U) || (block_size == 0U)) {
    mp = NULL;
  }
  else {
    mp = NULL;
    sz = MEMPOOL_ARR_SIZE (block_count, block_size);

    name = NULL;
    mem_cb = -1;
    mem_mp = -1;

    if (attr != NULL) {
      if (attr->name != NULL) {
        name = attr->name;
      }

      if ((attr->cb_mem != NULL) && (attr->cb_size >= sizeof(MemPool_t))) {
        /* 提供了静态控制块 */
        mem_cb = 1;
      }
      else if ((attr->cb_mem == NULL) && (attr->cb_size == 0U)) {
        /* 在堆上分配控制块内存 */
        mem_cb = 0;
      }

      if ((attr->mp_mem == NULL) && (attr->mp_size == 0U)) {
        /* 在堆上分配内存数组 */
          mem_mp = 0;
      }
      else {
        if (attr->mp_mem != NULL) {
          /* 检查数组是否为 4 字节对齐 */
          if (((uint32_t)attr->mp_mem & 3U) == 0U) {
            /* 检查数组是否足够大 */
            if (attr->mp_size >= sz) {
              /* 提供了静态内存池数组 */
              mem_mp = 1;
            }
          }
        }
      }
    }
    else {
      /* 未提供属性，在堆上分配内存 */
      mem_cb = 0;
      mem_mp = 0;
    }

    if (mem_cb == 0) {
      mp = pvPortMalloc (sizeof(MemPool_t));
    } else {
      mp = attr->cb_mem;
    }

    if (mp != NULL) {
      /* 创建信号量（最大计数 == 初始计数 == block_count） */
      #if (configSUPPORT_STATIC_ALLOCATION == 1)
        mp->sem = xSemaphoreCreateCountingStatic (block_count, block_count, &mp->mem_sem);
      #elif (configSUPPORT_DYNAMIC_ALLOCATION == 1)
        mp->sem = xSemaphoreCreateCounting (block_count, block_count);
      #else
        mp->sem == NULL;
      #endif

      if (mp->sem != NULL) {
        /* 设置内存数组 */
        if (mem_mp == 0) {
          mp->mem_arr = pvPortMalloc (sz);
        } else {
          mp->mem_arr = attr->mp_mem;
        }
      }
    }

    if ((mp != NULL) && (mp->mem_arr != NULL)) {
      /* 内存池可以创建 */
      mp->head    = NULL;
      mp->mem_sz  = sz;
      mp->name    = name;
      mp->bl_sz   = block_size;
      mp->bl_cnt  = block_count;
      mp->n       = 0U;

      /* 设置堆分配内存标志 */
      mp->status = MPOOL_STATUS;

      if (mem_cb == 0) {
        /* 控制块在堆上 */
        mp->status |= 1U;
      }
      if (mem_mp == 0) {
        /* 内存数组在堆上 */
        mp->status |= 2U;
      }
    }
    else {
      /* 无法创建内存池，释放已分配的资源 */
      if ((mem_cb == 0) && (mp != NULL)) {
        /* 释放控制块内存 */
        vPortFree (mp);
      }
      mp = NULL;
    }
  }

  return (mp);
}

const char *osMemoryPoolGetName (osMemoryPoolId_t mp_id) {
  MemPool_t *mp = (osMemoryPoolId_t)mp_id;
  const char *p;

  if (IS_IRQ()) {
    p = NULL;
  }
  else if (mp_id == NULL) {
    p = NULL;
  }
  else {
    p = mp->name;
  }

  return (p);
}

void *osMemoryPoolAlloc (osMemoryPoolId_t mp_id, uint32_t timeout) {
  MemPool_t *mp;
  void *block;
  uint32_t isrm;

  if (mp_id == NULL) {
    /* 无效的输入参数 */
    block = NULL;
  }
  else {
    block = NULL;

    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) == MPOOL_STATUS) {
      if (IS_IRQ()) {
        if (timeout == 0U) {
          if (xSemaphoreTakeFromISR (mp->sem, NULL) == pdTRUE) {
            if ((mp->status & MPOOL_STATUS) == MPOOL_STATUS) {
              isrm  = taskENTER_CRITICAL_FROM_ISR();

              /* 从空闲链表中获取一个块 */
              block = AllocBlock(mp);

              if (block == NULL) {
                /* 空闲块链表为空，'创建'新块 */
                block = CreateBlock(mp);
              }

              taskEXIT_CRITICAL_FROM_ISR(isrm);
            }
          }
        }
      }
      else {
        if (xSemaphoreTake (mp->sem, (TickType_t)timeout) == pdTRUE) {
          if ((mp->status & MPOOL_STATUS) == MPOOL_STATUS) {
            taskENTER_CRITICAL();

            /* 从空闲链表中获取一个块 */
            block = AllocBlock(mp);

            if (block == NULL) {
              /* 空闲块链表为空，'创建'新块 */
              block = CreateBlock(mp);
            }

            taskEXIT_CRITICAL();
          }
        }
      }
    }
  }

  return (block);
}

osStatus_t osMemoryPoolFree (osMemoryPoolId_t mp_id, void *block) {
  MemPool_t *mp;
  osStatus_t stat;
  uint32_t isrm;
  BaseType_t yield;

  if ((mp_id == NULL) || (block == NULL)) {
    /* 无效的输入参数 */
    stat = osErrorParameter;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* 无效的对象状态 */
      stat = osErrorResource;
    }
    else if ((block < (void *)&mp->mem_arr[0]) || (block > (void*)&mp->mem_arr[mp->mem_sz-1])) {
      /* 块指针在内存数组区域之外 */
      stat = osErrorParameter;
    }
    else {
      stat = osOK;

      if (IS_IRQ()) {
        if (uxSemaphoreGetCountFromISR (mp->sem) == mp->bl_cnt) {
          stat = osErrorResource;
        }
        else {
          isrm = taskENTER_CRITICAL_FROM_ISR();

          /* 将块添加到空闲块链表 */
          FreeBlock(mp, block);

          taskEXIT_CRITICAL_FROM_ISR(isrm);

          yield = pdFALSE;
          xSemaphoreGiveFromISR (mp->sem, &yield);
          portYIELD_FROM_ISR (yield);
        }
      }
      else {
        if (uxSemaphoreGetCount (mp->sem) == mp->bl_cnt) {
          stat = osErrorResource;
        }
        else {
          taskENTER_CRITICAL();

          /* 将块添加到空闲块链表 */
          FreeBlock(mp, block);

          taskEXIT_CRITICAL();

          xSemaphoreGive (mp->sem);
        }
      }
    }
  }

  return (stat);
}

uint32_t osMemoryPoolGetCapacity (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  uint32_t  n;

  if (mp_id == NULL) {
    /* 无效的输入参数 */
    n = 0U;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* 无效的对象状态 */
      n = 0U;
    }
    else {
      n = mp->bl_cnt;
    }
  }

  /* 返回最大内存块数 */
  return (n);
}

uint32_t osMemoryPoolGetBlockSize (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  uint32_t  sz;

  if (mp_id == NULL) {
    /* 无效的输入参数 */
    sz = 0U;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* 无效的对象状态 */
      sz = 0U;
    }
    else {
      sz = mp->bl_sz;
    }
  }

  /* 返回内存块大小（字节） */
  return (sz);
}

uint32_t osMemoryPoolGetCount (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  uint32_t  n;

  if (mp_id == NULL) {
    /* 无效的输入参数 */
    n = 0U;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* 无效的对象状态 */
      n = 0U;
    }
    else {
      if (IS_IRQ()) {
        n = uxSemaphoreGetCountFromISR (mp->sem);
      } else {
        n = uxSemaphoreGetCount        (mp->sem);
      }

      n = mp->bl_cnt - n;
    }
  }

  /* 返回已使用的内存块数 */
  return (n);
}

uint32_t osMemoryPoolGetSpace (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  uint32_t  n;

  if (mp_id == NULL) {
    /* 无效的输入参数 */
    n = 0U;
  }
  else {
    mp = (MemPool_t *)mp_id;

    if ((mp->status & MPOOL_STATUS) != MPOOL_STATUS) {
      /* 无效的对象状态 */
      n = 0U;
    }
    else {
      if (IS_IRQ()) {
        n = uxSemaphoreGetCountFromISR (mp->sem);
      } else {
        n = uxSemaphoreGetCount        (mp->sem);
      }
    }
  }

  /* 返回可用内存块数 */
  return (n);
}

osStatus_t osMemoryPoolDelete (osMemoryPoolId_t mp_id) {
  MemPool_t *mp;
  osStatus_t stat;

  if (mp_id == NULL) {
    /* 无效的输入参数 */
    stat = osErrorParameter;
  }
  else if (IS_IRQ()) {
    stat = osErrorISR;
  }
  else {
    mp = (MemPool_t *)mp_id;

    taskENTER_CRITICAL();

    /* 使控制块状态无效 */
    mp->status  = mp->status & 3U;

    /* 唤醒等待池信号量的任务 */
    while (xSemaphoreGive (mp->sem) == pdTRUE);

    mp->head    = NULL;
    mp->bl_sz   = 0U;
    mp->bl_cnt  = 0U;

    if ((mp->status & 2U) != 0U) {
      /* 内存池数组在堆上分配 */
      vPortFree (mp->mem_arr);
    }
    if ((mp->status & 1U) != 0U) {
      /* 内存池控制块在堆上分配 */
      vPortFree (mp);
    }

    taskEXIT_CRITICAL();

    stat = osOK;
  }

  return (stat);
}

/*
  根据当前块索引创建新块。
*/
static void *CreateBlock (MemPool_t *mp) {
  MemPoolBlock_t *p = NULL;

  if (mp->n < mp->bl_cnt) {
    /* 存在未分配的块，将指针设置为新块 */
    p = (void *)(mp->mem_arr + (mp->bl_sz * mp->n));

    /* 递增块索引 */
    mp->n += 1U;
  }

  return (p);
}

/*
  通过读取空闲块链表来分配一个块。
*/
static void *AllocBlock (MemPool_t *mp) {
  MemPoolBlock_t *p = NULL;

  if (mp->head != NULL) {
    /* 空闲块链表存在，获取头块 */
    p = mp->head;

    /* 头块现在是链表上的下一个 */
    mp->head = p->next;
  }

  return (p);
}

/*
  通过将块放入空闲块链表来释放块。
*/
static void FreeBlock (MemPool_t *mp, void *block) {
  MemPoolBlock_t *p = block;

  /* 将当前头存储到块内存空间中 */
  p->next = mp->head;

  /* 将当前块存储为新的头 */
  mp->head = p;
}
#endif /* FREERTOS_MPOOL_H_ */
/*---------------------------------------------------------------------------*/

/* 回调函数原型 */
extern void vApplicationIdleHook (void);
extern void vApplicationTickHook (void);
extern void vApplicationMallocFailedHook (void);
extern void vApplicationDaemonTaskStartupHook (void);
extern void vApplicationStackOverflowHook (TaskHandle_t xTask, signed char *pcTaskName);

/**
  回调函数 vApplicationIdleHook() 的虚拟实现。
*/
#if (configUSE_IDLE_HOOK == 1)
__WEAK void vApplicationIdleHook (void){}
#endif

/**
  回调函数 vApplicationTickHook() 的虚拟实现。
*/
#if (configUSE_TICK_HOOK == 1)
 __WEAK void vApplicationTickHook (void){}
#endif

/**
  回调函数 vApplicationMallocFailedHook() 的虚拟实现。
*/
#if (configUSE_MALLOC_FAILED_HOOK == 1)
__WEAK void vApplicationMallocFailedHook (void){}
#endif

/**
  回调函数 vApplicationDaemonTaskStartupHook() 的虚拟实现。
*/
#if (configUSE_DAEMON_TASK_STARTUP_HOOK == 1)
__WEAK void vApplicationDaemonTaskStartupHook (void){}
#endif

/**
  回调函数 vApplicationStackOverflowHook() 的虚拟实现。
*/
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
__WEAK void vApplicationStackOverflowHook (TaskHandle_t xTask, signed char *pcTaskName) {
  (void)xTask;
  (void)pcTaskName;
  configASSERT(0);
}
#endif

/*---------------------------------------------------------------------------*/
#if (configSUPPORT_STATIC_ALLOCATION == 1)
/* 外部空闲和定时器任务静态内存分配函数 */
extern void vApplicationGetIdleTaskMemory  (StaticTask_t **ppxIdleTaskTCBBuffer,  StackType_t **ppxIdleTaskStackBuffer,  uint32_t *pulIdleTaskStackSize);
extern void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

/*
  当 configSUPPORT_STATIC_ALLOCATION 等于 1 时，vApplicationGetIdleTaskMemory 被调用，
  是静态内存分配支持所必需的。
*/
__WEAK void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
  /* 空闲任务控制块和栈 */
  static StaticTask_t Idle_TCB;
  static StackType_t  Idle_Stack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer   = &Idle_TCB;
  *ppxIdleTaskStackBuffer = &Idle_Stack[0];
  *pulIdleTaskStackSize   = (uint32_t)configMINIMAL_STACK_SIZE;
}

/*
  当 configSUPPORT_STATIC_ALLOCATION 等于 1 时，vApplicationGetTimerTaskMemory 被调用，
  是静态内存分配支持所必需的。
*/
__WEAK void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
  /* 定时器任务控制块和栈 */
  static StaticTask_t Timer_TCB;
  static StackType_t  Timer_Stack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer   = &Timer_TCB;
  *ppxTimerTaskStackBuffer = &Timer_Stack[0];
  *pulTimerTaskStackSize   = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}
#endif
