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

/*-----------------------------------------------------------
 * 针对 ARM CM4F 移植的 portable.h 中定义的函数的实现。
 *----------------------------------------------------------*/

/* 调度器包含文件。 */
#include "FreeRTOS.h"
#include "task.h"

#ifndef __TARGET_FPU_VFP
	#error 此移植仅可在项目选项配置为启用硬件浮点支持时使用。
#endif

#if configMAX_SYSCALL_INTERRUPT_PRIORITY == 0
	#error configMAX_SYSCALL_INTERRUPT_PRIORITY 不能设置为 0。请参阅 http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html
#endif

#ifndef configSYSTICK_CLOCK_HZ
	#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
	/* 确保 SysTick 与内核以相同的频率计时。 */
	#define portNVIC_SYSTICK_CLK_BIT	( 1UL << 2UL )
#else
	/* 如果 SysTick 的时钟与内核不同，则不修改 SysTick 的时钟方式。 */
	#define portNVIC_SYSTICK_CLK_BIT	( 0 )
#endif

/* 仅用于向后兼容的旧版宏。此宏曾用于替换配置 tick 中断时钟的
函数（prvSetupTimerInterrupt()），但现在该函数被声明为弱函数，
因此应用程序编写者可以通过简单地定义一个同名函数
（vApplicationSetupTickInterrupt()）来覆盖它。 */
#ifndef configOVERRIDE_DEFAULT_TICK_CONFIGURATION
	#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION 0
#endif

/* 操作内核所需的常量。首先是寄存器…… */
#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG			( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG				( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
/* ……然后是寄存器中的位。 */
#define portNVIC_SYSTICK_INT_BIT			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )

/* 用于检测 Cortex-M7 r0p1 内核的常量，该内核应使用 ARM_CM7 r0p1 移植。 */
#define portCPUID							( * ( ( volatile uint32_t * ) 0xE000ed00 ) )
#define portCORTEX_M7_r0p1_ID				( 0x410FC271UL )
#define portCORTEX_M7_r0p0_ID				( 0x410FC270UL )

#define portNVIC_PENDSV_PRI					( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 16UL )
#define portNVIC_SYSTICK_PRI				( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )

/* 检查中断优先级有效性所需的常量。 */
#define portFIRST_USER_INTERRUPT_NUMBER		( 16 )
#define portNVIC_IP_REGISTERS_OFFSET_16 	( 0xE000E3F0 )
#define portAIRCR_REG						( * ( ( volatile uint32_t * ) 0xE000ED0C ) )
#define portMAX_8_BIT_VALUE					( ( uint8_t ) 0xff )
#define portTOP_BIT_OF_BYTE					( ( uint8_t ) 0x80 )
#define portMAX_PRIGROUP_BITS				( ( uint8_t ) 7 )
#define portPRIORITY_GROUP_MASK				( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT					( 8UL )

/* 屏蔽 ICSR 寄存器中除 VECTACTIVE 位以外的所有位。 */
#define portVECTACTIVE_MASK					( 0xFFUL )

/* 操作 VFP 所需的常量。 */
#define portFPCCR					( ( volatile uint32_t * ) 0xe000ef34 ) /* 浮点上下文控制寄存器。 */
#define portASPEN_AND_LSPEN_BITS	( 0x3UL << 30UL )

/* 设置初始栈所需的常量。 */
#define portINITIAL_XPSR			( 0x01000000 )
#define portINITIAL_EXC_RETURN		( 0xfffffffd )

/* SysTick 是一个 24 位计数器。 */
#define portMAX_24_BIT_NUMBER		( 0xffffffUL )

/* 一个修正因子，用于估算在无 tick 空闲计算期间 SysTick 计数器停止时
本应发生的 SysTick 计数次数。 */
#define portMISSED_COUNTS_FACTOR	( 45UL )

/* 为了严格遵守 Cortex-M 规范，任务起始地址的 bit-0 应为 0，
因为它在从 ISR 退出时被加载到 PC 中。 */
#define portSTART_ADDRESS_MASK		( ( StackType_t ) 0xfffffffeUL )

/*
 * 设置定时器以生成 tick 中断。此文件中的实现是弱函数，
 * 以允许应用程序编写者更改用于生成 tick 中断的定时器。
 */
void vPortSetupTimerInterrupt( void );

/*
 * 异常处理函数。
 */
void xPortPendSVHandler( void );
void xPortSysTickHandler( void );
void vPortSVCHandler( void );

/*
 * 启动第一个任务是一个单独的函数，以便可以对其进行隔离测试。
 */
static void prvStartFirstTask( void );

/*
 * 在 portasm.s 中定义的用于启用 VFP 的函数。
 */
static void prvEnableVFP( void );

/*
 * 用于捕获试图从其实现函数返回的任务。
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/* 每个任务在临界区嵌套变量中维护自己的中断状态。 */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
 * 构成一个 tick 周期的 SysTick 增量数。
 */
#if( configUSE_TICKLESS_IDLE == 1 )
	static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * 可以被抑制的最大 tick 周期数受到 SysTick 定时器
 * 24 位分辨率的限制。
 */
#if( configUSE_TICKLESS_IDLE == 1 )
	static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * 补偿 SysTick 停止期间经过的 CPU 周期（仅限低功耗功能）。
 */
#if( configUSE_TICKLESS_IDLE == 1 )
	static uint32_t ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * 由 portASSERT_IF_INTERRUPT_PRIORITY_INVALID() 宏使用，
 * 以确保 FreeRTOS API 函数不会从被分配了高于
 * configMAX_SYSCALL_INTERRUPT_PRIORITY 优先级的中断中调用。
 */
#if ( configASSERT_DEFINED == 1 )
	 static uint8_t ucMaxSysCallPriority = 0;
	 static uint32_t ulMaxPRIGROUPValue = 0;
	 static const volatile uint8_t * const pcInterruptPriorityRegisters = ( uint8_t * ) portNVIC_IP_REGISTERS_OFFSET_16;
#endif /* configASSERT_DEFINED */

/*-----------------------------------------------------------*/

/*
 * 详细描述请参见头文件。
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
	/* 模拟由上下文切换中断创建的栈帧。 */

	/* 添加偏移量以考虑 MCU 在中断进入/退出时使用栈的方式，
	并确保对齐。 */
	pxTopOfStack--;

	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ( ( StackType_t ) pxCode ) & portSTART_ADDRESS_MASK;	/* PC */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) prvTaskExitError;	/* LR */

	/* 通过跳过寄存器初始化来节省代码空间。 */
	pxTopOfStack -= 5;	/* R12、R3、R2 和 R1。 */
	*pxTopOfStack = ( StackType_t ) pvParameters;	/* R0 */

	/* 正在使用的保存方法要求每个任务维护自己的异常返回值。 */
	pxTopOfStack--;
	*pxTopOfStack = portINITIAL_EXC_RETURN;

	pxTopOfStack -= 8;	/* R11、R10、R9、R8、R7、R6、R5 和 R4。 */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
	/* 实现任务的函数不得退出或试图返回给其调用者，
	因为没有可以返回的地方。如果任务想要退出，
	应该调用 vTaskDelete( NULL )。

	如果定义了 configASSERT()，则人为强制触发 assert()，
	然后在此停止，以便应用程序编写者可以捕获此错误。 */
	configASSERT( uxCriticalNesting == ~0UL );
	portDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

__asm void vPortSVCHandler( void )
{
	PRESERVE8

	/* 获取当前 TCB 的位置。 */
	ldr	r3, =pxCurrentTCB
	ldr r1, [r3]
	ldr r0, [r1]
	/* 弹出核心寄存器。 */
	ldmia r0!, {r4-r11, r14}
	msr psp, r0
	isb
	mov r0, #0
	msr	basepri, r0
	bx r14
}
/*-----------------------------------------------------------*/

__asm void prvStartFirstTask( void )
{
	PRESERVE8

	/* 使用 NVIC 偏移寄存器来定位栈。 */
	ldr r0, =0xE000ED08
	ldr r0, [r0]
	ldr r0, [r0]
	/* 将 msp 设置回栈的起始位置。 */
	msr msp, r0
	/* 清除指示 FPU 正在使用的位，以防 FPU 在调度器启动之前被使用——
	否则会导致在 SVC 栈中不必要地保留空间用于 FPU 寄存器的惰性保存。 */
	mov r0, #0
	msr control, r0
	/* 全局启用中断。 */
	cpsie i
	cpsie f
	dsb
	isb
	/* 调用 SVC 来启动第一个任务。 */
	svc 0
	nop
	nop
}
/*-----------------------------------------------------------*/

__asm void prvEnableVFP( void )
{
	PRESERVE8

	/* FPU 启用位位于 CPACR 中。 */
	ldr.w r0, =0xE000ED88
	ldr	r1, [r0]

	/* 启用 CP10 和 CP11 协处理器，然后保存回去。 */
	orr	r1, r1, #( 0xf << 20 )
	str r1, [r0]
	bx	r14
	nop
}
/*-----------------------------------------------------------*/

/*
 * 详细描述请参见头文件。
 */
BaseType_t xPortStartScheduler( void )
{
	/* configMAX_SYSCALL_INTERRUPT_PRIORITY 不能设置为 0。
	请参阅 http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
	configASSERT( configMAX_SYSCALL_INTERRUPT_PRIORITY );

	/* 此移植可用于除 r0p1 部件之外的所有 Cortex-M7 内核版本。
	r0p1 部件应使用 /source/portable/GCC/ARM_CM7/r0p1 目录中的移植。 */
	configASSERT( portCPUID != portCORTEX_M7_r0p1_ID );
	configASSERT( portCPUID != portCORTEX_M7_r0p0_ID );

	#if( configASSERT_DEFINED == 1 )
	{
		volatile uint32_t ulOriginalPriority;
		volatile uint8_t * const pucFirstUserPriorityRegister = ( uint8_t * ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
		volatile uint8_t ucMaxPriorityValue;

		/* 确定可以从中调用 ISR 安全 FreeRTOS API 函数的最大优先级。
		ISR 安全函数是那些以 "FromISR" 结尾的函数。FreeRTOS 维护
		独立的线程和 ISR API 函数，以确保中断进入尽可能快速和简单。

		保存即将被覆盖的中断优先级值。 */
		ulOriginalPriority = *pucFirstUserPriorityRegister;

		/* 确定可用的优先级位数。首先写入所有可能的位。 */
		*pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

		/* 读回该值以查看有多少位有效。 */
		ucMaxPriorityValue = *pucFirstUserPriorityRegister;

		/* 内核中断优先级应设置为最低优先级。 */
		configASSERT( ucMaxPriorityValue == ( configKERNEL_INTERRUPT_PRIORITY & ucMaxPriorityValue ) );

		/* 对最大系统调用优先级使用相同的掩码。 */
		ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

		/* 根据读回的位数计算最大可接受的优先级组值。 */
		ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;
		while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
		{
			ulMaxPRIGROUPValue--;
			ucMaxPriorityValue <<= ( uint8_t ) 0x01;
		}

		#ifdef __NVIC_PRIO_BITS
		{
			/* 检查定义优先级位数的 CMSIS 配置是否与从硬件
			实际查询到的优先级位数匹配。 */
			configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == __NVIC_PRIO_BITS );
		}
		#endif

		#ifdef configPRIO_BITS
		{
			/* 检查定义优先级位数的 FreeRTOS 配置是否与从硬件
			实际查询到的优先级位数匹配。 */
			configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == configPRIO_BITS );
		}
		#endif

		/* 将优先级组值移回其在 AIRCR 寄存器中的位置。 */
		ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
		ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;

		/* 将被覆盖的中断优先级寄存器恢复为其原始值。 */
		*pucFirstUserPriorityRegister = ulOriginalPriority;
	}
	#endif /* conifgASSERT_DEFINED */

	/* 使 PendSV 和 SysTick 成为最低优先级的中断。 */
	portNVIC_SYSPRI2_REG |= portNVIC_PENDSV_PRI;
	portNVIC_SYSPRI2_REG |= portNVIC_SYSTICK_PRI;

	/* 启动生成 tick ISR 的定时器。此处中断已经被禁用。 */
	vPortSetupTimerInterrupt();

	/* 初始化临界区嵌套计数，为第一个任务做好准备。 */
	uxCriticalNesting = 0;

	/* 确保 VFP 已启用——它本来就应该已启用。 */
	prvEnableVFP();

	/* 始终使用惰性保存。 */
	*( portFPCCR ) |= portASPEN_AND_LSPEN_BITS;

	/* 启动第一个任务。 */
	prvStartFirstTask();

	/* 不应该执行到这里！ */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* 在没有可返回内容的移植中未实现。
	人为强制触发 assert。 */
	configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;

	/* 这不是进入临界区函数的中断安全版本，
	因此如果从中断上下文中调用它，则触发 assert()。
	只有以 "FromISR" 结尾的 API 函数才能在中断中使用。
	仅在临界区嵌套计数为 1 时触发 assert，以防止在 assert 函数
	也使用临界区的情况下出现递归调用。 */
	if( uxCriticalNesting == 1 )
	{
		configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
	}
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	configASSERT( uxCriticalNesting );
	uxCriticalNesting--;
	if( uxCriticalNesting == 0 )
	{
		portENABLE_INTERRUPTS();
	}
}
/*-----------------------------------------------------------*/

__asm void xPortPendSVHandler( void )
{
	extern uxCriticalNesting;
	extern pxCurrentTCB;
	extern vTaskSwitchContext;

	PRESERVE8

	mrs r0, psp
	isb
	/* 获取当前 TCB 的位置。 */
	ldr	r3, =pxCurrentTCB
	ldr	r2, [r3]

	/* 任务是否正在使用 FPU 上下文？如果是，则压入高位 VFP 寄存器。 */
	tst r14, #0x10
	it eq
	vstmdbeq r0!, {s16-s31}

	/* 保存核心寄存器。 */
	stmdb r0!, {r4-r11, r14}

	/* 将新的栈顶保存到 TCB 的第一个成员中。 */
	str r0, [r2]

	stmdb sp!, {r0, r3}
	mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY
	msr basepri, r0
	dsb
	isb
	bl vTaskSwitchContext
	mov r0, #0
	msr basepri, r0
	ldmia sp!, {r0, r3}

	/* pxCurrentTCB 中的第一项是任务的栈顶。 */
	ldr r1, [r3]
	ldr r0, [r1]

	/* 弹出核心寄存器。 */
	ldmia r0!, {r4-r11, r14}

	/* 任务是否正在使用 FPU 上下文？如果是，则同时弹出高位 VFP 寄存器。 */
	tst r14, #0x10
	it eq
	vldmiaeq r0!, {s16-s31}

	msr psp, r0
	isb
	#ifdef WORKAROUND_PMU_CM001 /* XMC4000 特定勘误 */
		#if WORKAROUND_PMU_CM001 == 1
			push { r14 }
			pop { pc }
			nop
		#endif
	#endif

	bx r14
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
	/* SysTick 以最低中断优先级运行，因此当此中断执行时，
	所有中断必须是非屏蔽的。因此无需保存和恢复中断掩码值，
	因为其值已经是已知的——因此使用稍快的 vPortRaiseBASEPRI()
	函数代替 portSET_INTERRUPT_MASK_FROM_ISR()。 */
	vPortRaiseBASEPRI();
	{
		/* 递增 RTOS tick。 */
		if( xTaskIncrementTick() != pdFALSE )
		{
			/* 需要进行上下文切换。上下文切换在 PendSV 中断中执行。
			触发 PendSV 中断。 */
			portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
		}
	}
	vPortClearBASEPRIFromISR();
}
/*-----------------------------------------------------------*/

#if( configUSE_TICKLESS_IDLE == 1 )

	__weak void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
	{
	uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
	TickType_t xModifiableIdleTime;

		/* 确保 SysTick 重载值不会使计数器溢出。 */
		if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
		{
			xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
		}

		/* 暂时停止 SysTick。SysTick 停止的时间尽可能被考虑在内，
		但使用无 tick 模式将不可避免地导致内核维护的时间
		相对于日历时间产生一些微小的漂移。 */
		portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

		/* 计算等待 xExpectedIdleTime 个 tick 周期所需的重载值。
		使用 -1 是因为此代码将在某个 tick 周期内部分执行。 */
		ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );
		if( ulReloadValue > ulStoppedTimerCompensation )
		{
			ulReloadValue -= ulStoppedTimerCompensation;
		}

		/* 进入临界区，但不使用 taskENTER_CRITICAL() 方法，
		因为那会屏蔽应该退出睡眠模式的中断。 */
		__disable_irq();
		__dsb( portSY_FULL_READ_WRITE );
		__isb( portSY_FULL_READ_WRITE );

		/* 如果有一个上下文切换待处理，或者有任务正在等待调度器
		被取消挂起，则放弃低功耗进入。 */
		if( eTaskConfirmSleepModeStatus() == eAbortSleep )
		{
			/* 从计数寄存器中剩余的任何值重新启动以完成此 tick 周期。 */
			portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

			/* 重新启动 SysTick。 */
			portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

			/* 将重载寄存器重置为正常 tick 周期所需的值。 */
			portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

			/* 重新启用中断——请参阅上面 __disable_irq() 调用的注释。 */
			__enable_irq();
		}
		else
		{
			/* 设置新的重载值。 */
			portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

			/* 清除 SysTick 计数标志并将计数值重置为零。 */
			portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

			/* 重新启动 SysTick。 */
			portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

			/* 睡眠直到有事件发生。configPRE_SLEEP_PROCESSING() 可以
			将其参数设置为 0，以表示其实现中包含自己的等待中断或
			等待事件指令，因此不应再次执行 wfi。但是，原始的预期
			空闲时间变量必须保持不变，因此使用一个副本。 */
			xModifiableIdleTime = xExpectedIdleTime;
			configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
			if( xModifiableIdleTime > 0 )
			{
				__dsb( portSY_FULL_READ_WRITE );
				__wfi();
				__isb( portSY_FULL_READ_WRITE );
			}
			configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

			/* 重新启用中断，以允许将 MCU 从睡眠模式唤醒的中断
			立即执行。请参阅上面 __disable_interrupt() 调用的注释。 */
			__enable_irq();
			__dsb( portSY_FULL_READ_WRITE );
			__isb( portSY_FULL_READ_WRITE );

			/* 再次禁用中断，因为时钟即将被停止，而在时钟停止期间
			执行的中断会增加 RTOS 维护的时间与日历时间之间的
			任何偏差。 */
			__disable_irq();
			__dsb( portSY_FULL_READ_WRITE );
			__isb( portSY_FULL_READ_WRITE );

			/* 禁用 SysTick 时钟，不读取 portNVIC_SYSTICK_CTRL_REG 寄存器，
			以确保如果 portNVIC_SYSTICK_COUNT_FLAG_BIT 被设置，
			它不会被清除。同样，SysTick 停止的时间尽可能被考虑在内，
			但使用无 tick 模式将不可避免地导致内核维护的时间
			相对于日历时间产生一些微小的漂移。 */
			portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT );

			/* 确定 SysTick 时钟是否已经计数到零并被设置回当前重载值
			（该重载值对整个预期空闲时间是正确的），还是 SysTick
			尚未计数到零（在这种情况下，必须是 SysTick 以外的中断
			将系统从睡眠模式唤醒）。 */
			if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
			{
				uint32_t ulCalculatedLoadValue;

				/* Tick 中断已经挂起，SysTick 计数已用 ulReloadValue 重新加载。
				用此 tick 周期的剩余值重置 portNVIC_SYSTICK_LOAD_REG。 */
				ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL ) - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

				/* 不允许使用微小的值，或者由于睡眠后钩子执行时间过长
				而以某种方式下溢的值。 */
				if( ( ulCalculatedLoadValue < ulStoppedTimerCompensation ) || ( ulCalculatedLoadValue > ulTimerCountsForOneTick ) )
				{
					ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL );
				}

				portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

				/* 由于挂起的 tick 将在此函数退出后立即被处理，
				由 tick 维护的 tick 值向前步进比等待时间少一。 */
				ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
			}
			else
			{
				/* 是 tick 中断以外的其他事件结束了睡眠。
				计算睡眠持续了多长时间，四舍五入到完整的 tick 周期
				（不是考虑了部分 tick 的 ulReload 值）。 */
				ulCompletedSysTickDecrements = ( xExpectedIdleTime * ulTimerCountsForOneTick ) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

				/* 处理器等待期间经过了多少个完整的 tick 周期？ */
				ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

				/* 重载值设置为单个 tick 周期剩余的任意部分。 */
				portNVIC_SYSTICK_LOAD_REG = ( ( ulCompleteTickPeriods + 1UL ) * ulTimerCountsForOneTick ) - ulCompletedSysTickDecrements;
			}

			/* 重新启动 SysTick，使其再次从 portNVIC_SYSTICK_LOAD_REG 运行，
			然后将 portNVIC_SYSTICK_LOAD_REG 设置回其标准值。 */
			portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
			portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
			vTaskStepTick( ulCompleteTickPeriods );
			portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

			/* 在启用中断的情况下退出。 */
			__enable_irq();
		}
	}

#endif /* #if configUSE_TICKLESS_IDLE */

/*-----------------------------------------------------------*/

/*
 * 设置 SysTick 定时器以所需频率生成 tick 中断。
 */
#if( configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 0 )

	__weak void vPortSetupTimerInterrupt( void )
	{
		/* 计算配置 tick 中断所需的常量。 */
		#if( configUSE_TICKLESS_IDLE == 1 )
		{
			ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
			xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
			ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
		}
		#endif /* configUSE_TICKLESS_IDLE */

		/* 停止并清除 SysTick。 */
		portNVIC_SYSTICK_CTRL_REG = 0UL;
		portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

		/* 配置 SysTick 以请求的频率中断。 */
		portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
		portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
	}

#endif /* configOVERRIDE_DEFAULT_TICK_CONFIGURATION */
/*-----------------------------------------------------------*/

__asm uint32_t vPortGetIPSR( void )
{
	PRESERVE8

	mrs r0, ipsr
	bx r14
}
/*-----------------------------------------------------------*/

#if( configASSERT_DEFINED == 1 )

	void vPortValidateInterruptPriority( void )
	{
	uint32_t ulCurrentInterrupt;
	uint8_t ucCurrentPriority;

		/* 获取当前正在执行的中断的编号。 */
		ulCurrentInterrupt = vPortGetIPSR();

		/* 该中断号是用户定义的中断吗？ */
		if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
		{
			/* 查找该中断的优先级。 */
			ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];

			/* 以下断言将在以下情况下失败：一个被分配了高于
			configMAX_SYSCALL_INTERRUPT_PRIORITY 优先级的中断的
			服务例程（ISR）调用了 ISR 安全的 FreeRTOS API 函数。
			ISR 安全的 FreeRTOS API 函数必须*仅*从被分配了等于或低于
			configMAX_SYSCALL_INTERRUPT_PRIORITY 优先级的中断中调用。

			数值上低的中断优先级数字表示逻辑上高的中断优先级，
			因此中断的优先级必须设置为等于或数值上*高于*
			configMAX_SYSCALL_INTERRUPT_PRIORITY 的值。

			使用 FreeRTOS API 的中断不能保持其默认优先级零，
			因为那是最高的可能优先级，它保证高于
			configMAX_SYSCALL_INTERRUPT_PRIORITY，因此也保证是无效的。

			FreeRTOS 维护独立的线程和 ISR API 函数，
			以确保中断进入尽可能快速和简单。

			以下链接提供详细信息：
			http://www.freertos.org/RTOS-Cortex-M3-M4.html
			http://www.freertos.org/FAQHelp.html */
			configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
		}

		/* 优先级分组：中断控制器（NVIC）允许定义每个中断优先级的位
		被分割为定义中断抢占优先级的位和定义中断子优先级的位。
		为简单起见，所有位必须被定义为抢占优先级位。如果不是这种情况
		（如果某些位表示子优先级），以下断言将失败。

		如果应用程序仅使用 CMSIS 库进行中断配置，则可以通过在启动
		调度器之前调用 NVIC_SetPriorityGrouping( 0 ); 在所有 Cortex-M
		设备上实现正确的设置。但请注意，一些供应商特定的外设库
		假定非零的优先级组设置，在这种情况下使用零值将导致
		不可预测的行为。 */
		configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
	}

#endif /* configASSERT_DEFINED */
