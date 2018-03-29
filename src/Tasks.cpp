/*
 * Startup.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#include "Tasks.h"
#include "RepRap.h"
#include "Platform.h"

#ifdef RTOS
# include "FreeRTOS.h"
# include "task.h"
#endif

const uint8_t memPattern = 0xA5;

extern "C" char *sbrk(int i);

#ifdef RTOS
const uint32_t MainTaskStackSize = 2048;			// task stack size in dwords
const uint32_t MainTaskPriority = 1;

static StackType_t mainTaskStack[MainTaskStackSize];
static StaticTask_t mainTaskBuffer;
static TaskHandle_t mainTaskHandle;

extern "C" void MainTask(void * pvParameters);
#endif

// Application entry point
extern "C" void AppMain()
{
	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char* heapend = sbrk(0);
	register const char * stack_ptr asm ("sp");
	while (heapend + 16 < stack_ptr)
	{
		*heapend++ = memPattern;
	}

	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

	// When doing a software reset, we disable the NRST input (User reset) to prevent the negative-going pulse that gets generated on it
	// being held in the capacitor and changing the reset reason from Software to User. So enable it again here. We hope that the reset signal
	// will have gone away by now.
#ifndef RSTC_MR_KEY_PASSWD
// Definition of RSTC_MR_KEY_PASSWD is missing in the SAM3X ASF files
# define RSTC_MR_KEY_PASSWD (0xA5u << 24)
#endif
	RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD | RSTC_MR_URSTEN;	// ignore any signal on the NRST pin for now so that the reset reason will show as Software

#if USE_CACHE
	// Enable the cache
	struct cmcc_config g_cmcc_cfg;
	cmcc_get_config_defaults(&g_cmcc_cfg);
	cmcc_init(CMCC, &g_cmcc_cfg);
	EnableCache();
#endif

#ifdef RTOS
	// Create the startup task
	mainTaskHandle = xTaskCreateStatic(MainTask, "MAIN", ARRAY_SIZE(mainTaskStack), nullptr, MainTaskPriority, mainTaskStack, &mainTaskBuffer);
	vTaskStartScheduler();			// doesn't return
}

extern "C" void MainTask(void *pvParameters)
{
#endif
	reprap.Init();
	for (;;)
	{
		reprap.Spin();
	}
}

extern "C" uint32_t _estack;		// this is defined in the linker script

namespace Tasks
{

#ifdef RTOS

	void GetHandlerStackUsage(uint32_t* maxStack, uint32_t* neverUsed)
	{
		const char * const ramend = (const char *)&_estack;
		const char * const heapend = sbrk(0);
		const char * stack_lwm = heapend;
		while (stack_lwm < ramend && *stack_lwm == memPattern)
		{
			++stack_lwm;
		}
		if (maxStack != nullptr) { *maxStack = ramend - stack_lwm; }
		if (neverUsed != nullptr) { *neverUsed = stack_lwm - heapend; }
	}

	void TaskDiagnostics(MessageType mtype, TaskHandle_t ct)
	{
		TaskStatus_t taskDetails;
		vTaskGetInfo(ct, &taskDetails, pdTRUE, eInvalid);
		reprap.GetPlatform().MessageF(mtype, "Task %s: state %d stack rem %u\n", taskDetails.pcTaskName, (int)taskDetails.eCurrentState, (unsigned int)taskDetails.usStackHighWaterMark);
	}

	// Write data about the current task
	void CurrentTaskDiagnostics(MessageType mtype)
	{
		const TaskHandle_t ct = xTaskGetCurrentTaskHandle();
		TaskDiagnostics(mtype, ct);
	}

#else

	// Return the system stack usage and amount of memory that has never been used, in bytes
	void GetStackUsage(uint32_t* currentStack, uint32_t* maxStack, uint32_t* neverUsed)
	{
		const char * const ramend = (const char *)&_estack;
		const char * const heapend = sbrk(0);
		const char * stack_lwm = heapend;
		register const char * stack_ptr asm ("sp");
		while (stack_lwm < stack_ptr && *stack_lwm == memPattern)
		{
			++stack_lwm;
		}
		if (currentStack != nullptr) { *currentStack = ramend - stack_ptr; }
		if (maxStack != nullptr) { *maxStack = ramend - stack_lwm; }
		if (neverUsed != nullptr) { *neverUsed = stack_lwm - heapend; }
	}

#endif
}

#ifdef RTOS

static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

extern "C" void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer. */
    *pulIdleTaskStackSize = ARRAY_SIZE(uxIdleTaskStack);
}

static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

extern "C" void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Timer task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer. */
    *pulTimerTaskStackSize = ARRAY_SIZE(uxTimerTaskStack);
}

#endif

// Exception handlers
extern "C"
{
	// This intercepts the 1ms system tick. It must return 'false', otherwise the Arduino core tick handler will be bypassed.
	int sysTickHook()
	{
		reprap.Tick();
		return 0;
	}

	// Exception handlers
	// By default the Usage Fault, Bus Fault and Memory Management fault handlers are not enabled,
	// so they escalate to a Hard Fault and we don't need to provide separate exception handlers for them.
	void hardFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::hardFault, pulFaultStackAddress + 5);
	}

	// The fault handler implementation calls a function called hardFaultDispatcher()
	void HardFault_Handler() __attribute__((naked));
	void HardFault_Handler()
	{
	    __asm volatile
	    (
	        " tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
	        " ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
	        " mrseq r0, msp                                             \n"
	        " mrsne r0, psp                                             \n"
	        " ldr r2, handler_hf_address_const                          \n"
	        " bx r2                                                     \n"
	        " handler_hf_address_const: .word hardFaultDispatcher       \n"
	    );
	}

	void wdtFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::wdtFault, pulFaultStackAddress + 5);
	}

	void WDT_Handler() __attribute__((naked));
	void WDT_Handler()
	{
	    __asm volatile
	    (
	        " tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
	        " ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
	        " mrseq r0, msp                                             \n"
	        " mrsne r0, psp                                             \n"
	        " ldr r2, handler_wdt_address_const                         \n"
	        " bx r2                                                     \n"
	        " handler_wdt_address_const: .word wdtFaultDispatcher       \n"
	    );
	}

	void otherFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::otherFault, pulFaultStackAddress + 5);
	}

	// 2017-05-25: A user is getting 'otherFault' reports, so now we do a stack dump for those too.
	// The fault handler implementation calls a function called otherFaultDispatcher()
	void OtherFault_Handler() __attribute__((naked));
	void OtherFault_Handler()
	{
	    __asm volatile
	    (
	        " tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
	        " ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
	        " mrseq r0, msp                                             \n"
	        " mrsne r0, psp                                             \n"
	        " ldr r2, handler_oflt_address_const                        \n"
	        " bx r2                                                     \n"
	        " handler_oflt_address_const: .word otherFaultDispatcher    \n"
	    );
	}

	// We could set up the following fault handlers to retrieve the program counter in the same way as for a Hard Fault,
	// however these exceptions are unlikely to occur, so for now we just report the exception type.
	void NMI_Handler        () { reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::NMI); }
	void UsageFault_Handler () { reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::usageFault); }

	void DebugMon_Handler   () __attribute__ ((alias("OtherFault_Handler")));

#ifdef RTOS
	// FreeRTOS hooks that we need to provide
	void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::stackOverflow, pulFaultStackAddress + 5);
	}

	void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) __attribute((naked));
	void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
	{
	    __asm volatile
	    (
	        " tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
	        " ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
	        " mrseq r0, msp                                             \n"
	        " mrsne r0, psp                                             \n"
	        " ldr r2, handler_sovf_address_const                        \n"
	        " bx r2                                                     \n"
	        " handler_sovf_address_const: .word stackOverflowDispatcher \n"
	    );
	}

	void assertCalledDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress + 5);
	}

	void vAssertCalled(uint32_t line, const char *file) __attribute((naked));
	void vAssertCalled(uint32_t line, const char *file)
	{
	    __asm volatile
	    (
	        " tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
	        " ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
	        " mrseq r0, msp                                             \n"
	        " mrsne r0, psp                                             \n"
	        " ldr r2, handler_asrt_address_const                        \n"
	        " bx r2                                                     \n"
	        " handler_asrt_address_const: .word assertCalledDispatcher  \n"
	    );
	}
#endif

}

// End
