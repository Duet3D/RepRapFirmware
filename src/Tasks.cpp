/*
 * Startup.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#include "Tasks.h"
#include "RepRap.h"
#include "Platform.h"
#include "Hardware/Cache.h"

#include <malloc.h>

#ifdef RTOS
# include "FreeRTOS.h"
# include "task.h"
#endif

const uint8_t memPattern = 0xA5;

extern "C" char *sbrk(int i);
extern char _end;

#ifdef RTOS

// The main task currently runs GCodes, so it needs to be large enough to hold the matrices used for auto calibration.
// The timer and idle tasks currently never do I/O, so they can be much smaller.
#if defined(LPC_NETWORKING)
constexpr unsigned int MainTaskStackWords = 1600-424;
#else
constexpr unsigned int MainTaskStackWords = 1600;
#endif

constexpr unsigned int IdleTaskStackWords = 60;

static Task<IdleTaskStackWords> idleTask;
static Task<MainTaskStackWords> mainTask;

static Mutex spiMutex;
static Mutex i2cMutex;
static Mutex sysDirMutex;
static Mutex mallocMutex;

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = idleTask.GetTaskMemory();
	*ppxIdleTaskStackBuffer = idleTask.GetStackBase();
	*pulIdleTaskStackSize = idleTask.GetStackSize();
}

#if configUSE_TIMERS

constexpr unsigned int TimerTaskStackWords = 60;
static Task<TimerTaskStackWords> timerTask;

extern "C" void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = timerTask.GetTaskMemory();
    *ppxTimerTaskStackBuffer = timerTask.GetStackBase();
    *pulTimerTaskStackSize = timerTask.GetStackSize();
}

#endif

extern "C" void MainTask(void * pvParameters);

// We need to make malloc/free thread safe, else sprintf and related I/O functions are liable to crash.
// We must use a recursive mutex for it.
extern "C" void __malloc_lock ( struct _reent *_r )
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't take mutex if scheduler not started or suspended
	{
		mallocMutex.Take();
	}
}

extern "C" void __malloc_unlock ( struct _reent *_r )
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't release mutex if scheduler not started or suspended
	{
		mallocMutex.Release();
	}
}

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

#ifdef __LPC17xx__
	// Setup LEDs, start off
	pinMode(LED_PLAY, OUTPUT_LOW);
	pinMode(LED1, OUTPUT_LOW);
	pinMode(LED2, OUTPUT_LOW);
	pinMode(LED3, OUTPUT_LOW);
	pinMode(LED4, OUTPUT_LOW);
#else
	// When doing a software reset, we disable the NRST input (User reset) to prevent the negative-going pulse that gets generated on it
	// being held in the capacitor and changing the reset reason from Software to User. So enable it again here. We hope that the reset signal
	// will have gone away by now.
# ifndef RSTC_MR_KEY_PASSWD
// Definition of RSTC_MR_KEY_PASSWD is missing in the SAM3X ASF files
#  define RSTC_MR_KEY_PASSWD (0xA5u << 24)
# endif
	RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD | RSTC_MR_URSTEN;	// ignore any signal on the NRST pin for now so that the reset reason will show as Software
#endif

	Cache::Init();
	Cache::Enable();

#ifdef RTOS
	// Add the FreeRTOS internal tasks to the task list
	idleTask.AddToList();

# if configUSE_TIMERS
	timerTask.AddToList();
# endif

	// Create the startup task
	mainTask.Create(MainTask, "MAIN", nullptr, TaskPriority::SpinPriority);
	vTaskStartScheduler();			// doesn't return
}

extern "C" void MainTask(void *pvParameters)
{
	mallocMutex.Create("Malloc");
	spiMutex.Create("SPI");
	i2cMutex.Create("I2C");
	sysDirMutex.Create("SysDir");
#else
	SysTickInit();					// set up system tick interrupt
#endif

	reprap.Init();
	for (;;)
	{
		reprap.Spin();
	}
}

extern "C" uint32_t _estack;		// this is defined in the linker script

#if __LPC17xx__
	// These are defined in Linker Scripts for LPC
	extern "C" unsigned int __AHB0_block_start;
	extern "C" unsigned int __AHB0_dyn_start;
	extern "C" unsigned int __AHB0_end;

	extern "C" unsigned long __StackLimit;
	extern "C" unsigned long __StackTop;

	extern "C" size_t xPortGetTotalHeapSize( void );

	volatile uint8_t sysTickLed = 0;
#endif

namespace Tasks
{

#ifdef RTOS
	static void GetHandlerStackUsage(uint32_t* maxStack, uint32_t* neverUsed)
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
#else
	// Return the system stack usage and amount of memory that has never been used, in bytes
	static void GetStackUsage(uint32_t* currentStack, uint32_t* maxStack, uint32_t* neverUsed)
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

	uint32_t GetNeverUsedRam()
	{
		uint32_t neverUsedRam;

#ifdef RTOS
		GetHandlerStackUsage(nullptr, &neverUsedRam);
#else
		GetStackUsage(nullptr, nullptr, &neverUsedRam);
#endif
		return neverUsedRam;
	}

	// Write data about the current task (if RTOS) or the system
	void Diagnostics(MessageType mtype)
	{
		Platform& p = reprap.GetPlatform();
#ifdef RTOS
		p.Message(mtype, "=== RTOS ===\n");
#else
		p.Message(mtype, "=== System ===\n");
#endif
		// Print memory stats
		{
			const char * const ramstart =
#if SAME70
				(char *) 0x20400000;
#elif SAM4E || SAM4S
				(char *) 0x20000000;
#elif SAM3XA
				(char *) 0x20070000;
#elif __LPC17xx__
				(char *) 0x10000000;
#else
# error Unsupported processor
#endif
			p.MessageF(mtype, "Static ram: %d\n", &_end - ramstart);

			const struct mallinfo mi = mallinfo();
			p.MessageF(mtype, "Dynamic ram: %d of which %d recycled\n", mi.uordblks, mi.fordblks);

			uint32_t maxStack, neverUsed;
#ifdef RTOS
			GetHandlerStackUsage(&maxStack, &neverUsed);
			p.MessageF(mtype, "Exception stack ram used: %" PRIu32 "\n", maxStack);
#else
			uint32_t currentStack;
			Tasks::GetStackUsage(&currentStack, &maxStack, &neverUsed);
			p.MessageF(mtype, "Stack ram used: %" PRIu32 " current, %" PRIu32 " maximum\n", currentStack, maxStack);
#endif
			p.MessageF(mtype, "Never used ram: %" PRIu32 "\n", neverUsed);

#ifdef __LPC17xx__
			const uint32_t ahbStaticUsed = (uint32_t)&__AHB0_dyn_start -(uint32_t)&__AHB0_block_start;
			const uint32_t totalMainUsage = (uint32_t)((&_end - ramstart) + mi.uordblks + maxStack);

			p.MessageF(mtype, "AHB_RAM Static ram used : %" PRIu32 "\n", ahbStaticUsed);
			p.Message(mtype, "=== Ram Totals ===\n");
			p.MessageF(mtype, "Main SRAM         : %" PRIu32 "/%" PRIu32 " (%" PRIu32 " free, %" PRIu32 " never used)\n", totalMainUsage, (uint32_t)32*1024, 32*1024-totalMainUsage, neverUsed );
			p.MessageF(mtype, "RTOS Dynamic Heap : %" PRIi32 "/%" PRIu32 " (%d free, %d never used)\n", (uint32_t)(xPortGetTotalHeapSize()-xPortGetFreeHeapSize()),(uint32_t)xPortGetTotalHeapSize(), xPortGetFreeHeapSize(),xPortGetMinimumEverFreeHeapSize() );

			//Print out the PWM and timers freq
			uint16_t freqs[4];
			GetTimerInfo(freqs);
			p.MessageF(mtype, "\n=== LPC PWM ===\n");
			p.MessageF(mtype, "Hardware PWM: %d Hz\nPWMTimer1: %d Hz\nPWMTimer2: %d Hz\nPWMTimer3: %d Hz\n", freqs[0], freqs[1], freqs[2], freqs[3]);

			//Print out our Special Pins Available:
			p.MessageF(mtype, "\n=== GPIO Special Pins available === (i.e. with M42)\nLogicalPin - PhysicalPin\n");
			for (size_t i=0; i<ARRAY_SIZE(SpecialPinMap); i++)
			{
				if (SpecialPinMap[i] != NoPin)
				{
					const uint8_t portNumber =  (SpecialPinMap[i]>>5);		// Divide the pin number by 32 go get the PORT number
					const uint8_t pinNumber  =   SpecialPinMap[i] & 0x1f;	// lower 5-bits contains the bit number of a 32bit port

					p.MessageF(mtype, " %d - P%d_%d ", (60+i), portNumber, pinNumber);
					if (TimerPWMPinsArray[SpecialPinMap[i]])
					{
						const uint8_t tim = TimerPWMPinsArray[SpecialPinMap[i]] & 0x0F;
						p.MessageF(mtype, "[Timer %s]", (tim&TimerPWM_1)?"1":(tim&TimerPWM_2)?"2":"3" );
					}
					else if ((g_APinDescription[SpecialPinMap[i]].ulPinAttribute & PIN_ATTR_PWM)==PIN_ATTR_PWM)
					{
						p.MessageF(mtype, "[HW PWM]");
					}
					p.MessageF(mtype, "\n");
				}
			}
#endif //end __LPC17xx__

		}	// end memory stats scope

#ifdef RTOS
		p.Message(mtype, "Tasks:");
		for (const TaskBase *t = TaskBase::GetTaskList(); t != nullptr; t = t->GetNext())
		{
			TaskStatus_t taskDetails;
			vTaskGetInfo(t->GetHandle(), &taskDetails, pdTRUE, eInvalid);
			const char* const stateText = (taskDetails.eCurrentState == eRunning) ? "running"
											: (taskDetails.eCurrentState == eReady) ? "ready"
												: (taskDetails.eCurrentState == eBlocked) ? "blocked"
													: (taskDetails.eCurrentState == eSuspended) ? "suspended"
														: "invalid";
			p.MessageF(mtype, " %s(%s,%u)",
				taskDetails.pcTaskName, stateText, (unsigned int)(taskDetails.usStackHighWaterMark * sizeof(StackType_t)));
		}
		p.Message(mtype, "\nOwned mutexes:");

		for (const Mutex *m = Mutex::GetMutexList(); m != nullptr; m = m->GetNext())
		{
			const TaskHandle holder = m->GetHolder();
			if (holder != nullptr)
			{
				p.MessageF(mtype, " %s(%s)", m->GetName(), pcTaskGetName(holder));
			}
		}
		p.MessageF(mtype, "\n");
#endif
	}

	const Mutex *GetSpiMutex()
	{
#ifdef RTOS
		return &spiMutex;
#else
		return nullptr;
#endif
	}

	const Mutex *GetI2CMutex()
	{
#ifdef RTOS
		return &i2cMutex;
#else
		return nullptr;
#endif
	}

	const Mutex *GetSysDirMutex()
	{
#ifdef RTOS
		return &sysDirMutex;
#else
		return nullptr;
#endif
	}
}

// Exception handlers
extern "C"
{
	// This intercepts the 1ms system tick
#ifdef RTOS
	void vApplicationTickHook()
#else
	void SysTick_Handler()
#endif
	{
		CoreSysTick();
		reprap.Tick();

#ifdef __LPC17xx__
		//blink the PLAY_LED to indicate systick is running
		sysTickLed++;						//uint8_t let it wrap around
		if (sysTickLed == 255)
		{
			const bool state = GPIO_PinRead(LED_PLAY);
			GPIO_PinWrite(LED_PLAY, !state);
		}
#endif
	}

	// Exception handlers
	// By default the Usage Fault, Bus Fault and Memory Management fault handlers are not enabled,
	// so they escalate to a Hard Fault and we don't need to provide separate exception handlers for them.
	void hardFaultDispatcher(const uint32_t *pulFaultStackAddress) __attribute((noreturn));
	void hardFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::hardFault, pulFaultStackAddress + 5);
	}

	// The fault handler implementation calls a function called hardFaultDispatcher()
	void HardFault_Handler() __attribute__((naked, noreturn));
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

	void wdtFaultDispatcher(const uint32_t *pulFaultStackAddress) __attribute((noreturn));
	void wdtFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::wdtFault, pulFaultStackAddress + 5);
	}

#if __LPC17xx__
    void WDT_IRQHandler() __attribute__((naked, noreturn));
    void WDT_IRQHandler(void)
    {
    	LPC_WDT->WDMOD &=~((uint32_t)(1<<2)); //SD::clear timout flag before resetting to prevent the Smoothie bootloader going into DFU mode
#else
    void WDT_Handler() __attribute__((naked, noreturn));
	void WDT_Handler()
	{
#endif
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

	void otherFaultDispatcher(const uint32_t *pulFaultStackAddress) __attribute((noreturn));
	void otherFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::otherFault, pulFaultStackAddress + 5);
	}

	// 2017-05-25: A user is getting 'otherFault' reports, so now we do a stack dump for those too.
	// The fault handler implementation calls a function called otherFaultDispatcher()
	void OtherFault_Handler() __attribute__((naked, noreturn));
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

	void DebugMon_Handler   () __attribute__ ((noreturn,alias("OtherFault_Handler")));

#ifdef RTOS

// FreeRTOS hooks that we need to provide
void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress, char* pcTaskName) __attribute((noreturn));
void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress, char* pcTaskName)
{
	reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::stackOverflow, pulFaultStackAddress);
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) __attribute((naked, noreturn));
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	// r0 = pxTask, r1 = pxTaskName
	__asm volatile
	(
		" push {r0, r1, lr}											\n"		/* save parameters and call address on the stack */
		" mov r0, sp												\n"
		" ldr r2, handler_sovf_address_const                        \n"
		" bx r2                                                     \n"
		" handler_sovf_address_const: .word stackOverflowDispatcher \n"
	);
}

#else

// Handlers that FreeRTOS would provide if we were using it
void SVC_Handler		( void ) __attribute__ ((noreturn, alias("OtherFault_Handler")));
void PendSV_Handler		( void ) __attribute__ ((noreturn, alias("OtherFault_Handler")));

#endif

	void assertCalledDispatcher(const uint32_t *pulFaultStackAddress) __attribute((noreturn));
	void assertCalledDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
	}

	void vAssertCalled(uint32_t line, const char *file) __attribute((naked, noreturn));
	void vAssertCalled(uint32_t line, const char *file)
	{
	    __asm volatile
	    (
	    	" push {r0, r1, lr}											\n"		/* save parameters and call address */
	    	" mov r0, sp												\n"
	        " ldr r2, handler_asrt_address_const                        \n"
	        " bx r2                                                     \n"
	        " handler_asrt_address_const: .word assertCalledDispatcher  \n"
	    );
	}

#if __LPC17xx__
	void applicationMallocFailedCalledDispatcher(const uint32_t *pulFaultStackAddress)
	{
		reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
	}

	void vApplicationMallocFailedHook( void ) __attribute((naked));
	void vApplicationMallocFailedHook( void )
	{
		 __asm volatile
		(
			" push {r0, r1, lr}											\n"        /* save parameters and call address */
			" mov r0, sp												\n"
			" ldr r2, handler_amf_address_const							\n"
			" bx r2														\n"
			" handler_amf_address_const: .word applicationMallocFailedCalledDispatcher  \n"
		 );
	}
#endif

}	// end extern "C"

namespace std
{
	// We need to define this function in order to use lambda functions with captures
	void __throw_bad_function_call() { vAssertCalled(__LINE__, __FILE__); }
}

// End
