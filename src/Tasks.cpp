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
#include <TaskPriorities.h>

#if SAME5x
# include <DmacManager.h>
# include <hpl_user_area.h>
#endif

#include "FreeRTOS.h"
#include "task.h"
#include <malloc.h>

const uint8_t memPattern = 0xA5;

extern "C" char *sbrk(int i);
extern char _end;						// defined in linker script
extern uint32_t _estack;				// defined in linker script

#ifndef DEBUG
extern uint32_t _firmware_crc;			// defined in linker script
#endif

// MAIN task data
// The main task currently runs GCodes, so it needs to be large enough to hold the matrices used for delta auto calibration.
// The worst case stack usage is after running delta auto calibration with Move debugging enabled.
// The timer and idle tasks currently never do I/O, so they can be much smaller.
#if SAME70 || SAME5x
constexpr unsigned int MainTaskStackWords = 1800;			// on the SAME70 we use matrices of doubles
#elif defined(__LPC17xx__)
constexpr unsigned int MainTaskStackWords = 1110-(16*9);	// LPC builds only support 16 calibration points, so less space needed
#else
constexpr unsigned int MainTaskStackWords = 1110;			// on other processors we use matrixes of floats
#endif

static Task<MainTaskStackWords> mainTask;
extern "C" [[noreturn]] void MainTask(void * pvParameters) noexcept;

// Idle task data
constexpr unsigned int IdleTaskStackWords = 40;				// currently we don't use the idle talk for anything, so this can be quite small
static Task<IdleTaskStackWords> idleTask;

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) noexcept
{
	*ppxIdleTaskTCBBuffer = idleTask.GetTaskMemory();
	*ppxIdleTaskStackBuffer = idleTask.GetStackBase();
	*pulIdleTaskStackSize = idleTask.GetStackSize();
}

#if configUSE_TIMERS

// Timer task data
constexpr unsigned int TimerTaskStackWords = 60;
static Task<TimerTaskStackWords> timerTask;

extern "C" void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) noexcept
{
    *ppxTimerTaskTCBBuffer = timerTask.GetTaskMemory();
    *ppxTimerTaskStackBuffer = timerTask.GetStackBase();
    *pulTimerTaskStackSize = timerTask.GetStackSize();
}

#endif

// Mutexes
static Mutex i2cMutex;
static Mutex sysDirMutex;
static Mutex mallocMutex;

// We need to make malloc/free thread safe. We must use a recursive mutex for it.
extern "C" void __malloc_lock (struct _reent *_r) noexcept
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't take mutex if scheduler not started or suspended
	{
		mallocMutex.Take();
	}
}

extern "C" void __malloc_unlock (struct _reent *_r) noexcept
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't release mutex if scheduler not started or suspended
	{
		mallocMutex.Release();
	}
}

// Application entry point
extern "C" [[noreturn]] void AppMain() noexcept
{
	pinMode(DiagPin, (DiagOnPolarity) ? OUTPUT_LOW : OUTPUT_HIGH);	// set up diag LED for debugging and turn it off

#if SAME5x
	// delayMicroseconds in CoreN2G uses systick to get accurate delays
	// Temporarily set up systick so that delayMicroseconds works
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
#endif

#if !defined(DEBUG) && !defined(__LPC17xx__)	// don't check the CRC of a debug build because debugger breakpoints mess up the CRC
	// Check the integrity of the firmware by checking the firmware CRC
	{
		const char *firmwareStart = reinterpret_cast<const char*>(SCB->VTOR & 0xFFFFFF80);
		CRC32 crc;
		crc.Update(firmwareStart, (const char*)&_firmware_crc - firmwareStart);
		if (crc.Get() != _firmware_crc)
		{
			// CRC failed so flash the diagnostic LED 3 times, pause and repeat. This is the same error code used by the Duet 3 expansion boards bootloader.
			for (unsigned int i = 0; ; ++i)
			{
				const bool on = (i & 1) == 0 && (i & 15) < 6;				// turn LED on if count is 0, 2, 4 or 16, 18, 20 etc. otherwise turn it off
				digitalWrite(DiagPin, XNor(on, DiagOnPolarity));
				for (unsigned int j = 0; j < 500; ++j)
				{
					delayMicroseconds(1000);								// delayMicroseconds only works with low values of delay so do 1ms at a time
				}
			}
		}
	}
#endif	// !defined(DEBUG) && !defined(__LPC17xx__)

	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char* heapend = sbrk(0);
	register const char * stack_ptr asm ("sp");
	while (heapend + 16 < stack_ptr)
	{
		*heapend++ = memPattern;
	}

#if SAME5x
	{
		const uint32_t bootloaderSize = SCB->VTOR & 0xFFFFFF80;
		if (bootloaderSize == 0x4000)
		{
			// Looks like this is release firmware that was loaded by a bootloader in the first 16Kb of flash
			// Check that the bootloader is protected and EEPROM is configured
			uint64_t nvmUserRow0 = *reinterpret_cast<const uint64_t*>(NVMCTRL_USER);						// we only need values in the first 64 bits of the user area
			constexpr uint64_t mask =     ((uint64_t)0x0F << 32) | ((uint64_t)0x07 << 36) | (0x0F << 26);	// we just want NVM_BOOT (bits 26-29), SEE.SBLK (bits 32-35) and SEE.PSZ (bits 36:38)
			constexpr uint64_t reqValue = ((uint64_t)0x01 << 32) | ((uint64_t)0x03 << 36) | (13 << 26);		// 4K SMART EEPROM and 16K bootloader (SBLK=1 PSZ=3 BOOTPROT=13)

			if ((nvmUserRow0 & mask) != reqValue)
			{
				nvmUserRow0 = (nvmUserRow0 & ~mask) | reqValue;												// set up the required value
				_user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), 0, reinterpret_cast<const uint8_t*>(&nvmUserRow0), sizeof(nvmUserRow0));

				// If we reset immediately then the user area write doesn't complete and the bits get set to all 1s.
				delayMicroseconds(10000);
				Reset();
			}
		}
	}

	CoreInit();
	DeviceInit();
#endif

	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

#if !defined(__LPC17xx__) && !SAME5x
	// When doing a software reset, we disable the NRST input (User reset) to prevent the negative-going pulse that gets generated on it being held
	// in the capacitor and changing the reset reason from Software to User. So enable it again here. We hope that the reset signal will have gone away by now.
# ifndef RSTC_MR_KEY_PASSWD
// Definition of RSTC_MR_KEY_PASSWD is missing in the SAM3X ASF files
#  define RSTC_MR_KEY_PASSWD (0xA5u << 24)
# endif
	RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD | RSTC_MR_URSTEN;
#endif

	Cache::Init();					// initialise the cache and/or the MPU, if applicable to this processor
	Cache::Enable();

#if SAM4S
	efc_enable_cloe(EFC0);			// enable code loop optimisation
#elif SAM4E || SAME70
	efc_enable_cloe(EFC);			// enable code loop optimisation
#endif

	idleTask.AddToList();			// add the FreeRTOS internal tasks to the task list

#if configUSE_TIMERS
	timerTask.AddToList();
#endif

	// Create the startup task
	mainTask.Create(MainTask, "MAIN", nullptr, TaskPriority::SpinPriority);
	vTaskStartScheduler();			// doesn't return
	for (;;) { }					// keep gcc happy
}

extern "C" [[noreturn]] void MainTask(void *pvParameters) noexcept
{
	mallocMutex.Create("Malloc");
	i2cMutex.Create("I2C");
	sysDirMutex.Create("SysDir");

	reprap.Init();
	for (;;)
	{
		reprap.Spin();
	}
}

#ifdef __LPC17xx__
	extern "C" size_t xPortGetTotalHeapSize( void );
#endif

static void GetHandlerStackUsage(uint32_t* maxStack, uint32_t* neverUsed) noexcept
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

uint32_t Tasks::GetNeverUsedRam() noexcept
{
	uint32_t neverUsedRam;

	GetHandlerStackUsage(nullptr, &neverUsedRam);
	return neverUsedRam;
}

// Write data about the current task
void Tasks::Diagnostics(MessageType mtype) noexcept
{
	Platform& p = reprap.GetPlatform();
	p.Message(mtype, "=== RTOS ===\n");
	// Print memory stats
	{
		const char * const ramstart =
#if SAME70
			(char *) 0x20400000;
#elif SAM4E || SAM4S || SAME5x
			(char *) 0x20000000;
#elif SAM3XA
			(char *) 0x20070000;
#elif defined(__LPC17xx__)
			(char *) 0x10000000;
#else
# error Unsupported processor
#endif
		p.MessageF(mtype, "Static ram: %d\n", &_end - ramstart);

#ifdef __LPC17xx__
		p.MessageF(mtype, "Dynamic Memory (RTOS Heap 5): %d free, %d never used\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize() );
#else
		const struct mallinfo mi = mallinfo();
		p.MessageF(mtype, "Dynamic ram: %d of which %d recycled\n", mi.uordblks, mi.fordblks);
#endif
		uint32_t maxStack, neverUsed;
		GetHandlerStackUsage(&maxStack, &neverUsed);
		p.MessageF(mtype, "Exception stack ram used: %" PRIu32 "\n", maxStack);
		p.MessageF(mtype, "Never used ram: %" PRIu32 "\n", neverUsed);

	}	// end memory stats scope

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
}

const Mutex *Tasks::GetI2CMutex() noexcept
{
	return &i2cMutex;
}

const Mutex *Tasks::GetSysDirMutex() noexcept
{
	return &sysDirMutex;
}

// Exception handlers
extern "C"
{
	// This intercepts the 1ms system tick
	void vApplicationTickHook() noexcept
	{
		CoreSysTick();
		reprap.Tick();
	}

	// Exception handlers
	// By default the Usage Fault, Bus Fault and Memory Management fault handlers are not enabled,
	// so they escalate to a Hard Fault and we don't need to provide separate exception handlers for them.
	[[noreturn]] void hardFaultDispatcher(const uint32_t *pulFaultStackAddress) noexcept
	{
	    reprap.SoftwareReset((uint16_t)SoftwareResetReason::hardFault, pulFaultStackAddress + 5);
	}

	// The fault handler implementation calls a function called hardFaultDispatcher()
    [[noreturn]] void HardFault_Handler() noexcept __attribute__((naked));
	void HardFault_Handler() noexcept
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

#if USE_MPU

	[[noreturn]] void memManageDispatcher(const uint32_t *pulFaultStackAddress) noexcept
	{
	    reprap.SoftwareReset((uint16_t)SoftwareResetReason::memFault, pulFaultStackAddress + 5);
	}

	// The fault handler implementation calls a function called memManageDispatcher()
	[[noreturn]] void MemManage_Handler() noexcept __attribute__((naked));
	void MemManage_Handler() noexcept
	{
	    __asm volatile
	    (
	        " tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
	        " ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
	        " mrseq r0, msp                                             \n"
	        " mrsne r0, psp                                             \n"
	        " ldr r2, handler_mf_address_const                          \n"
	        " bx r2                                                     \n"
	        " handler_mf_address_const: .word memManageDispatcher       \n"
	    );
	}

#endif

	[[noreturn]] void wdtFaultDispatcher(const uint32_t *pulFaultStackAddress) noexcept
	{
	    reprap.SoftwareReset((uint16_t)SoftwareResetReason::wdtFault, pulFaultStackAddress + 5);
	}

#ifdef __LPC17xx__
	[[noreturn]] void WDT_IRQHandler() noexcept __attribute__((naked));
    void WDT_IRQHandler() noexcept
    {
    	LPC_WDT->MOD &=~((uint32_t)(1<<2)); //SD::clear timout flag before resetting to prevent the Smoothie bootloader going into DFU mode
#else
    [[noreturn]] void WDT_Handler() noexcept __attribute__((naked));
	void WDT_Handler() noexcept
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

	[[noreturn]] void otherFaultDispatcher(const uint32_t *pulFaultStackAddress) noexcept
	{
	    reprap.SoftwareReset((uint16_t)SoftwareResetReason::otherFault, pulFaultStackAddress + 5);
	}

	// 2017-05-25: A user is getting 'otherFault' reports, so now we do a stack dump for those too.
	// The fault handler implementation calls a function called otherFaultDispatcher()
	[[noreturn]] void OtherFault_Handler() noexcept __attribute__((naked));
	void OtherFault_Handler() noexcept
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
	[[noreturn]] void NMI_Handler        () noexcept { reprap.SoftwareReset((uint16_t)SoftwareResetReason::NMI); }
	[[noreturn]] void UsageFault_Handler () noexcept { reprap.SoftwareReset((uint16_t)SoftwareResetReason::usageFault); }

	[[noreturn]] void DebugMon_Handler   () noexcept __attribute__ ((alias("OtherFault_Handler")));

	// FreeRTOS hooks that we need to provide
	[[noreturn]] void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress, char* pcTaskName) noexcept
	{
		reprap.SoftwareReset((uint16_t)SoftwareResetReason::stackOverflow, pulFaultStackAddress);
	}

	[[noreturn]] void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) noexcept __attribute((naked));
	void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) noexcept
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

	[[noreturn]] void assertCalledDispatcher(const uint32_t *pulFaultStackAddress) noexcept
	{
	    reprap.SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
	}

	[[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept __attribute((naked));
	void vAssertCalled(uint32_t line, const char *file) noexcept
	{
#if false
		debugPrintf("ASSERTION FAILED IN %s on LINE %d\n", file, line);
		SERIAL_MAIN_DEVICE.flush();
#endif
	    __asm volatile
	    (
	    	" push {r0, r1, lr}											\n"		/* save parameters and call address */
	    	" mov r0, sp												\n"
	        " ldr r2, handler_asrt_address_const                        \n"
	        " bx r2                                                     \n"
	        " handler_asrt_address_const: .word assertCalledDispatcher  \n"
	    );
	}

#ifdef __LPC17xx__
	[[noreturn]] void applicationMallocFailedCalledDispatcher(const uint32_t *pulFaultStackAddress) noexcept
	{
		reprap.SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
	}

	[[noreturn]] void vApplicationMallocFailedHook() noexcept __attribute((naked));
	void vApplicationMallocFailedHook() noexcept
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
	[[noreturn]] void __throw_bad_function_call() noexcept { vAssertCalled(__LINE__, __FILE__); }
}

// The default terminate handler pulls in sprintf and lots of other functions, which makes the binary too large. So we replace it.
[[noreturn]] void Terminate() noexcept
{
	register const uint32_t * stack_ptr asm ("sp");
	reprap.SoftwareReset((uint16_t)SoftwareResetReason::terminateCalled, stack_ptr);
}

namespace __cxxabiv1
{
	std::terminate_handler __terminate_handler = Terminate;
}

extern "C" [[noreturn]] void __cxa_pure_virtual() noexcept
{
	register const uint32_t * stack_ptr asm ("sp");
	reprap.SoftwareReset((uint16_t)SoftwareResetReason::pureVirtual, stack_ptr);
}

extern "C" [[noreturn]] void __cxa_deleted_virtual() noexcept
{
	register const uint32_t * stack_ptr asm ("sp");
	reprap.SoftwareReset((uint16_t)SoftwareResetReason::deletedVirtual, stack_ptr);
}

// We don't need the time zone functionality. Declaring these saves 8Kb.
extern "C" void __tzset() noexcept { }
extern "C" void __tz_lock() noexcept { }
extern "C" void __tz_unlock() noexcept { }
extern "C" void _tzset_unlocked() noexcept { }

// End
