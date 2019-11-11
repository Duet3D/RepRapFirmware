/*
 * Startup.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#include "Tasks.h"
#include "RepRap.h"
#include "Platform.h"
#include "Storage/CRC32.h"

#include <malloc.h>

#include "FreeRTOS.h"
#include "task.h"
#include <TaskPriorities.h>

#if USE_CACHE

# if SAM4E
#  include "cmcc/cmcc.h"
# endif

# if SAME70
#  include <core_cm7.h>

void EnableCache()
{
	SCB_EnableICache();
	SCB_EnableDCache();
}

void DisableCache()
{
	SCB_DisableICache();
	SCB_DisableDCache();
}

# endif

#endif

#if SAME70 && USE_MPU
# include <mpu_armv7.h>

// Macro ARM_MPU_RASR_EX is incorrectly defined in CMSIS 5.4.0, see https://github.com/ARM-software/CMSIS_5/releases. Redefine it here.

# undef ARM_MPU_RASR_EX

/**
* MPU Region Attribute and Size Register Value
*
* \param DisableExec       Instruction access disable bit, 1= disable instruction fetches.
* \param AccessPermission  Data access permissions, allows you to configure read/write access for User and Privileged mode.
* \param AccessAttributes  Memory access attribution, see \ref ARM_MPU_ACCESS_.
* \param SubRegionDisable  Sub-region disable field.
* \param Size              Region size of the region to be configured, for example 4K, 8K.
*/
# define ARM_MPU_RASR_EX(DisableExec, AccessPermission, AccessAttributes, SubRegionDisable, Size)    \
  ((((DisableExec)      << MPU_RASR_XN_Pos)   & MPU_RASR_XN_Msk)                                  | \
   (((AccessPermission) << MPU_RASR_AP_Pos)   & MPU_RASR_AP_Msk)                                  | \
   (((AccessAttributes) & (MPU_RASR_TEX_Msk | MPU_RASR_S_Msk | MPU_RASR_C_Msk | MPU_RASR_B_Msk))) | \
   (((SubRegionDisable) << MPU_RASR_SRD_Pos)  & MPU_RASR_SRD_Msk)                                 | \
   (((Size)             << MPU_RASR_SIZE_Pos) & MPU_RASR_SIZE_Msk)                                | \
   (((MPU_RASR_ENABLE_Msk))))

#endif

const uint8_t memPattern = 0xA5;

extern "C" char *sbrk(int i);
extern char _end;						// defined in linker script
extern uint32_t _firmware_crc;			// defined in linker script
extern uint32_t _estack;				// defined in linker script

// MAIN task data
// The main task currently runs GCodes, so it needs to be large enough to hold the matrices used for delta auto calibration.
// The timer and idle tasks currently never do I/O, so they can be much smaller.
#if defined(LPC_NETWORKING)
constexpr unsigned int MainTaskStackWords = 1600-424;
#else
constexpr unsigned int MainTaskStackWords = 1600;
#endif

static Task<MainTaskStackWords> mainTask;
extern "C" [[noreturn]] void MainTask(void * pvParameters);

// Idle task data
constexpr unsigned int IdleTaskStackWords = 60;
static Task<IdleTaskStackWords> idleTask;

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = idleTask.GetTaskMemory();
	*ppxIdleTaskStackBuffer = idleTask.GetStackBase();
	*pulIdleTaskStackSize = idleTask.GetStackSize();
}

#if configUSE_TIMERS

// Timer task data
constexpr unsigned int TimerTaskStackWords = 60;
static Task<TimerTaskStackWords> timerTask;

extern "C" void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = timerTask.GetTaskMemory();
    *ppxTimerTaskStackBuffer = timerTask.GetStackBase();
    *pulTimerTaskStackSize = timerTask.GetStackSize();
}

#endif

// Mutexes
static Mutex spiMutex;
static Mutex i2cMutex;
static Mutex sysDirMutex;
static Mutex mallocMutex;

// We need to make malloc/free thread safe. We must use a recursive mutex for it.
extern "C" void __malloc_lock (struct _reent *_r)
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't take mutex if scheduler not started or suspended
	{
		mallocMutex.Take();
	}
}

extern "C" void __malloc_unlock (struct _reent *_r)
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't release mutex if scheduler not started or suspended
	{
		mallocMutex.Release();
	}
}

// Application entry point
extern "C" [[noreturn]] void AppMain()
{
	pinMode(DiagPin, OUTPUT_LOW);				// set up diag LED for debugging and turn it off

	// Check the integrity of the firmware by checking the firmware CRC
	{
#ifdef IFLASH_ADDR
		const char *firmwareStart = reinterpret_cast<const char *>(IFLASH_ADDR);
#else
		const char *firmwareStart = reinterpret_cast<const char *>(IFLASH0_ADDR);
#endif
		CRC32 crc;
		crc.Update(firmwareStart, (const char*)&_firmware_crc - firmwareStart);
		if (crc.Get() != _firmware_crc)
		{
			// CRC failed so flash the diagnostic LED 3 times, pause and repeat. This is the same error code used by the Duet 3 expansion boards bootloader.
			for (unsigned int i = 0; ; ++i)
			{
				digitalWrite(DiagPin, (i & 1) == 0 && (i & 15) < 6);		// turn LED on if count is 0, 2, 4 or 16, 18, 20 etc. otherwise turn it off
				for (unsigned int j = 0; j < 5000; ++j)
				{
					delayMicroseconds(100);									// delayMicroseconds only works with low values of delay so do 100us at a time
				}
			}
		}
	}

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

#if SAME70 && USE_MPU

	// Set up the MPU so that we can have a non-cacheable RAM region, and so that we can trap accesses to non-existent memory
	// Where regions overlap, the region with the highest region number takes priority
	constexpr ARM_MPU_Region_t regionTable[] =
	{
		// Flash memory: read-only, execute allowed, cacheable
		{
			ARM_MPU_RBAR(0, IFLASH_ADDR),
			ARM_MPU_RASR_EX(0u, ARM_MPU_AP_RO, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_1MB)
		},
		// First 256kb RAM, read-write, cacheable, execute disabled. Parts of this are is overridden later.
		{
			ARM_MPU_RBAR(1, IRAM_ADDR),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_256KB)
		},
		// Final 128kb RAM, read-write, cacheable, execute disabled
		{
			ARM_MPU_RBAR(2, IRAM_ADDR + 0x00040000),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_128KB)
		},
		// Non-cachable RAM. This must be before normal RAM because it includes CAN buffers which must be within first 64kb.
		// Read write, execute disabled, non-cacheable
		{
			ARM_MPU_RBAR(3, IRAM_ADDR),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_ORDERED, 0, ARM_MPU_REGION_SIZE_64KB)
		},
		// RAMFUNC memory. Read-only (the code has already been written to it), execution allowed. The initialised data memory follows, so it must be RW.
		// 256 bytes is enough at present (check the linker memory map if adding more RAMFUNCs).
		{
			ARM_MPU_RBAR(4, IRAM_ADDR + 0x00010000),
			ARM_MPU_RASR_EX(0u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_256B)
		},
		// Peripherals
		{
			ARM_MPU_RBAR(5, 0x40000000),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_DEVICE(1u), 0u, ARM_MPU_REGION_SIZE_16MB)
		},
		// USBHS
		{
			ARM_MPU_RBAR(6, 0xA0100000),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_DEVICE(1u), 0u, ARM_MPU_REGION_SIZE_1MB)
		},
		// ROM
		{
			ARM_MPU_RBAR(7, IROM_ADDR),
			ARM_MPU_RASR_EX(0u, ARM_MPU_AP_RO, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_4MB)
		},
		// ARM Private Peripheral Bus
		{
			ARM_MPU_RBAR(8, 0xE0000000),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_ORDERED, 0u, ARM_MPU_REGION_SIZE_1MB)
		}
	};

	// Ensure MPU is disabled
	ARM_MPU_Disable();

	// Clear all regions
	const uint32_t numRegions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
	for (unsigned int region = 0; region < numRegions; ++region)
	{
		ARM_MPU_ClrRegion(region);
	}

	// Load regions from our table
	ARM_MPU_Load(regionTable, ARRAY_SIZE(regionTable));

	// Enable the MPU, disabling the default map but allowing exception handlers to use it
	ARM_MPU_Enable(0x01);
#endif

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
	RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD | RSTC_MR_URSTEN;
#endif

#if USE_CACHE
# if SAM4E
	// Initialise the cache controller
	cmcc_config g_cmcc_cfg;
	cmcc_get_config_defaults(&g_cmcc_cfg);
	cmcc_init(CMCC, &g_cmcc_cfg);
#endif
	EnableCache();
#endif

	// Add the FreeRTOS internal tasks to the task list
	idleTask.AddToList();

#if configUSE_TIMERS
	timerTask.AddToList();
#endif

	// Create the startup task
	mainTask.Create(MainTask, "MAIN", nullptr, TaskPriority::SpinPriority);
	vTaskStartScheduler();			// doesn't return
	for (;;) { }					// kep gcc happy
}

extern "C" [[noreturn]] void MainTask(void *pvParameters)
{
	mallocMutex.Create("Malloc");
	spiMutex.Create("SPI");
	i2cMutex.Create("I2C");
	sysDirMutex.Create("SysDir");

	reprap.Init();
	for (;;)
	{
		reprap.Spin();
	}
}

#ifdef __LPC17xx__
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

	uint32_t GetNeverUsedRam()
	{
		uint32_t neverUsedRam;

		GetHandlerStackUsage(nullptr, &neverUsedRam);
		return neverUsedRam;
	}

	// Write data about the current task
	void Diagnostics(MessageType mtype)
	{
		Platform& p = reprap.GetPlatform();
		p.Message(mtype, "=== RTOS ===\n");
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
			GetHandlerStackUsage(&maxStack, &neverUsed);
			p.MessageF(mtype, "Exception stack ram used: %" PRIu32 "\n", maxStack);
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

	const Mutex *GetSpiMutex()
	{
		return &spiMutex;
	}

	const Mutex *GetI2CMutex()
	{
		return &i2cMutex;
	}

	const Mutex *GetSysDirMutex()
	{
		return &sysDirMutex;
	}
}

// Exception handlers
extern "C"
{
	// This intercepts the 1ms system tick
	void vApplicationTickHook()
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
	[[noreturn]] void hardFaultDispatcher(const uint32_t *pulFaultStackAddress)
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

#if USE_MPU

	[[noreturn]] void memManageDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::memFault, pulFaultStackAddress + 5);
	}

	// The fault handler implementation calls a function called hardFaultDispatcher()
    void MemManage_Handler() __attribute__((naked, noreturn));
	void MemManage_Handler()
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

	[[noreturn]] void wdtFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::wdtFault, pulFaultStackAddress + 5);
	}

#ifdef __LPC17xx__
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

	[[noreturn]] void otherFaultDispatcher(const uint32_t *pulFaultStackAddress)
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

	// FreeRTOS hooks that we need to provide
	[[noreturn]] void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress, char* pcTaskName)
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

	[[noreturn]] void assertCalledDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
	}

	void vAssertCalled(uint32_t line, const char *file) __attribute((naked, noreturn));
	void vAssertCalled(uint32_t line, const char *file)
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
