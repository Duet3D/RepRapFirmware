/*
 * Startup.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#include "Tasks.h"
#include "RepRap.h"
#include "Platform.h"
#include <malloc.h>

#include "FreeRTOS.h"
#include "task.h"

#if USE_CACHE
# include "cmcc/cmcc.h"
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

// Calculate the CRC32 of a block of memory
uint32_t CalcCrc32(const uint8_t *p, size_t size)
{
	// Lookup table for calculating the CRC32 of the firmware. We could generate this at runtime, but flash memory is more plentiful than RAM.
	static const uint32_t Crc32Table[256] =
	{
		0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
		0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
		0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
		0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
		0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
		0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
		0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
		0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
		0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
		0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
		0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
		0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
		0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
		0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
		0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
		0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
		0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
		0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
		0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
		0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
		0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
		0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
		0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
		0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
		0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
		0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
		0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
		0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
		0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
		0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
		0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
		0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
		0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
		0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
		0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
		0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
		0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
		0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
		0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
		0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
		0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
		0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
		0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
	};

	uint32_t crc = ~0u;
	while (size != 0)
	{
		--size;
		crc = Crc32Table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
	}
	return ~crc;
}

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

	pinMode(DiagPin, OUTPUT_LOW);				// set up diag LED for debugging and turn it off

	// Check the integrity of the firmware by checking the firmware CRC
	{
#ifdef IFLASH_ADDR
		const uint8_t *firmwareStart = reinterpret_cast<const uint8_t *>(IFLASH_ADDR);
#else
		const uint8_t *firmwareStart = reinterpret_cast<const uint8_t *>(IFLASH0_ADDR);
#endif
		const uint32_t crc = CalcCrc32(firmwareStart, (const uint8_t*)&_firmware_crc - firmwareStart);
		if (crc != _firmware_crc)
		{
			// CRC failed so flash the diagnostic LED 3 times, pause and repeat. This is the same error code used by the Duet 3 expansion boards bootloader.
			for (unsigned int i = 0; ; ++i)
			{
				digitalWrite(DiagPin, (i & 1) == 0 && (i & 15) < 6);		// turn LED on if count is 0, 2, 4 or 16, 18, 20 etc. otherwise turn it off
				for (unsigned int j = 0; j < 1500; ++j)
				{
					delayMicroseconds(100);									// delayMicroseconds only works with low values of delay so do 100us at a time
				}
			}
		}
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
	RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD | RSTC_MR_URSTEN;
#endif

#if USE_CACHE
	// Enable the cache
	cmcc_config g_cmcc_cfg;
	cmcc_get_config_defaults(&g_cmcc_cfg);
	cmcc_init(CMCC, &g_cmcc_cfg);
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
