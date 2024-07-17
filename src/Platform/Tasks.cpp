/*
 * Startup.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#include "Tasks.h"
#include "RepRap.h"
#include "Platform.h"
#include <Cache.h>
#include <Platform/TaskPriorities.h>
#include <Hardware/SoftwareReset.h>
#include <Hardware/NonVolatileMemory.h>
#include <Storage/CRC32.h>
#include <Movement/StepTimer.h>

#if SAM4E || SAM4S || SAME70
# include <efc/efc.h>		// for efc_enable_cloe()
#endif

#if SAME5x
# include <hpl_user_area.h>
#endif

#include <FreeRTOS.h>
#include <task.h>
#include <freertos_task_additions.h>
#include <malloc.h>

const uint8_t memPattern = 0xA5;		// this must be the same pattern as FreeRTOS because we use common code for checking for stack overflow

// Define replacement standard library functions
#include <syscalls.h>

// MAIN task data
// The main task currently runs GCodes, so it needs to be large enough to hold the matrices used for delta auto calibration.
// The worst case stack usage points are as follows:
// 1. After running delta auto calibration with Move debugging enabled
// 2. We create an array of (2 * MaxAxes^2) floats when inverting the movement matrix for Core kinematics.
#if SAME70
// On the SAME70 we use matrices of doubles when doing auto calibration, so we need 1800 words of stack even when MaxAxes is only 15
constexpr unsigned int MainTaskStackWords = max<unsigned int>(1800, (MaxAxes * MaxAxes * 2) + 550);
#else
// On other processors we use matrices of floats when doing auto calibration
// Increase minimum stack words to 1370 for WPA Enterprise support
constexpr unsigned int MainTaskStackWords = max<unsigned int>(1370, (MaxAxes * MaxAxes * 2) + 550);
#endif

static Task<MainTaskStackWords> mainTask;
extern "C" [[noreturn]] void MainTask(void * pvParameters) noexcept;
extern DeviceVectors exception_table;

// Idle task data
// The timer and idle tasks currently never do I/O, so they don't need much stack.
constexpr unsigned int IdleTaskStackWords = 50;				// currently we don't use the idle task for anything, so this can be quite small
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
static Mutex mallocMutex;

// We need to make malloc/free thread safe. We must use a recursive mutex for it.
extern "C" void GetMallocMutex() noexcept
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't take mutex if scheduler not started or suspended
	{
		mallocMutex.Take();
	}
}

extern "C" void ReleaseMallocMutex() noexcept
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't release mutex if scheduler not started or suspended
	{
		mallocMutex.Release();
	}
}

// Get a 4-byte aligned NonVolatileMemory buffer suitable for the crash handler to use for reading/writing flash memory.
// We don't want to use a static buffer because that is wasteful of RAM, given that only the crash handler uses it, we have interrupts disabled while we use it,
// and we reset immediately afterwards.
// Instead we use either the bottom or top of the main task stack.
// Parameter 'stk' is the stack we are interested in, which we must not overwrite; or null.
// If it is not null then the caller is either using the same stack a little lower, or the exception stack.
void *Tasks::GetNVMBuffer(const uint32_t *_ecv_array null stk) noexcept
{
	constexpr size_t stackAllowance = 128;
	static_assert((sizeof(NonVolatileMemory) & 3) == 0);
	static_assert(MainTaskStackWords * 4 >= 2 * sizeof(NonVolatileMemory) + stackAllowance + 4);
	const char * const cStack = reinterpret_cast<const char*>((stk == nullptr) ? GetStackPointer() : stk);

	// See if we can use the bottom of the main task stack
	char *ret = (char *)&mainTask + sizeof(TaskBase);
	if (cStack > ret + (sizeof(NonVolatileMemory) + stackAllowance + 4))	// allow space for the buffer + 128b in case we are on that stack
	{
		ret += 4;															// the +4 is so that we leave the stack marker alone in case the main task raised the exception
	}
	else
	{
		ret += (MainTaskStackWords * 4) - sizeof(NonVolatileMemory);		// use the top area instead
	}
	return ret;
}

// Application entry point
[[noreturn]] void AppMain() noexcept
{
#if defined(DUET3_MB6HC)													// for MB6HC the Status and Activity pins and polarity depend on the board version
	const BoardType bt = Platform::GetMB6HCBoardType();
	const Pin DiagPin = (bt >= BoardType::Duet3_6HC_v102) ? DiagPin102 : DiagPinPre102;
	const Pin ActLedPin = (bt >= BoardType::Duet3_6HC_v102) ? ActLedPin102 : ActLedPinPre102;
	const bool DiagOnPolarity = (bt >= BoardType::Duet3_6HC_v102) ? DiagOnPolarity102 : DiagOnPolarityPre102;
	if (bt >= BoardType::Duet3_6HC_v102)
	{
		pinMode(UsbPowerSwitchPin, OUTPUT_LOW);								// turn USB power off
		pinMode(UsbModePin, OUTPUT_LOW);									// USB mode = device/UFP
	}
#endif
#if defined(DUET3_MB6XD)
	const BoardType bt = Platform::GetMB6XDBoardType();
	if (bt >= BoardType::Duet3_6XD_v101)
	{
		pinMode(UsbPowerSwitchPin, OUTPUT_LOW);								// turn USB power off
		pinMode(UsbModePin, OUTPUT_LOW);									// USB mode = device/UFP
	}
#endif
	pinMode(DiagPin, (DiagOnPolarity) ? OUTPUT_LOW : OUTPUT_HIGH);			// set up status LED for debugging and turn it off
#if defined(DUET3MINI) || defined(DUET3_MB6HC) || defined(DUET3_MB6XD)
	pinMode(ActLedPin, (ActOnPolarity) ? OUTPUT_LOW : OUTPUT_HIGH);			// set up activity LED and turn it off
#endif

#if !defined(DEBUG)		// don't check the CRC of a debug build because debugger breakpoints mess up the CRC
	// Check the integrity of the firmware by checking the firmware CRC
	// If we have embedded files then the CRC is stored after those files, so we need to fetch the CRC address form the vector table
	{
		const char *firmwareStart = reinterpret_cast<const char*>(SCB->VTOR & 0xFFFFFF80);
		const char *firmwareCrcAddr = (const char*)exception_table
# if SAME5x
										.pvReservedM9;
# else
										.pfnReserved1_Handler;
# endif
		CRC32 crc;
		crc.Update(firmwareStart, firmwareCrcAddr - firmwareStart);
		if (crc.Get() != *((const uint32_t*)firmwareCrcAddr))
		{
			// CRC failed so flash the diagnostic LED 3 times, pause and repeat. This is the same error code used by the Duet 3 expansion boards bootloader.
			for (unsigned int i = 0; ; ++i)
			{
				const bool on = (i & 1) == 0 && (i & 15) < 6;				// turn LED on if count is 0, 2, 4 or 16, 18, 20 etc. otherwise turn it off
				digitalWrite(DiagPin, XNor(on, DiagOnPolarity));
				for (unsigned int j = 0; j < 250; ++j)
				{
					delayMicroseconds(1000);								// delayMicroseconds only works with low values of delay so do 1ms at a time
				}
			}
		}
	}
#endif	// !defined(DEBUG)

	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char *_ecv_array heapend = heapTop;
	const char *_ecv_array stack_ptr = (const char*_ecv_array)GetStackPointer();
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
				ResetProcessor();
			}
		}
	}
#endif

	CoreInit();
	DeviceInit();

	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

#if !SAME5x
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

	// Create the mutexes and the startup task
	mallocMutex.Create("Malloc");
	i2cMutex.Create("I2C");
	mainTask.Create(MainTask, "MAIN", nullptr, TaskPriority::SpinPriority);

	StepTimer::Init();				// initialise the step pulse timer now because we use it for measuring task CPU usage
	vTaskStartScheduler();			// doesn't return
	for (;;) { }					// keep gcc happy
}

extern "C" [[noreturn]] void MainTask(void *pvParameters) noexcept
{
	reprap.Init();
	for (;;)
	{
		reprap.Spin();
	}
}

// Return the amount of free handler stack space. It may be negative if the stack has overflowed into the area reserved for the heap.
static ptrdiff_t GetHandlerFreeStack() noexcept
{
	const char * const ramend = (const char*)&_estack;
	const char * stack_lwm = sysStackLimit;
	while (stack_lwm < ramend && *stack_lwm == memPattern)
	{
		++stack_lwm;
	}
	return stack_lwm - sysStackLimit;
}

ptrdiff_t Tasks::GetNeverUsedRam() noexcept
{
	return heapLimit - heapTop;
}

const char* Tasks::GetHeapTop() noexcept
{
	return heapTop;
}

// Allocate memory permanently. Using this saves about 8 bytes per object. You must not call free() on the returned object.
// It doesn't try to allocate from the free list maintained by malloc, only from virgin memory.
void *Tasks::AllocPermanent(size_t sz, std::align_val_t align) noexcept
{
	GetMallocMutex();
	void * const ret = CoreAllocPermanent(sz, align);
	ReleaseMallocMutex();
	return ret;
}

// Function called by FreeRTOS to get the total number of timer ticks since last reset
// We use a 64-bit value because a 32-bit value wraps after about 95 minutes on Duet 3, a little less on Duet 2.
// This gets called fairly often, so when the 32-bit tick counter wraps round we assume it has only wrapped once.
extern "C" uint64_t TaskGetRunTimeTicks() noexcept
{
	static uint32_t msw = 0;
	static uint32_t ticksAtLastCall = 0;

	const uint32_t ticks = StepTimer::GetTimerTicks();
	if (ticks < ticksAtLastCall) { ++msw; }
	ticksAtLastCall = ticks;
	return ((uint64_t)msw << 32) | ticks;
}

// Function called by FreeRTOS and internally to reset the run-time counter and return the number of timer ticks since it was last reset
extern "C" uint64_t TaskResetRunTimeCounter() noexcept
{
	static uint64_t whenRunTimeCounterLastReset = 0;

	const uint64_t now = TaskGetRunTimeTicks();
	const uint64_t ret = now - whenRunTimeCounterLastReset;
	whenRunTimeCounterLastReset = now;
	return ret;
}

// Write data about the current task
void Tasks::Diagnostics(MessageType mtype) noexcept
{
	Platform& p = reprap.GetPlatform();
	p.Message(mtype, "=== RTOS ===\n");
	// Print memory stats
	{
		const char * const ramstart =
#if SAME5x
			(char *) HSRAM_ADDR;
#else
			(char *) IRAM_ADDR;
#endif
		p.MessageF(mtype, "Static ram: %d\n", &_end - ramstart);

		const struct mallinfo mi = mallinfo();
		p.MessageF(mtype, "Dynamic ram: %d of which %d recycled\n", mi.uordblks, mi.fordblks);
		p.MessageF(mtype, "Never used RAM %d, free system stack %d words\n", GetNeverUsedRam(), GetHandlerFreeStack()/4);

		//DEBUG
		//p.MessageF(mtype, "heap top %.08" PRIx32 ", limit %.08" PRIx32 "\n", (uint32_t)heapTop, (uint32_t)heapLimit);
		//ENDDB
	}	// end memory stats scope

	const uint64_t timeSinceLastCall = TaskResetRunTimeCounter();
	float totalCpuPercent = 0.0;
	p.Message(mtype, "Tasks:");
	for (TaskBase *t = TaskBase::GetTaskList(); t != nullptr; t = t->GetNext())
	{
		ExtendedTaskStatus_t taskDetails;
		vTaskGetExtendedInfo(t->GetFreeRTOSHandle(), &taskDetails);

		const char* stateText;
		switch (taskDetails.eCurrentState)
		{
		case esRunning:
			stateText = "running";
			break;
		case esReady:
			stateText = "ready";
			break;
		case esNotifyWaiting:
			stateText = "nWait";
			break;
		case esResourceWaiting:
			stateText = "rWait:";
			break;
		case esDelaying:
			stateText = "delaying";
			break;
		case esSuspended:
			stateText = "suspended";
			break;
		case esBlocked:
			stateText = "blocked";
			break;
		default:
			stateText = "invalid";
			break;
		}

		const float cpuPercent = (100 * (float)taskDetails.ulRunTimeCounter)/(float)timeSinceLastCall;
		totalCpuPercent += cpuPercent;
		String<StringLength50> str;
		str.printf(" %s(%u,%s", taskDetails.pcTaskName, (unsigned int)taskDetails.uxCurrentPriority, stateText);
		switch (taskDetails.eCurrentState)
		{
		case esResourceWaiting:
			{
				const Mutex *m = Mutex::GetMutexList();
				while (m != nullptr)
				{
					if ((const void *)m == taskDetails.pvResource)
					{
						str.catf(" %s", m->GetName());
						break;
					}
					m = m->GetNext();
				}
			}
			break;

		case esNotifyWaiting:
			str.catf(" %" PRIu32, taskDetails.notifyIndex);
			break;

		default:
			break;
		}
		str.catf(",%.1f%%,%u)", (double)cpuPercent, (unsigned int)taskDetails.usStackHighWaterMark);
		p.Message(mtype, str.c_str());
	}
	p.MessageF(mtype, ", total %.1f%%\nOwned mutexes:", (double)totalCpuPercent);

	for (const Mutex *m = Mutex::GetMutexList(); m != nullptr; m = m->GetNext())
	{
		const TaskHandle holder = m->GetHolder();
		if (holder != nullptr)
		{
			p.MessageF(mtype, " %s(%s)", m->GetName(), pcTaskGetName(holder->GetFreeRTOSHandle()));
		}
	}
	p.Message(mtype, "\n");
}

TaskHandle Tasks::GetMainTask() noexcept
{
	return &mainTask;
}

void Tasks::TerminateMainTask() noexcept
{
	mainTask.TerminateAndUnlink();
}

Mutex *Tasks::GetI2CMutex() noexcept
{
	return &i2cMutex;
}

// This intercepts the 1ms system tick
extern "C" void vApplicationTickHook() noexcept
{
	CoreSysTick();
	reprap.Tick();
}

// We don't need the time zone functionality. Declaring these saves 8Kb.
extern "C" void __tzset() noexcept { }
extern "C" void __tz_lock() noexcept { }
extern "C" void __tz_unlock() noexcept { }
extern "C" void _tzset_unlocked() noexcept { }

#if SUPPORT_CAN_EXPANSION

// Functions called by CanMessageBuffer in CANlib
void *MessageBufferAlloc(size_t sz, std::align_val_t align) noexcept
{
	return Tasks::AllocPermanent(sz, align);
}

void MessageBufferDelete(void *ptr, std::align_val_t align) noexcept { }

#endif

// End
