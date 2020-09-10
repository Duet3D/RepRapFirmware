/*
 * CrashHandlers.cpp
 *
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#include "ExceptionHandlers.h"
#include <RepRap.h>
#include <Platform.h>
#include <Hardware/NonVolatileMemory.h>
#include <Cache.h>

// Perform a software reset. 'stk' points to the exception stack (r0 r1 r2 r3 r12 lr pc xPSR) if the cause is an exception, otherwise it is nullptr.
void SoftwareReset(uint16_t reason, const uint32_t *stk) noexcept
{
	cpu_irq_disable();							// disable interrupts before we call any flash functions. We don't enable them again.
	WatchdogReset();							// kick the watchdog

#if SAM4E || SAME70
	rswdt_restart(RSWDT);						// kick the secondary watchdog
#endif

	Cache::Disable();

	if (reason == (uint16_t)SoftwareResetReason::erase)
	{
#if SAME5x
		//TODO invalidate flash so the USB bootloader runs
#else
		EraseAndReset();
#endif
 	}
	else
	{
		if (reason != (uint16_t)SoftwareResetReason::user)
		{
			if (SERIAL_MAIN_DEVICE.canWrite() == 0)
			{
				reason |= (uint16_t)SoftwareResetReason::inUsbOutput;	// if we are resetting because we are stuck in a Spin function, record whether we are trying to send to USB
			}

#if HAS_AUX_DEVICES
			if (SERIAL_AUX_DEVICE.canWrite() == 0
# ifdef SERIAL_AUX2_DEVICE
				|| SERIAL_AUX2_DEVICE.canWrite() == 0
# endif
			   )
			{
				reason |= (uint16_t)SoftwareResetReason::inAuxOutput;	// if we are resetting because we are stuck in a Spin function, record whether we are trying to send to aux
			}
#endif
		}
		reason |= (uint8_t)reprap.GetSpinningModule();
		if (reprap.GetPlatform().WasDeliberateError())
		{
			reason |= (uint16_t)SoftwareResetReason::deliberate;
		}

		// Record the reason for the software reset
		NonVolatileMemory mem;
		SoftwareResetData * const srd = mem.AllocateResetDataSlot();
        srd->Populate(reason, stk);
        mem.EnsureWritten();
	}

#if defined(__LPC17xx__)
    LPC_SYSCTL->RSID = 0x3F;					// Clear bits in reset reasons stored in RSID
#elif !SAME5x
	RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD;			// ignore any signal on the NRST pin for now so that the reset reason will show as Software
#endif
	Reset();
	for(;;) {}
}

// Exception handlers
// By default the Usage Fault, Bus Fault and Memory Management fault handlers are not enabled,
// so they escalate to a Hard Fault and we don't need to provide separate exception handlers for them.
extern "C" [[noreturn]] void hardFaultDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	SoftwareReset((uint16_t)SoftwareResetReason::hardFault, pulFaultStackAddress);
}

// The fault handler implementation calls a function called hardFaultDispatcher()
extern "C" [[noreturn]] void HardFault_Handler() noexcept __attribute__((naked));
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
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_hf_address_const: .word hardFaultDispatcher       \n"
	);
}

#if USE_MPU

extern "C" [[noreturn]] void memManageDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	SoftwareReset((uint16_t)SoftwareResetReason::memFault, pulFaultStackAddress);
}

// The fault handler implementation calls a function called memManageDispatcher()
extern "C" [[noreturn]] void MemManage_Handler() noexcept __attribute__((naked));
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
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_mf_address_const: .word memManageDispatcher       \n"
	);
}

#endif

extern "C" [[noreturn]] void wdtFaultDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	SoftwareReset((uint16_t)SoftwareResetReason::wdtFault, pulFaultStackAddress);
}

#ifdef __LPC17xx__
extern "C" [[noreturn]] void WDT_IRQHandler() noexcept __attribute__((naked));
void WDT_IRQHandler() noexcept
{
	LPC_WDT->MOD &=~((uint32_t)(1<<2)); //SD::clear timout flag before resetting to prevent the Smoothie bootloader going into DFU mode
#else
extern "C" [[noreturn]] void WDT_Handler() noexcept __attribute__((naked));
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
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_wdt_address_const: .word wdtFaultDispatcher       \n"
	);
}

extern "C" [[noreturn]] void otherFaultDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	SoftwareReset((uint16_t)SoftwareResetReason::otherFault, pulFaultStackAddress);
}

// 2017-05-25: A user is getting 'otherFault' reports, so now we do a stack dump for those too.
// The fault handler implementation calls a function called otherFaultDispatcher()
extern "C" [[noreturn]] void OtherFault_Handler() noexcept __attribute__((naked));
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
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_oflt_address_const: .word otherFaultDispatcher    \n"
	);
}

// We could set up the following fault handlers to retrieve the program counter in the same way as for a Hard Fault,
// however these exceptions are unlikely to occur, so for now we just report the exception type.
extern "C" [[noreturn]] void NMI_Handler        () noexcept { SoftwareReset((uint16_t)SoftwareResetReason::NMI); }
extern "C" [[noreturn]] void UsageFault_Handler () noexcept { SoftwareReset((uint16_t)SoftwareResetReason::usageFault); }

extern "C" [[noreturn]] void DebugMon_Handler   () noexcept __attribute__ ((alias("OtherFault_Handler")));

// FreeRTOS hooks that we need to provide
extern "C" [[noreturn]] void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress, char* pcTaskName) noexcept
{
	SoftwareReset((uint16_t)SoftwareResetReason::stackOverflow, pulFaultStackAddress);
}

extern "C" [[noreturn]] void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) noexcept __attribute((naked));
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) noexcept
{
	// r0 = pxTask, r1 = pxTaskName
	__asm volatile
	(
		" push {r0, r1, lr}											\n"		/* save parameters and call address on the stack */
		" mov r0, sp												\n"
		" ldr r2, handler_sovf_address_const                        \n"
		" bx r2                                                     \n"
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_sovf_address_const: .word stackOverflowDispatcher \n"
	);
}

extern "C" [[noreturn]] void assertCalledDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
}

extern "C" [[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept __attribute((naked));
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
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_asrt_address_const: .word assertCalledDispatcher  \n"
	);
}

#ifdef __LPC17xx__
[[noreturn]] void applicationMallocFailedCalledDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	reprap.SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
}

[[noreturn]] extern "C" void vApplicationMallocFailedHook() noexcept __attribute((naked));
void vApplicationMallocFailedHook() noexcept
{
	 __asm volatile
	(
		" push {r0, r1, lr}											\n"        /* save parameters and call address */
		" mov r0, sp												\n"
		" ldr r2, handler_amf_address_const							\n"
		" bx r2														\n"
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_amf_address_const: .word applicationMallocFailedCalledDispatcher  \n"
	 );
}
#endif

namespace std
{
	// We need to define this function in order to use lambda functions with captures
	[[noreturn]] void __throw_bad_function_call() noexcept { vAssertCalled(__LINE__, __FILE__); }
}

// The default terminate handler pulls in sprintf and lots of other functions, which makes the binary too large. So we replace it.
[[noreturn]] void Terminate() noexcept
{
	register const uint32_t * stack_ptr asm ("sp");
	SoftwareReset((uint16_t)SoftwareResetReason::terminateCalled, stack_ptr);
}

namespace __cxxabiv1
{
	std::terminate_handler __terminate_handler = Terminate;
}

extern "C" [[noreturn]] void __cxa_pure_virtual() noexcept
{
	register const uint32_t * stack_ptr asm ("sp");
	SoftwareReset((uint16_t)SoftwareResetReason::pureVirtual, stack_ptr);
}

extern "C" [[noreturn]] void __cxa_deleted_virtual() noexcept
{
	register const uint32_t * stack_ptr asm ("sp");
	SoftwareReset((uint16_t)SoftwareResetReason::deletedVirtual, stack_ptr);
}

// End
