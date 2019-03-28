/*
 * StepTimer.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPTIMER_H_
#define SRC_MOVEMENT_STEPTIMER_H_

#include "RepRapFirmware.h"

namespace StepTimer
{
	constexpr uint32_t StepClockRate = VARIANT_MCK/128;					// just under 1MHz
	constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
	constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;
	constexpr uint32_t MinInterruptInterval = 6;						// 12 clocks is about 6us

	void Init();

	// Function GetInterruptClocksInterruptsDisabled() is quite long for SAM4S and SAME70 processors, so it is moved to StepTimer.cpp and no longer inlined
	// On other processors we have had trouble with the compiler moving instructions around too much when it is inlined, so we don't inline it any more.
	uint32_t GetInterruptClocksInterruptsDisabled() __attribute__ ((hot));	// Get the interrupt clock count, when we know already that interrupts are disabled or base priority >= step interrupt priority

#if SAM4S || SAME70		// if the TCs are 16-bit

	// Get the interrupt clock count
	static inline uint32_t GetInterruptClocks()
	{
		const uint32_t baseprio = ChangeBasePriority(NvicPriorityStep);	// ensure step interrupts are disabled
		const uint32_t rslt = GetInterruptClocksInterruptsDisabled();
		RestoreBasePriority(baseprio);									// restore interrupt enable state
		return rslt;
	}

#else					// TCs are 32-bit

	// Get the interrupt clock count
	static inline uint32_t GetInterruptClocks()
	{
		return GetInterruptClocksInterruptsDisabled();					// no need to disable interrupts on these processors
	}

#endif

	// Get the interrupt clock count when we only care about the lowest 16 bits. More efficient than calling GetInterruptClocks on platforms with 16-bit timers.
	static inline uint16_t GetInterruptClocks16()
	{
#if __LPC17xx__
        return (uint16_t)STEP_TC->TC;
#else
        return (uint16_t)STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
#endif
	}

	bool ScheduleStepInterrupt(uint32_t tim) __attribute__ ((hot));		// Schedule an interrupt at the specified clock count, or return true if it has passed already
	void DisableStepInterrupt();										// Make sure we get no step interrupts
	bool ScheduleSoftTimerInterrupt(uint32_t tim);						// Schedule an interrupt at the specified clock count, or return true if it has passed already
	void DisableSoftTimerInterrupt();									// Make sure we get no software timer interrupts
}

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
