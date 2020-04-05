/*
 * StepTimer.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPTIMER_H_
#define SRC_MOVEMENT_STEPTIMER_H_

#include "RepRapFirmware.h"

// Class to implement a software timer with a few microseconds resolution
// Important! In systems that use 16-bit timers, callbacks may take place at multiples of 65536 ticks before they are actually due.
// In order to achieve the maximum step rate possible, the timer code doesn't check for this, because the step generation code checks which drivers are due steps anyway.
// Any other client that uses the timer MUST do a similar check. The simple way to do this is to use a callback function of the following form:
// if (timer.ScheduleCallbackFromIsr()) { /* code to execute it the callback really was due */ }
class StepTimer
{
public:
	typedef uint32_t Ticks;
	typedef void (*TimerCallbackFunction)(CallbackParameter) noexcept;

	StepTimer() noexcept;

	// Set up the callback function and parameter
	void SetCallback(TimerCallbackFunction cb, CallbackParameter param) noexcept;

	// Schedule a callback at a particular tick count, returning true if it was not scheduled because it is already due or imminent
	bool ScheduleCallback(Ticks when) noexcept;

	// As ScheduleCallback but base priority >= NvicPriorityStep when called. Can be called from within a callback.
	bool ScheduleCallbackFromIsr(Ticks when) noexcept;

	// Check whether a callback really is due, schedule it if not. Returns true if it really is due. Can be called from within a callback.
	bool ScheduleCallbackFromIsr() noexcept;

	// Cancel any scheduled callbacks
	void CancelCallback() noexcept;

	// As CancelCallback but base priority >= NvicPriorityStep when called
	void CancelCallbackFromIsr() noexcept;

	// Initialise the timer system
	static void Init() noexcept;

	// Disable the timer interrupt. Called when we shut down the system.
	static void DisableTimerInterrupt() noexcept;

	// Get the current tick count
	static Ticks GetTimerTicks() noexcept __attribute__ ((hot));

	// Get the current tick count when we only need a 16-bit value. Faster than GetTimerTicks() on the SAM4S and SAME70.
	static uint16_t GetTimerTicks16() noexcept;

	// Get the tick rate (can also access it directly as StepClockRate)
	static constexpr uint32_t GetTickRate() noexcept { return StepClockRate; }

	// ISR called from StepTimer
	static void Interrupt() noexcept;

#if SAME70
	static constexpr uint32_t StepClockRate = 48000000/64;						// 750kHz
#elif defined(__LPC17xx__)
	static constexpr uint32_t StepClockRate = 1000000;                          // 1MHz
#else
	static constexpr uint32_t StepClockRate = VARIANT_MCK/128;					// just under 1MHz
#endif

	static constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
	static constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;
	static constexpr uint32_t MinInterruptInterval = 6;							// about 6us

private:
	static bool ScheduleTimerInterrupt(uint32_t tim) noexcept;					// Schedule an interrupt at the specified clock count, or return true if it has passed already

	StepTimer *next;
	Ticks whenDue;
	TimerCallbackFunction callback;
	CallbackParameter cbParam;
	volatile bool active;

	static StepTimer * volatile pendingList;			// list of pending callbacks, soonest first
};

// Function GetTimerTicks() is quite long for SAM4S and SAME70 processors, so it is moved to StepTimer.cpp and no longer inlined
#if !(SAM4S || SAME70)

inline StepTimer::Ticks StepTimer::GetTimerTicks() noexcept
{
# ifdef __LPC17xx__
	return STEP_TC->TC;
# else
	return STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
# endif
}

#endif

inline uint16_t StepTimer::GetTimerTicks16() noexcept
{
#ifdef __LPC17xx__
	return (uint16_t)STEP_TC->TC;
#else
	return (uint16_t)STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
#endif
}

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
