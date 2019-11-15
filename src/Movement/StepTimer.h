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
class StepTimer
{
public:
	// The callback function returns true if it wants another callback, after setting the requested time via the second parameter
	typedef uint32_t Ticks;
	typedef bool (*TimerCallbackFunction)(CallbackParameter, Ticks&);

	StepTimer();

	// Set up the callback function and parameter
	void SetCallback(TimerCallbackFunction cb, CallbackParameter param);

	// Schedule a callback at a particular tick count, returning true if it was not scheduled because it is already due or imminent
	bool ScheduleCallback(Ticks when);

	// As ScheduleCallback but base priority >= NvicPriorityStep when called
	bool ScheduleCallbackFromIsr(Ticks when);

	// Cancel any scheduled callbacks
	void CancelCallback();

	// As CancelCallback but base priority >= NvicPriorityStep when called
	void CancelCallbackFromIsr();

	// Initialise the timer system
	static void Init();

	// Disable the timer interrupt. Called when we shut down the system.
	static void DisableTimerInterrupt();

	// Get the current tick count
	static Ticks GetTimerTicks() __attribute__ ((hot));

	// Get the current tick count when we only need a 16-bit value. Faster than GetTimerTicks() on the SAM4S and SAME70.
	static uint16_t GetTimerTicks16();

	// Get the tick rate (can also access it directly as StepClockRate)
	static uint32_t GetTickRate() { return StepClockRate; }

	// ISR called from StepTimer. May sometimes get called prematurely.
	static void Interrupt();

#if SAME70
	static constexpr uint32_t StepClockRate = 48000000/64;						// 750kHz
#else
	static constexpr uint32_t StepClockRate = VARIANT_MCK/128;					// just under 1MHz
#endif

	static constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
	static constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;
	static constexpr uint32_t MinInterruptInterval = 6;							// about 6us

private:
	static bool ScheduleTimerInterrupt(uint32_t tim);							// Schedule an interrupt at the specified clock count, or return true if it has passed already

	StepTimer *next;
	Ticks whenDue;
	TimerCallbackFunction callback;
	CallbackParameter cbParam;
	volatile bool active;

	static StepTimer * volatile pendingList;			// list of pending callbacks, soonest first
};

// Function GetTimerTicks() is quite long for SAM4S and SAME70 processors, so it is moved to StepTimer.cpp and no longer inlined
#if !(SAM4S || SAME70)

inline StepTimer::Ticks StepTimer::GetTimerTicks()
{
# ifdef __LPC17xx__
	return STEP_TC->TC;
# else
	return STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
# endif
}

#endif

inline uint16_t StepTimer::GetTimerTicks16()
{
#ifdef __LPC17xx__
	return (uint16_t)STEP_TC->TC;
#else
	return (uint16_t)STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
#endif
}

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
