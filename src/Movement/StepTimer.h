/*
 * StepTimer.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPTIMER_H_
#define SRC_MOVEMENT_STEPTIMER_H_

#include "RepRapFirmware.h"

#define STEP_TIMER_DEBUG	0

class CanMessageTimeSync;

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
	bool ScheduleCallback(Ticks when) noexcept SPEED_CRITICAL;

	// As ScheduleCallback but base priority >= NvicPriorityStep when called. Can be called from within a callback.
	bool ScheduleCallbackFromIsr(Ticks when) noexcept SPEED_CRITICAL;

	// Check whether a callback really is due, schedule it if not. Returns true if it really is due. Can be called from within a callback.
	bool ScheduleCallbackFromIsr() noexcept SPEED_CRITICAL;

	// Cancel any scheduled callbacks
	void CancelCallback() noexcept;

	// As CancelCallback but base priority >= NvicPriorityStep when called
	void CancelCallbackFromIsr() noexcept SPEED_CRITICAL;

	// Initialise the timer system
	static void Init() noexcept;

	// Disable the timer interrupt. Called when we shut down the system.
	static void DisableTimerInterrupt() noexcept;

	// Get the current tick count
	static Ticks GetTimerTicks() noexcept SPEED_CRITICAL;

	// Get the current tick count when we only need a 16-bit value. Faster than GetTimerTicks() on the SAM4S and SAME70.
	static uint16_t GetTimerTicks16() noexcept;

	// Get the tick rate (can also access it directly as StepClockRate)
	static constexpr uint32_t GetTickRate() noexcept { return StepClockRate; }

	// ISR called from StepTimer
	static void Interrupt() noexcept;

	// Append diagnostics to reply string
	static void Diagnostics(const StringRef& reply) noexcept;

	static constexpr uint32_t MinInterruptInterval = 6;							// Minimum interval between step timer interrupts, in step clocks; about 6us

	// Convert a number of step timer ticks to microseconds
	// Our tick rate is a multiple of 1000 so instead of multiplying n by 1000000 and risking overflow, we multiply by 1000 and divide by StepClockRate/1000
	static uint32_t TicksToIntegerMicroseconds(uint32_t n) noexcept { return (n * 1000)/(StepClockRate/1000); }
	static float TicksToFloatMicroseconds(uint32_t n) noexcept { return (float)n * (1000000.0f/StepClockRate); }

#if SUPPORT_REMOTE_COMMANDS
	static uint32_t GetLocalTimeOffset() noexcept { return localTimeOffset; }
	static void ProcessTimeSyncMessage(const CanMessageTimeSync& msg, size_t msgLen, uint16_t timeStamp) noexcept;
	static uint32_t ConvertToLocalTime(uint32_t masterTime) noexcept { return masterTime + localTimeOffset; }
	static uint32_t ConvertToMasterTime(uint32_t localTime) noexcept { return localTime - localTimeOffset; }
	static uint32_t GetMasterTime() noexcept { return ConvertToMasterTime(GetTimerTicks()); }
	static bool IsSynced() noexcept;

	static constexpr uint32_t MinSyncInterval = 2000;							// maximum interval in milliseconds between sync messages for us to remain synced
																				// increased from 1000 because of workaround we added for bad Tx time stamps on SAME70
#endif

#if STEP_TIMER_DEBUG
	static uint32_t maxInterval;
#endif

private:
	static bool ScheduleTimerInterrupt(uint32_t tim) noexcept;					// Schedule an interrupt at the specified clock count, or return true if it has passed already

	StepTimer *next;
	Ticks whenDue;
	TimerCallbackFunction callback;
	CallbackParameter cbParam;
	volatile bool active;

	static StepTimer * volatile pendingList;									// list of pending callbacks, soonest first

#if STEP_TIMER_DEBUG
	static uint32_t lastTimerResult;
#endif

#if SUPPORT_REMOTE_COMMANDS
	static volatile uint32_t localTimeOffset;									// local time minus master time
	static volatile uint32_t whenLastSynced;									// the millis tick count when we last synced
	static uint32_t prevMasterTime;												// the previous master time received
	static uint32_t prevLocalTime;												// the previous local time when the master time was received, corrected for receive processing delay
	static int32_t peakPosJitter, peakNegJitter;								// the max and min corrections we made to local time offset while synced
	static bool gotJitter;														// true if we have recorded the jitter
	static uint32_t peakReceiveDelay;											// the maximum receive delay we measured by using the receive time stamp
	static volatile unsigned int syncCount;										// the number of messages we have received since starting sync
	static unsigned int numJitterResyncs, numTimeoutResyncs;

	static constexpr uint32_t MaxSyncJitter = StepClockRate/100;				// 10ms
	static constexpr unsigned int MaxSyncCount = 10;
#endif
};

// Function GetTimerTicks() is quite long for SAM4S and SAME70 processors, so it is moved to StepTimer.cpp and no longer inlined
#if !(SAM4S || SAME70 || SAME5x)

inline __attribute__((always_inline)) StepTimer::Ticks StepTimer::GetTimerTicks() noexcept
{
	return STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
}

#endif

inline __attribute__((always_inline)) uint16_t StepTimer::GetTimerTicks16() noexcept
{
#if SAME5x
	return (uint16_t)GetTimerTicks();
#else
	return (uint16_t)STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
#endif
}

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
