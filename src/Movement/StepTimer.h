/*
 * StepTimer.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPTIMER_H_
#define SRC_MOVEMENT_STEPTIMER_H_

#include "RepRapFirmware.h"

class CanMessageTimeSync;

// Class to implement a software timer with a few microseconds resolution
// Important! In systems that use 16-bit timers, callbacks may take place at multiples of 65536 ticks before they are actually due.
// In order to achieve the maximum step rate possible, the timer code doesn't check for this, because the step generation code checks which drivers are due steps anyway.
// Any other client that uses the timer MUST do a similar check. The simple way to do this is to use a callback function of the following form:
// if (timer.ScheduleCallbackFromIsr()) { /* code to execute it the callback really was due */ }
class StepTimer final
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

	// As ScheduleCallback but add the movement delay, and must have base priority >= NvicPriorityStep when called. Can be called from within a callback.
	bool ScheduleMovementCallbackFromIsr(Ticks when) noexcept SPEED_CRITICAL;

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

	// Get the current tick count, adjusted for the movement delay
	static Ticks GetMovementTimerTicks() noexcept SPEED_CRITICAL;

	// Convert local time to movement time
	static Ticks ConvertLocalToMovementTime(Ticks localTime) noexcept;

	// Get the current tick count when we only need a 16-bit value. Faster than GetTimerTicks() on the SAM4S and SAME70.
	static uint16_t GetTimerTicks16() noexcept;

	// Get the tick rate (can also access it directly as StepClockRate)
	static constexpr uint32_t GetTickRate() noexcept { return StepClockRate; }

	// Add more movement delay
	static void IncreaseMovementDelay(uint32_t increase) noexcept;

	// Return the current movement delay
	static Ticks GetMovementDelay() noexcept { return movementDelay; }

#if SUPPORT_CAN_EXPANSION
	// Handle a request for movement delay received from an expansion board
	static void ProcessMovementDelayRequest(uint32_t delayRequested) noexcept;

	// Check whether the movement delay has increased since we last called this. If yes, return the movement delay; else return zero.
	static Ticks CheckMovementDelayIncreased() noexcept;

	// Report the amount of movement delay that this board is responsible for
	static uint32_t GetOwnMovementDelay() noexcept { return ownMovementDelay; }
#endif

#if SUPPORT_REMOTE_COMMANDS
	// Check whether the movement delay has increased since we last called this. If yes, return the movement delay; else return zero.
	// We leave the movementDelayIncreased flag set until the main board acknowledges the increased movement delay.
	static Ticks CheckMovementDelayIncreasedNoClear() noexcept;
#endif

	// ISR called from StepTimer
	static void Interrupt() noexcept;

	// Append diagnostics to reply string
	static void Diagnostics(const StringRef& reply) noexcept;

	// Convert a number of step timer ticks to microseconds
	// Our tick rate is a multiple of 1000 so instead of multiplying n by 1000000 and risking overflow, we multiply by 1000 and divide by StepClockRate/1000
	static uint32_t TicksToIntegerMicroseconds(uint32_t n) noexcept { return (n * 1000)/(StepClockRate/1000); }
	static float TicksToFloatMicroseconds(uint32_t n) noexcept { return (float)n * (1000000.0f/(float)StepClockRate); }

#if SUPPORT_REMOTE_COMMANDS
	static uint32_t GetLocalTimeOffset() noexcept { return localTimeOffset; }
	static void ProcessTimeSyncMessage(const CanMessageTimeSync& msg, size_t msgLen, uint16_t timeStamp) noexcept;
	static uint32_t ConvertToLocalTime(uint32_t masterTime) noexcept { return masterTime + localTimeOffset; }
	static uint32_t ConvertToMasterTime(uint32_t localTime) noexcept { return localTime - localTimeOffset; }
	static uint32_t GetMasterTime() noexcept { return ConvertToMasterTime(GetTimerTicks()); }

	static bool CheckSynced() noexcept;											// check whether we have synced and received a clock sync message recently
	static bool IsSynced() noexcept;											// check whether we have synced

	static constexpr uint32_t MinSyncInterval = 2000;							// maximum interval in milliseconds between sync messages for us to remain synced
																				// increased from 1000 because of workaround we added for bad Tx time stamps on SAME70
#endif

private:
	static bool ScheduleTimerInterrupt(uint32_t tim) noexcept;					// Schedule an interrupt at the specified clock count, or return true if it has passed already

	static uint32_t movementDelay;												// how many timer ticks the move timer is behind the raw timer

#if SUPPORT_CAN_EXPANSION
	static uint32_t ownMovementDelay;											// the amount of movement delay requested by this board
	static bool movementDelayIncreased;											// true if movement delay has increased and we haven't yet broadcast that
#endif

	StepTimer *_ecv_null next;
	Ticks whenDue;
	TimerCallbackFunction _ecv_null callback;
	CallbackParameter cbParam;
	volatile bool active;

	static StepTimer *_ecv_null volatile pendingList;									// list of pending callbacks, soonest first

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

// Add more movement delay
inline void StepTimer::IncreaseMovementDelay(uint32_t increase) noexcept
{
	movementDelay += increase;
#if SUPPORT_CAN_EXPANSION
	ownMovementDelay += increase;
	movementDelayIncreased = true;
#endif
}

// Get the current tick count
inline StepTimer::Ticks StepTimer::GetMovementTimerTicks() noexcept
{
	return GetTimerTicks() - movementDelay;
}

// Convert local time to movement time
inline StepTimer::Ticks StepTimer::ConvertLocalToMovementTime(Ticks localTime) noexcept
{
	return localTime - movementDelay;
}

#if SUPPORT_CAN_EXPANSION

// Check whether the movement delay has increased since we last called this. If yes, return the movement delay; else return zero.
inline StepTimer::Ticks StepTimer::CheckMovementDelayIncreased() noexcept
{
	AtomicCriticalSectionLocker lock;
	if (movementDelayIncreased)
	{
		movementDelayIncreased = false;
		return movementDelay;
	}
	return 0;
}

#endif

#if SUPPORT_REMOTE_COMMANDS

// Check whether the movement delay has increased since we last called this. If yes, return the movement delay; else return zero.
// We leave the movementDelayIncreased flag set until the main board acknowledges the increased movement delay.
inline StepTimer::Ticks StepTimer::CheckMovementDelayIncreasedNoClear() noexcept
{
	return (movementDelayIncreased) ? movementDelay : 0;
}

#endif

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
