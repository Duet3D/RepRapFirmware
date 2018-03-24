/*
 * SoftTimer.h
 *
 *  Created on: 21 Jul 2017
 *      Author: David
 */

#ifndef SRC_SOFTTIMER_H_
#define SRC_SOFTTIMER_H_

#include "RepRapFirmware.h"


// Class to implement a software timer
class SoftTimer
{
public:
	typedef bool (*Callback)(void*, uint32_t&);
	typedef uint32_t Ticks;

	SoftTimer();

	// Schedule a callback at a particular tick count, returning true if it was not scheduled because it is already due or imminent
	bool ScheduleCallback(Ticks when, Callback cb, void* param);

	// Cancel any scheduled callbacks
	void CancelCallback();

	// Get the current tick count
	static Ticks GetTimerTicksNow();

	// Get the tick rate
	static Ticks GetTickRate();

	// ISR called from Platform. May sometimes get called prematurely.
	static void Interrupt();

private:
	SoftTimer *next;
	Ticks whenDue;
	Callback callback;
	void *cbParam;

	static SoftTimer * volatile pendingList;			// list of pending callbacks, soonest first
};

#endif /* SRC_SOFTTIMER_H_ */
