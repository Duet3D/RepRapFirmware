/*
 * SoftTimer.cpp
 *
 *  Created on: 21 Jul 2017
 *      Author: David
 */

#include "SoftTimer.h"
#include "Platform.h"

SoftTimer * volatile SoftTimer::pendingList = nullptr;

SoftTimer::SoftTimer() : next(nullptr), callback(nullptr)
{
}

// Schedule a callback at a particular tick count, returning true if it was not scheduled because it is already due or imminent.
// There must be no callback already scheduled for this timer, else the linked list will get messed up. If in doubt, call CancelCallback before calling this.
bool SoftTimer::ScheduleCallback(Ticks when, Callback cb, void *param)
{
	whenDue = when;
	callback = cb;
	cbParam = param;

	const irqflags_t flags = cpu_irq_save();
	const Ticks now = GetTimerTicksNow();
	const int32_t howSoon = (int32_t)(when - now);
	SoftTimer** ppst = const_cast<SoftTimer**>(&pendingList);
	if (*ppst == nullptr || howSoon < (int32_t)((*ppst)->whenDue - now))
	{
		// No other callbacks are scheduled, or this one is due earlier than the first existing one
		if (Platform::ScheduleSoftTimerInterrupt(when))
		{
			cpu_irq_restore(flags);
			return true;
		}
	}
	else
	{
		while (*ppst != nullptr && (int32_t)((*ppst)->whenDue - now) < howSoon)
		{
			ppst = &((*ppst)->next);
		}
	}

	next = *ppst;
	*ppst = this;
	cpu_irq_restore(flags);
	return false;
}

// Cancel any scheduled callback for this timer. Harmless if there is no callback scheduled.
void SoftTimer::CancelCallback()
{
	const irqflags_t flags = cpu_irq_save();
	for (SoftTimer** ppst = const_cast<SoftTimer**>(&pendingList); *ppst != nullptr; ppst = &((*ppst)->next))
	{
		if (*ppst == this)
		{
			*ppst = this->next;		// unlink this from the pending list
			break;
		}
	}
	cpu_irq_restore(flags);
}

// Get the current tick count
/*static*/ SoftTimer::Ticks SoftTimer::GetTimerTicksNow()
{
	return Platform::GetInterruptClocks();
}

// Get the tick rate
/*static*/ SoftTimer::Ticks SoftTimer::GetTickRate()
{
	return StepClockRate;			// the software timer uses the same counter as the step timer
}

// ISR called from Platform. May sometimes get called prematurely.
/*static*/ void SoftTimer::Interrupt()
{
	for (;;)
	{
		SoftTimer * const tmr = pendingList;
		if (tmr == nullptr)
		{
			break;
		}

		// On the first iteration, the timer at the head of the list is probably expired.
		// Try to schedule another interrupt for it, if we get a true return then it has indeed expired and we need to execute the callback.
		// On subsequent iterations this just sets up the interrupt for the next timer that is due to expire.
		if (Platform::ScheduleSoftTimerInterrupt(tmr->whenDue))
		{
			pendingList = tmr->next;														// remove it from the pending list
			if (tmr->callback != nullptr && tmr->callback(tmr->cbParam, tmr->whenDue))		// execute its callback
			{
				// Schedule another callback for this timer
				SoftTimer** ppst = const_cast<SoftTimer**>(&pendingList);
				while (*ppst != nullptr && (int32_t)(tmr->whenDue - (*ppst)->whenDue) > 0)
				{
					ppst = &((*ppst)->next);
				}
				tmr->next = *ppst;
				*ppst = tmr;
			}
		}
		else
		{
			break;
		}
	}
}

// End
