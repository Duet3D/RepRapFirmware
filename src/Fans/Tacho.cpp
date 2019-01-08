/*
 * Tacho.cpp
 *
 *  Created on: 16 Jul 2018
 *      Author: David
 */

#include "Tacho.h"
#include "Platform.h"
#include "Movement/StepTimer.h"

void FanInterrupt(CallbackParameter cb)
{
	static_cast<Tacho *>(cb.vp)->Interrupt();
}

Tacho::Tacho() : fanInterruptCount(0), fanLastResetTime(0), fanInterval(0), pin(NoPin)
{
}

void Tacho::Init(Pin p_pin)
{
	pin = p_pin;
	if (pin != NoPin)
	{
		pinModeDuet(pin, INPUT_PULLUP, 1500);		// enable pullup and 1500Hz debounce filter (500Hz only worked up to 7000RPM)
		attachInterrupt(pin, FanInterrupt, INTERRUPT_MODE_FALLING, this);
	}
}

uint32_t Tacho::GetRPM() const
{
	// The ISR sets fanInterval to the number of step interrupt clocks it took to get fanMaxInterruptCount interrupts.
	// We get 2 tacho pulses per revolution, hence 2 interrupts per revolution.
	// However, if the fan stops then we get no interrupts and fanInterval stops getting updated.
	// We must recognise this and return zero.
	return (fanInterval != 0 && StepTimer::GetInterruptClocks() - fanLastResetTime < 3 * StepTimer::StepClockRate)	// if we have a reading and it is less than 3 second old
			? (StepTimer::StepClockRate * fanMaxInterruptCount * (60/2))/fanInterval		// then calculate RPM assuming 2 interrupts per rev
			: 0;																// else assume fan is off or tacho not connected
}

void Tacho::Interrupt()
{
	++fanInterruptCount;
	if (fanInterruptCount == fanMaxInterruptCount)
	{
		const uint32_t now = StepTimer::GetInterruptClocks();
		fanInterval = now - fanLastResetTime;
		fanLastResetTime = now;
		fanInterruptCount = 0;
	}
}

// End
