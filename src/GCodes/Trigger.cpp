/*
 * Trigger.cpp
 *
 *  Created on: 6 Jun 2019
 *      Author: David
 */

#include "Trigger.h"
#include "RepRap.h"
#include "PrintMonitor.h"

Trigger::Trigger() noexcept
{
	condition = 0;
}

// Initialise the trigger
void Trigger::Init() noexcept
{
	for (IoPort& port : ports)
	{
		port.Release();
	}
	condition = 0;
}

// Return true if this trigger is unused, i.e. it doesn't watch any pins
bool Trigger::IsUnused() const noexcept
{
	for (const IoPort& port : ports)
	{
		if (port.IsValid())
		{
			return false;
		}
	}
	return true;
}

// Check whether this trigger is active and update the input states
bool Trigger::Check() noexcept
{
	bool triggered = false;
	for (unsigned int i = 0; i < ARRAY_SIZE(ports); ++i)
	{
		if (!ports[i].IsValid())					// reached the end of the used ports
		{
			break;
		}
		const bool b = ports[i].Read();
		if (b != inputStates.IsBitSet(i))			// if the input level has changed
		{
			if (b)
			{
				inputStates.SetBit(i);
				triggered = true;
			}
			else
			{
				inputStates.ClearBit(i);
			}
		}
	}
	return triggered && (condition == 0 || (condition == 1 && reprap.GetPrintMonitor().IsPrinting()));
}

// End
