/*
 * PortControl.cpp
 *
 *  Created on: 15 Jun 2017
 *      Author: David
 */

#include "PortControl.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#if SUPPORT_IOBITS

PortControl::PortControl() noexcept
{
}

void PortControl::Init() noexcept
{
	numConfiguredPorts = 0;
	advanceMillis = 0;
	advanceClocks = 0;
	currentPortState = 0;
}

void PortControl::Exit() noexcept
{
	UpdatePorts(0);
	numConfiguredPorts = 0;
}

// Set up the GPIO ports returning true if error
bool PortControl::Configure(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	if (gb.Seen('C'))
	{
		seen = true;
		UpdatePorts(0);
		IoPort *_ecv_from portAddresses[MaxPorts];
		PinAccess access[MaxPorts];
		for (size_t i = 0; i < MaxPorts; ++i)
		{
			portAddresses[i] = &portMap[i];
			access[i] = PinAccess::write0;
		}
		numConfiguredPorts = IoPort::AssignPorts(gb, reply, PinUsedBy::gpout, MaxPorts, portAddresses, access);
		if (numConfiguredPorts == 0)
		{
			return true;
		}
	}
	if (gb.Seen('T'))
	{
		seen = true;
		advanceMillis = (unsigned int)constrain<int>(gb.GetIValue(), 0, 1000);
		if constexpr (StepClockRate % 1000 == 0)
		{
			advanceClocks = advanceMillis * (StepClockRate/1000);
		}
		else if constexpr (StepClockRate % 500 == 0)
		{
			advanceClocks = (advanceMillis * (StepClockRate/500))/2;
		}
		else
		{
			advanceClocks = (advanceMillis * (uint64_t)StepClockRate)/1000;
		}
	}
	if (!seen)
	{
		reply.printf("Advance %ums, ", advanceMillis);
		if (numConfiguredPorts == 0)
		{
			reply.cat("no port mapping configured");
		}
		else
		{
			reply.cat("ports");
			for (size_t i = 0; i < numConfiguredPorts; ++i)
			{
				reply.cat(' ');
				portMap[i].AppendPinName(reply);
			}
		}
	}
	return false;
}

// Set the ports to the requested state
void PortControl::UpdatePorts(IoBits_t newPortState) noexcept
{
	if (newPortState != currentPortState)
	{
		const IoBits_t bitsToClear = currentPortState & ~newPortState;
		const IoBits_t bitsToSet = newPortState & ~currentPortState;
		for (size_t i = 0; i < numConfiguredPorts; ++i)
		{
			const IoBits_t mask = 1u << i;
			if ((bitsToClear & mask) != 0)
			{
				portMap[i].WriteDigital(false);
			}
			else if ((bitsToSet & mask) != 0)
			{
				portMap[i].WriteDigital(true);
			}
		}
		currentPortState = newPortState;
	}
}

#endif

// End
