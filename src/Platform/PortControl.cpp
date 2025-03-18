/*
 * PortControl.cpp
 *
 *  Created on: 15 Jun 2017
 *      Author: David
 */

#include "PortControl.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GPIO/GpOutPort.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

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

// Set up the GPIO ports
GCodeResult PortControl::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	if (gb.Seen('P'))
	{
		seen = true;
		UpdatePorts(0);
		numConfiguredPorts = 0;
		uint32_t tempPorts[MaxPorts];
		size_t nports = MaxPorts;
		gb.GetUnsignedArray(tempPorts, nports, false);
		Platform& platform = reprap.GetPlatform();
		for (size_t i = 0; i < nports; ++i)
		{
			if (   tempPorts[i] >= MaxGpOutPorts
				|| platform.GetGpOutPort(tempPorts[i]).IsUnused()
#if SUPPORT_CAN_EXPANSION
				|| !platform.GetGpOutPort(tempPorts[i]).IsLocal()
#endif
			   )
			{
				reply.printf("GpOut port %" PRIu32 " is not valid", tempPorts[i]);
				return GCodeResult::error;
			}
			gpOutPortNumbers[i] = tempPorts[i];
		}
		numConfiguredPorts = nports;
	}

	if (gb.TryGetLimitedUIValue('T', advanceMillis, seen, MaxAdvanceMillis + 1))
	{
		if constexpr (StepClockRate % 1000 == 0)
		{
			advanceClocks = advanceMillis * (StepClockRate/1000);				// this works for Duet 3, step clock rate is 750kHz
		}
		else if constexpr (StepClockRate % 500 == 0)
		{
			advanceClocks = (advanceMillis * (StepClockRate/500))/2;			// this works for Duet 2, step clock rate is 937500Hz
		}
		else
		{
			advanceClocks = (advanceMillis * (uint64_t)StepClockRate)/1000;		// catch-all in case of using other step clock rates
		}
	}

	if (!seen)
	{
		if (numConfiguredPorts == 0)
		{
			reply.cat("no port mapping configured");
		}
		else
		{
			reply.printf("Advance %" PRIu32 "ms, ", advanceMillis);
			reply.cat("ports");
			for (size_t i = 0; i < numConfiguredPorts; ++i)
			{
				reply.catf(" %u", gpOutPortNumbers[i]);
			}
		}
	}
	return GCodeResult::ok;
}

// Set the ports to the requested state
void PortControl::UpdatePorts(IoBits_t newPortState) noexcept
{
	if (newPortState != currentPortState)
	{
		const IoBits_t bitsToClear = currentPortState & ~newPortState;
		const IoBits_t bitsToSet = newPortState & ~currentPortState;
		Platform& platform = reprap.GetPlatform();
		for (size_t i = 0; i < numConfiguredPorts; ++i)
		{
			GpOutputPort& port = platform.GetGpOutPort(gpOutPortNumbers[i]);
			const IoBits_t mask = 1u << i;
			if ((bitsToClear & mask) != 0)
			{
				port.WriteDigital(false);
			}
			else if ((bitsToSet & mask) != 0)
			{
				port.WriteDigital(true);
			}
		}
		currentPortState = newPortState;
	}
}

#endif

// End
