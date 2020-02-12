/*
 * PortControl.cpp
 *
 *  Created on: 15 Jun 2017
 *      Author: David
 */

#include "PortControl.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Movement/Move.h"
#include "Movement/DDA.h"
#include "Movement/StepTimer.h"

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

// Update the IO bits. Return the number of milliseconds before we need to be called again, or 0 to be called when movement restarts.
uint32_t PortControl::UpdatePorts() noexcept
{
	if (numConfiguredPorts == 0)
	{
		return 0;
	}

	SetBasePriority(NvicPriorityStep);
	const DDA * cdda = reprap.GetMove().GetMainDDARing().GetCurrentDDA();
	if (cdda == nullptr)
	{
		// Movement has stopped, so turn all ports off
		SetBasePriority(0);
		UpdatePorts(0);
		return 0;
	}

	// Find the DDA whose IO port bits we should set now
	const uint32_t now = StepTimer::GetTimerTicks() + advanceClocks;
	uint32_t moveEndTime = cdda->GetMoveStartTime();
	DDA::DDAState st = cdda->GetState();
	do
	{
		moveEndTime += cdda->GetClocksNeeded();
		if ((int32_t)(moveEndTime - now) >= 0)
		{
			SetBasePriority(0);
			UpdatePorts(cdda->GetIoBits());
			return (moveEndTime - now + StepTimer::StepClockRate/1000 - 1)/(StepTimer::StepClockRate/1000);
		}
		cdda = cdda->GetNext();
		st = cdda->GetState();
	} while (st == DDA::executing || st == DDA::frozen);

	SetBasePriority(0);
	UpdatePorts(0);
	return 0;
}

// Set up the GPIO ports returning true if error
bool PortControl::Configure(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	if (gb.Seen('C'))
	{
		seen = true;
		UpdatePorts(0);
		IoPort * portAddresses[MaxPorts];
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
		advanceClocks = (advanceMillis * (uint64_t)StepTimer::StepClockRate)/1000;
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

void PortControl::UpdatePorts(IoBits_t newPortState) noexcept
{
	if (newPortState != currentPortState)
	{
		const IoBits_t bitsToClear = currentPortState & ~newPortState;
		const IoBits_t bitsToSet = newPortState & ~currentPortState;
		for (size_t i = 0; i < numConfiguredPorts; ++i)
		{
			const IoBits_t mask = 1u << i;
			if (bitsToClear & mask)
			{
				portMap[i].WriteDigital(false);
			}
			else if (bitsToSet & mask)
			{
				portMap[i].WriteDigital(true);
			}
		}
		currentPortState = newPortState;
	}
}

#endif

// End
