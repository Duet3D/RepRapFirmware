/*
 * PortControl.cpp
 *
 *  Created on: 15 Jun 2017
 *      Author: David
 */

#include "PortControl.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/Move.h"
#include "Movement/DDA.h"

#if SUPPORT_IOBITS

PortControl::PortControl()
{
}

void PortControl::Init()
{
	numConfiguredPorts = 0;
	advanceMillis = 0;
	advanceClocks = 0;
	currentPortState = 0;
}

void PortControl::Exit()
{
	UpdatePorts(0);
	numConfiguredPorts = 0;
}

void PortControl::Spin(bool full)
{
	if (numConfiguredPorts != 0)
	{
		cpu_irq_disable();
		const DDA * cdda = reprap.GetMove().GetCurrentDDA();
		if (cdda == nullptr)
		{
			// Movement has stopped, so turn all ports off
			cpu_irq_enable();
			UpdatePorts(0);
		}
		else
		{
			const uint32_t now = Platform::GetInterruptClocks() + advanceClocks;
			uint32_t moveEndTime = cdda->GetMoveStartTime();
			DDA::DDAState st = cdda->GetState();
			do
			{
				moveEndTime += cdda->GetClocksNeeded();
				if ((int32_t)(moveEndTime - now) >= 0)
				{
					break;
				}
				cdda = cdda->GetNext();
				st = cdda->GetState();
			} while (st == DDA::executing || st == DDA::frozen);
			cpu_irq_enable();

			const IoBits_t bits = (st == DDA::executing || st == DDA::frozen || st == DDA::provisional) ? cdda->GetIoBits() : 0;
			UpdatePorts(bits);
		}
	}
}

bool PortControl::Configure(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	if (gb.Seen('P'))
	{
		seen = true;
		UpdatePorts(0);
		numConfiguredPorts = 0;
		uint32_t portNumbers[MaxPorts];
		size_t numPorts = MaxPorts;
		gb.GetUnsignedArray(portNumbers, numPorts, false);
		for (size_t i = 0; i < numPorts; ++i)
		{
			const uint32_t pnum = portNumbers[i];
			if (pnum > HighestLogicalPin)
			{
				reply.printf("Port number %ld out of range", pnum);
				return true;
			}
			IoPort& pm = portMap[i];
			if (!pm.Set((LogicalPin)pnum, PinAccess::write, false))
			{
				reply.printf("Port number %ld is not available", pnum);
				return true;
			}
			pm.WriteDigital(false);				// ensure the port is off
			if (i >= numConfiguredPorts)
			{
				numConfiguredPorts = i + 1;
			}
		}
	}
	if (gb.Seen('T'))
	{
		seen = true;
		advanceMillis = (unsigned int)constrain<int>(gb.GetIValue(), 0, 1000);
		advanceClocks = (advanceMillis * (uint64_t)StepClockRate)/1000;
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
			reply.cat("port numbers");
			for (size_t i = 0; i < numConfiguredPorts; ++i)
			{
				reply.catf(" %u", (unsigned int)portMap[i].GetLogicalPin());
			}
		}
	}
	return false;
}

void PortControl::UpdatePorts(IoBits_t newPortState)
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
