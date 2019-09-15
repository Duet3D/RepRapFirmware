/*
 * LocalSwitchEndstop.cpp
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#include "LocalSwitchEndstop.h"

#include "RepRap.h"
#include "Platform.h"
#include "Movement/Kinematics/Kinematics.h"

// Switch endstop
LocalSwitchEndstop::LocalSwitchEndstop(uint8_t axis, EndStopPosition pos) : Endstop(axis, pos), numPortsUsed(0)
{
	// ports will be initialised automatically by the IoPort default constructor
}

LocalSwitchEndstop::~LocalSwitchEndstop()
{
	for (size_t i = 0; i < ARRAY_SIZE(ports); ++i)
	{
		ports[i].Release();
	}
}

bool LocalSwitchEndstop::Configure(GCodeBuffer& gb, const StringRef& reply, EndStopInputType inputType)
{
	IoPort *portAddrs[MaxDriversPerAxis];
	PinAccess access[MaxDriversPerAxis];
	for (size_t i = 0; i < MaxDriversPerAxis; ++i)
	{
		portAddrs[i] = &ports[i];
		access[i] = PinAccess::read;
	}
	//TODO the port strings may include remote ports
	numPortsUsed = IoPort::AssignPorts(gb, reply, PinUsedBy::endstop, MaxDriversPerAxis, portAddrs, access);
	for (IoPort& pp : ports)
	{
		pp.ToggleInvert(inputType == EndStopInputType::activeLow);
	}
	return numPortsUsed != 0;
}

bool LocalSwitchEndstop::Configure(const char *pinNames, const StringRef& reply, EndStopInputType inputType)
{
	IoPort *portAddrs[MaxDriversPerAxis];
	PinAccess access[MaxDriversPerAxis];
	for (size_t i = 0; i < MaxDriversPerAxis; ++i)
	{
		portAddrs[i] = &ports[i];
		access[i] = PinAccess::read;
	}
	numPortsUsed = IoPort::AssignPorts(pinNames, reply, PinUsedBy::endstop, MaxDriversPerAxis, portAddrs, access);
	for (IoPort& pp : ports)
	{
		pp.ToggleInvert(inputType == EndStopInputType::activeLow);
	}
	return numPortsUsed != 0;
}

void LocalSwitchEndstop::Reconfigure(EndStopPosition pos, EndStopInputType inputType)
{
	SetAtHighEnd(pos == EndStopPosition::highEndStop);
	for (IoPort& pp : ports)
	{
		pp.SetInvert(inputType == EndStopInputType::activeLow);
	}
}

EndStopInputType LocalSwitchEndstop::GetEndstopType() const
{
	return (ports[0].GetInvert()) ? EndStopInputType::activeLow : EndStopInputType::activeHigh;
}

// Test whether we are at or near the stop
EndStopHit LocalSwitchEndstop::Stopped() const
{
	for (size_t i = 0; i < numPortsUsed; ++i)
	{
		if (ports[i].Read())
		{
			return EndStopHit::atStop;
		}
	}
	return EndStopHit::noStop;
}

// This is called to prime axis endstops
void LocalSwitchEndstop::Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers)
{
	// Decide whether we stop just the driver, just the axis, or everything
	stopAll = ((kin.GetConnectedAxes(GetAxis()) & ~MakeBitmap<AxesBitmap>(GetAxis())) != 0);
	numPortsLeftToTrigger = (numPortsUsed != axisDrivers.numDrivers) ? 1 : numPortsUsed;
	portsLeftToTrigger = LowestNBits<PortsBitmap>(numPortsUsed);
}

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
EndstopHitDetails LocalSwitchEndstop::CheckTriggered(bool goingSlow)
{
	EndstopHitDetails rslt;				// initialised by default constructor
	for (size_t i = 0; i < numPortsUsed; ++i)
	{
		if (IsBitSet(portsLeftToTrigger, i) && ports[i].Read())
		{
			rslt.axis = GetAxis();
			if (stopAll)
			{
				rslt.SetAction(EndstopHitAction::stopAll);
				if (GetAtHighEnd())
				{
					rslt.setAxisHigh = true;
				}
				else
				{
					rslt.setAxisLow = true;
				}
			}
			else if (numPortsLeftToTrigger == 1)
			{
				rslt.SetAction(EndstopHitAction::stopAxis);
				if (GetAtHighEnd())
				{
					rslt.setAxisHigh = true;
				}
				else
				{
					rslt.setAxisLow = true;
				}
			}
			else
			{
				rslt.SetAction(EndstopHitAction::stopDriver);
				rslt.internalUse = i;			// remember which port it is, for the call to Acknowledge
				rslt.driver = reprap.GetPlatform().GetAxisDriversConfig(GetAxis()).driverNumbers[i];
			}
			break;
		}
	}

	return rslt;
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Called from the step ISR.
// Return true if we have finished with this endstop or probe in this move.
bool LocalSwitchEndstop::Acknowledge(EndstopHitDetails what)
{
	switch (what.GetAction())
	{
	case EndstopHitAction::stopAll:
	case EndstopHitAction::stopAxis:
		return true;

	case EndstopHitAction::stopDriver:
		ClearBit(portsLeftToTrigger, what.internalUse);
		--numPortsLeftToTrigger;
		return false;

	default:
		return false;
	}
}

void LocalSwitchEndstop::AppendDetails(const StringRef& str)
{
	str.catf("%s on pin(s)", (ports[0].GetInvert()) ? "active low switch" :  "active high switch");
	for (size_t i = 0; i < numPortsUsed; ++i)
	{
		str.cat(' ');
		ports[i].AppendPinName(str);
	}
}

// End

