/*
 * Endstop.cpp
 *
 *  Created on: 4 Apr 2019
 *      Author: David
 */

#include "Endstop.h"
#include "ZProbe.h"
#include "RepRap.h"
#include "Platform.h"
#include "Movement/Kinematics/Kinematics.h"

// Endstop base class
DriversBitmap EndstopOrZProbe::stalledDrivers = 0;			// used to track which drivers are reported as stalled, for stall detect endstops and stall detect Z probes

Endstop::Endstop(uint8_t p_axis, EndStopPosition pos) : axis(p_axis), atHighEnd(pos == EndStopPosition::highEndStop)
{
}

// Switch endstop
SwitchEndstop::SwitchEndstop(uint8_t axis, EndStopPosition pos) : Endstop(axis, pos), numPortsUsed(0)
{
	// ports will be initialised automatically by the IoPort default constructor
}

SwitchEndstop::~SwitchEndstop()
{
	for (size_t i = 0; i < ARRAY_SIZE(ports); ++i)
	{
		ports[i].Release();
	}
}

bool SwitchEndstop::Configure(GCodeBuffer& gb, const StringRef& reply, EndStopInputType inputType)
{
	IoPort *portAddrs[MaxDriversPerAxis];
	PinAccess access[MaxDriversPerAxis];
	for (size_t i = 0; i < MaxDriversPerAxis; ++i)
	{
		portAddrs[i] = &ports[i];
		access[i] = PinAccess::read;
	}
	numPortsUsed = IoPort::AssignPorts(gb, reply, PinUsedBy::endstop, MaxDriversPerAxis, portAddrs, access);
	for (IoPort& pp : ports)
	{
		pp.ToggleInvert(inputType == EndStopInputType::activeLow);
	}
	return numPortsUsed != 0;
}

bool SwitchEndstop::Configure(const char *pinNames, const StringRef& reply, EndStopInputType inputType)
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

void SwitchEndstop::Reconfigure(EndStopPosition pos, EndStopInputType inputType)
{
	SetAtHighEnd(pos == EndStopPosition::highEndStop);
	for (IoPort& pp : ports)
	{
		pp.SetInvert(inputType == EndStopInputType::activeLow);
	}
}

EndStopInputType SwitchEndstop::GetEndstopType() const
{
	return (ports[0].GetInvert()) ? EndStopInputType::activeLow : EndStopInputType::activeHigh;
}

// Test whether we are at or near the stop
EndStopHit SwitchEndstop::Stopped() const
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

void SwitchEndstop::AppendPinNames(const StringRef& str)
{
	str.cat(" on pin(s)");
	for (size_t i = 0; i < numPortsUsed; ++i)
	{
		str.cat(' ');
		ports[i].AppendPinName(str);
	}
}

// This is called to prime axis endstops
void SwitchEndstop::Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers)
{
	// Decide whether we stop just the driver, just the axis, or everything
	stopAll = ((kin.GetConnectedAxes(GetAxis()) & ~MakeBitmap<AxesBitmap>(GetAxis())) != 0);
	numPortsLeftToTrigger = (numPortsUsed != axisDrivers.numDrivers) ? 1 : numPortsUsed;
	portsLeftToTrigger = LowestNBits<PortsBitmap>(numPortsUsed);
}

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
EndstopHitDetails SwitchEndstop::CheckTriggered(bool goingSlow)
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
bool SwitchEndstop::Acknowledge(EndstopHitDetails what)
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

// Stall detection endstop
StallDetectionEndstop::StallDetectionEndstop(uint8_t axis, EndStopPosition pos) : Endstop(axis, pos), driversMonitored(0)
{
}

// Test whether we are at or near the stop
EndStopHit StallDetectionEndstop::Stopped() const
{
	return ((GetStalledDrivers() & driversMonitored) != 0) ? EndStopHit::atStop : EndStopHit::noStop;
}

// This is called to prime axis endstops
void StallDetectionEndstop::Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers)
{
	// Find which drivers are relevant, Decide whether we stop just the driver, just the axis, or everything
	// Decide whether we stop just the driver, just the axis, or everything
	stopAll = (kin.GetConnectedAxes(GetAxis()) & ~MakeBitmap<AxesBitmap>(GetAxis())) != 0;
	numDriversLeft = axisDrivers.numDrivers;
	driversMonitored = axisDrivers.GetDriversBitmap();
}

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
// Note, the result will not necessarily be acted on because there may be a higher priority endstop!
EndstopHitDetails StallDetectionEndstop::CheckTriggered(bool goingSlow)
{
	EndstopHitDetails rslt;				// initialised by default constructor
	DriversBitmap relevantStalledDrivers = driversMonitored && GetStalledDrivers();
	if (relevantStalledDrivers != 0)
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
		else if (numDriversLeft == 1)
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
			rslt.driver = LowestSetBitNumber(relevantStalledDrivers);
		}
	}

	return rslt;
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Called from the step ISR.
// Return true if we have finished with this endstop or probe in this move.
bool StallDetectionEndstop::Acknowledge(EndstopHitDetails what)
{
	switch (what.GetAction())
	{
	case EndstopHitAction::stopAll:
	case EndstopHitAction::stopAxis:
		return true;

	case EndstopHitAction::stopDriver:
		ClearBit(driversMonitored, what.driver);
		--numDriversLeft;
		return false;

	default:
		return false;
	}
}

// Z probe endstop
ZProbeEndstop::ZProbeEndstop(uint8_t axis, EndStopPosition pos) : Endstop(axis, pos), zProbeNumber(0)
{
}

// Test whether we are at or near the stop
EndStopHit ZProbeEndstop::Stopped() const
{
	const ZProbe * const zp = reprap.GetPlatform().GetEndstops().GetZProbe(zProbeNumber);
	return (zp != nullptr) ? zp->Stopped() : EndStopHit::atStop;
}

// This is called to prime axis endstops
void ZProbeEndstop::Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers)
{
	// Decide whether we stop just the driver, just the axis, or everything
	stopAll = (kin.GetConnectedAxes(GetAxis()) & ~MakeBitmap<AxesBitmap>(GetAxis())) != 0;
}

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
EndstopHitDetails ZProbeEndstop::CheckTriggered(bool goingSlow)
{
	EndstopHitDetails rslt;				// initialised by default constructor
	switch (Stopped())
	{
	case EndStopHit::atStop:
		rslt.SetAction((stopAll) ? EndstopHitAction::stopAll : EndstopHitAction::stopAxis);
		rslt.axis = GetAxis();
		if (GetAtHighEnd())
		{
			rslt.setAxisHigh = true;
		}
		else
		{
			rslt.setAxisLow = true;
		}
		break;

	case EndStopHit::nearStop:
		if (!goingSlow)
		{
			rslt.SetAction(EndstopHitAction::reduceSpeed);
		}
		break;

	default:
		break;
	}
	return rslt;
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Called from the step ISR.
// Return true if we have finished with this endstop or probe in this move.
bool ZProbeEndstop::Acknowledge(EndstopHitDetails what)
{
	return what.GetAction() != EndstopHitAction::reduceSpeed;
}

// End
