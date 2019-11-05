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
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#if SUPPORT_CAN_EXPANSION
# include "CanId.h"
# include "CanMessageBuffer.h"
# include "CanMessageFormats.h"
# include "CAN/CanInterface.h"
#endif

// Switch endstop
LocalSwitchEndstop::LocalSwitchEndstop(uint8_t axis, EndStopPosition pos) : Endstop(axis, pos), numPortsUsed(0)
{
	// ports will be initialised automatically by the IoPort default constructor
}

LocalSwitchEndstop::~LocalSwitchEndstop()
{
	ReleasePorts();
}

// Release any local and remote ports we have allocated and set numPortsUsed to zero
void LocalSwitchEndstop::ReleasePorts()
{
	while (numPortsUsed != 0)
	{
		--numPortsUsed;
#if SUPPORT_CAN_EXPANSION
		const CanAddress bn = boardNumbers[numPortsUsed];
		if (bn != CanId::MasterAddress)
		{
			CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
			if (buf == nullptr)
			{
				reprap.GetPlatform().Message(ErrorMessage, "No CAN buffer\n");
			}
			else
			{
				const CanRequestId rid = CanInterface::AllocateRequestId(bn);
				auto msg = buf->SetupRequestMessage<CanMessageChangeInputMonitor>(rid, CanId::MasterAddress, bn);
				msg->handle.Set(RemoteInputHandle::typeEndstop, GetAxis(), numPortsUsed);
				msg->action = CanMessageChangeInputMonitor::actionDelete;
				String<StringLength100> reply;
				if (CanInterface::SendRequestAndGetStandardReply(buf, rid, reply.GetRef()) != GCodeResult::ok)
				{
					reply.cat('\n');
					reprap.GetPlatform().Message(ErrorMessage, reply.c_str());
				}
			}
		}
#endif
		ports[numPortsUsed].Release();
	}
}

GCodeResult LocalSwitchEndstop::Configure(GCodeBuffer& gb, const StringRef& reply, EndStopInputType inputType)
{
	String<StringLength50> portNames;
	if (!gb.GetReducedString(portNames.GetRef()))
	{
		reply.copy("Missing port name string");
		return GCodeResult::error;
	}

	return Configure(portNames.c_str(), reply, inputType);
}

GCodeResult LocalSwitchEndstop::Configure(const char *pinNames, const StringRef& reply, EndStopInputType inputType)
{
	ReleasePorts();

#if SUPPORT_CAN_EXPANSION
	activeLow = (inputType == EndStopInputType::activeLow);
#endif

	// Parse the string into individual port names
	size_t index = 0;
	while (numPortsUsed < MaxDriversPerAxis)
	{
		// Get the next port name
		String<StringLength50> pn;
		char c;
		while ((c = pinNames[index]) != 0 && c != '+')
		{
			pn.cat(c);
			++index;
		}

#if SUPPORT_CAN_EXPANSION
		const CanAddress boardAddress = IoPort::RemoveBoardAddress(pn.GetRef());
		boardNumbers[numPortsUsed] = boardAddress;
		if (boardAddress != CanId::MasterAddress)
		{
			CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
			if (buf == nullptr)
			{
				reprap.GetPlatform().Message(ErrorMessage, "No CAN buffer\n");
				ReleasePorts();
				return GCodeResult::error;
			}
			else
			{
				const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
				auto msg = buf->SetupRequestMessage<CanMessageCreateInputMonitor>(rid, CanId::MasterAddress, boardAddress);
				msg->handle.Set(RemoteInputHandle::typeEndstop, GetAxis(), numPortsUsed);
				msg->threshold = 0;
				msg->minInterval = MinimumSwitchReportInterval;
				SafeStrncpy(msg->pinName, pn.c_str(), ARRAY_SIZE(msg->pinName));
				buf->dataLength = msg->GetActualDataLength();

				uint8_t currentState;
				const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf, rid, reply, &currentState);
				if (rslt != GCodeResult::ok)
				{
					ReleasePorts();
					return rslt;
				}
				states[numPortsUsed] = (currentState != 0);
			}
		}
		else
#endif
		{
			// Try to allocate the port
			if (!ports[numPortsUsed].AssignPort(pn.c_str(), reply, PinUsedBy::endstop, PinAccess::read))
			{
				ReleasePorts();
				return GCodeResult::error;
			}
		}

		++numPortsUsed;
		if (c != '+')
		{
			break;
		}
		++index;					// skip the "+"
	}
	return GCodeResult::ok;
}

void LocalSwitchEndstop::Reconfigure(EndStopPosition pos, EndStopInputType inputType)
{
	SetAtHighEnd(pos == EndStopPosition::highEndStop);

#if SUPPORT_CAN_EXPANSION
	activeLow = (inputType == EndStopInputType::activeLow);
#endif

	for (IoPort& pp : ports)
	{
		pp.SetInvert(inputType == EndStopInputType::activeLow);
	}
}

EndStopInputType LocalSwitchEndstop::GetEndstopType() const
{
#if SUPPORT_CAN_EXPANSION
	return (activeLow)? EndStopInputType::activeLow : EndStopInputType::activeHigh;
#else
	return (ports[0].GetInvert()) ? EndStopInputType::activeLow : EndStopInputType::activeHigh;
#endif
}

// Test whether we are at or near the stop
EndStopHit LocalSwitchEndstop::Stopped() const
{
	for (size_t i = 0; i < numPortsUsed; ++i)
	{
		if (IsTriggered(i))
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
	if (portsLeftToTrigger != 0)
	{
		for (size_t i = 0; i < numPortsUsed; ++i)
		{
			if (IsBitSet(portsLeftToTrigger, i) && IsTriggered(i))
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
	str.catf("%s on pin(s)",
#if SUPPORT_CAN_EXPANSION
				(activeLow)
#else
				(ports[0].GetInvert())
#endif
				? "active low switch" : "active high switch");

	for (size_t i = 0; i < numPortsUsed; ++i)
	{
		str.cat(' ');
#if SUPPORT_CAN_EXPANSION
		if (boardNumbers[i] != CanId::MasterAddress)
		{
			str.catf("%u.some_pin", boardNumbers[i]);
		}
		else
#endif
		{
			ports[i].AppendPinName(str);
		}
	}
}

#if SUPPORT_CAN_EXPANSION

// Process a remote endstop input change that relates to this endstop
void LocalSwitchEndstop::HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool state)
{
	if (handleMinor < numPortsUsed && boardNumbers[handleMinor] == src)
	{
		states[handleMinor] = state;
	}
}

#endif

// End

