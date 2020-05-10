/*
 * RemoteZProbe.cpp
 *
 *  Created on: 14 Sep 2019
 *      Author: David
 */

#include "RemoteZProbe.h"

#if SUPPORT_CAN_EXPANSION

#include <CanMessageBuffer.h>
#include <CAN/CanInterface.h>
#include <CAN/CanMessageGenericConstructor.h>
#include <RepRap.h>
#include <Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

// Members of class RemoteZProbe
RemoteZProbe::~RemoteZProbe()
{
	String<StringLength100> reply;
	const GCodeResult rslt = CanInterface::DeleteHandle(boardAddress, handle, reply.GetRef());
	if (rslt != GCodeResult::ok)
	{
		reply.cat('\n');
		reprap.GetPlatform().Message(GetGenericMessageType(rslt), reply.c_str());
	}
}

GCodeResult RemoteZProbe::AppendPinNames(const StringRef& str) noexcept
{
	String<StringLength100> reply;
	const GCodeResult rslt = CanInterface::GetHandlePinName(boardAddress, handle, state, reply.GetRef());
	if (rslt == GCodeResult::ok)
	{
		str.cat(", input pin ");
		str.cat(reply.c_str());
	}
	else
	{
		str.copy(reply.c_str());
	}
	return rslt;
}

uint16_t RemoteZProbe::GetRawReading() const noexcept
{
	return (state) ? 1000 : 0;
}

void RemoteZProbe::SetProbing(bool isProbing) noexcept
{
	String<StringLength100> reply;
	const GCodeResult rslt = CanInterface::ChangeHandleResponseTime(boardAddress, handle, (isProbing) ? ActiveProbeReportInterval : InactiveProbeReportInterval, state, reply.GetRef());
	if (rslt != GCodeResult::ok)
	{
		reply.cat('\n');
		reprap.GetPlatform().Message(GetGenericMessageType(rslt), reply.c_str());
	}
}

// Create a remote Z probe
GCodeResult RemoteZProbe::Create(const StringRef& pinNames, const StringRef& reply) noexcept
{
	if (type != ZProbeType::unfilteredDigital && type != ZProbeType::blTouch)
	{
		reply.copy("M558: only Z probe types 8 and 9 are supported on expansion boards");
		return GCodeResult::error;
	}

	if (strchr(pinNames.c_str(), '+') != nullptr)
	{
		reply.copy("M558: output port not supported on expansion boards");
		return GCodeResult::error;
	}

	handle.Set(RemoteInputHandle::typeZprobe, number, 0);
	return CanInterface::CreateHandle(boardAddress, handle, pinNames.c_str(), 0, ActiveProbeReportInterval, state, reply);
}

// Configure an existing remote Z probe
GCodeResult RemoteZProbe::Configure(GCodeBuffer& gb, const StringRef &reply, bool& seen) THROWS(GCodeException)
{
	if (gb.Seen('P'))
	{
		seen = true;
		const uint32_t newType = gb.GetUIValue();
		if (newType != (uint32_t)ZProbeType::unfilteredDigital && newType != (uint32_t)ZProbeType::blTouch)
		{
			reply.copy("M558: only Z probe types 8 and 9 are supported on expansion boards");
			return GCodeResult::error;
		}
		type = (ZProbeType)newType;
	}

	// No other configuration items affect remote probes differently from others, so just call the base class function
	return ZProbe::Configure(gb, reply, seen);
}

GCodeResult RemoteZProbe::SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept
{
	//TODO
	reply.copy("Programming remote Z probes not supported");
	return GCodeResult::error;
}

void RemoteZProbe::HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool newState) noexcept
{
	if (src == boardAddress)
	{
		state = newState;
	}
}

#endif

// End
