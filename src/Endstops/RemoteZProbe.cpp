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
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

static void GlobalScanningProbeCallback(CallbackParameter param, RemoteInputHandle h, uint32_t val) noexcept
{
	((RemoteZProbe*)param.vp)->ScanningProbeCallback(h, val);
}

// Members of class RemoteZProbe
RemoteZProbe::~RemoteZProbe()
{
	if (handle.IsValid())
	{
		String<StringLength100> reply;
		const GCodeResult rslt = CanInterface::DeleteHandle(boardAddress, handle, reply.GetRef());
		if (rslt != GCodeResult::ok)
		{
			reply.cat('\n');
			reprap.GetPlatform().Message(GetGenericMessageType(rslt), reply.c_str());
		}
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

// Get the raw reading. Not used with scanning Z probes except for reporting in the object model.
uint32_t RemoteZProbe::GetRawReading() const noexcept
{
	return (type == ZProbeType::scanningAnalog) ? lastValue
			: (state) ? 1000 : 0;
}

bool RemoteZProbe::SetProbing(bool isProbing) noexcept
{
	String<StringLength100> reply;
	const GCodeResult rslt = CanInterface::ChangeHandleResponseTime(boardAddress, handle, (isProbing) ? ActiveProbeReportInterval : InactiveProbeReportInterval, state, reply.GetRef());
	if (rslt != GCodeResult::ok)
	{
		reply.cat('\n');
		reprap.GetPlatform().Message(GetGenericMessageType(rslt), reply.c_str());
	}
	return rslt == GCodeResult::ok;
}

// Create a remote Z probe
GCodeResult RemoteZProbe::Create(const StringRef& pinNames, const StringRef& reply) noexcept
{
	if (type != ZProbeType::unfilteredDigital && type != ZProbeType::blTouch && type != ZProbeType::scanningAnalog)
	{
		reply.copy("only Z probe types 8, 9 and 11 are supported on expansion boards");
		return GCodeResult::error;
	}

	if (strchr(pinNames.c_str(), '+') != nullptr)
	{
		reply.copy("expansion boards do not support Z probe output ports");
		return GCodeResult::error;
	}

	RemoteInputHandle h;
	h.Set(RemoteInputHandle::typeZprobe, number, 0);
	const uint16_t threshold = (type == ZProbeType::scanningAnalog) ? 1 : 0;		// nonzero threshold makes it an analog handle
	const GCodeResult rc = CanInterface::CreateHandle(boardAddress, h, pinNames.c_str(), threshold, ActiveProbeReportInterval, state, reply);
	if (rc < GCodeResult::error)						// don't set the handle unless it is valid, or we will get an error when this probe is deleted
	{
		handle = h;
		if (type == ZProbeType::scanningAnalog)
		{
			(void)GetCalibratedReading();				// get an initial reading for the object model
		}
	}
	return rc;
}

// Configure an existing remote Z probe
GCodeResult RemoteZProbe::Configure(GCodeBuffer& gb, const StringRef &reply, bool& seen) THROWS(GCodeException)
{
	if (gb.Seen('P'))
	{
		seen = true;
		const uint32_t newType = gb.GetUIValue();
		if (newType != (uint32_t)ZProbeType::unfilteredDigital && newType != (uint32_t)ZProbeType::blTouch && newType != (uint32_t)ZProbeType::scanningAnalog)
		{
			reply.copy("only Z probe types 8, 9 and 11 are supported on expansion boards");
			return GCodeResult::error;
		}

		// We don't support changing the type between analog and digital
		if (newType != (uint32_t)type && (newType == (uint32_t)ZProbeType::scanningAnalog || type == ZProbeType::scanningAnalog))
		{
			reply.copy("changing the type of this Z probe in this way is not supported");
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

// Functions used only with scanning Z probes
// Get the height compared to the nominal trigger height
float RemoteZProbe::GetCalibratedReading() const noexcept
{
	String<1> dummyReply;
	if (CanInterface::ReadRemoteHandles(boardAddress, handle, handle, GlobalScanningProbeCallback, CallbackParameter((void*)this), dummyReply.GetRef()) == GCodeResult::ok)
	{
		return ConvertReadingToHeightDifference((int32_t)lastValue);
	}
	return 0.0;
}

// Callback function for digital Z probes
void RemoteZProbe::HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool newState) noexcept
{
	if (src == boardAddress)
	{
		state = newState;
	}
}

// Callback function for scanning analog Z probes
void RemoteZProbe::ScanningProbeCallback(RemoteInputHandle h, uint32_t val) noexcept
{
	if (h == handle)
	{
		lastValue = val;
	}
}

#endif

// End
