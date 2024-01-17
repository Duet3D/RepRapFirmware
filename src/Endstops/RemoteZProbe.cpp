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
#include <limits>

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
	GCodeResult rslt = GCodeResult::ok;
	if (isProbing && type == ZProbeType::scanningAnalog)
	{
		rslt = CanInterface::ChangeHandleThreshold(boardAddress, handle, targetAdcValue, state, reply.GetRef());
	}
	if (rslt == GCodeResult::ok)
	{
		rslt = CanInterface::ChangeHandleResponseTime(boardAddress, handle, (isProbing) ? ActiveProbeReportInterval : InactiveProbeReportInterval, state, reply.GetRef());
	}
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
	const uint16_t threshold = (type == ZProbeType::scanningAnalog) ? DefaultZProbeADValue : 0;		// nonzero threshold makes it an analog handle
	const GCodeResult rc = CanInterface::CreateHandle(boardAddress, h, pinNames.c_str(), threshold, ActiveProbeReportInterval, state, reply);
	if (rc < GCodeResult::error)						// don't set the handle unless it is valid, or we will get an error when this probe is deleted
	{
		handle = h;
		if (type == ZProbeType::scanningAnalog)
		{
			float dummyValue;
			(void)GetCalibratedReading(dummyValue);				// get an initial reading for the object model
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

GCodeResult RemoteZProbe::HandleG31(GCodeBuffer& gb, const StringRef& reply) /*override*/ THROWS(GCodeException)
{
	GCodeResult rslt = ZProbe::HandleG31(gb, reply);
	if (type == ZProbeType::scanningAnalog && gb.Seen('P') && (rslt == GCodeResult::ok || rslt <= GCodeResult::warning))
	{
		const GCodeResult rslt2 = CanInterface::ChangeHandleThreshold(boardAddress, handle, targetAdcValue, state, reply);
		if (rslt2 > rslt) { rslt = rslt2; }
	}
	return rslt;
}

GCodeResult RemoteZProbe::SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept
{
	//TODO
	reply.copy("Programming remote Z probes not supported");
	return GCodeResult::error;
}

// Functions used only with scanning Z probes

// Fetch a new reading and get the height compared to the nominal trigger height
GCodeResult RemoteZProbe::GetCalibratedReading(float& val) const noexcept
{
	String<1> dummyReply;
	const GCodeResult rslt = CanInterface::ReadRemoteHandles(boardAddress, handle, handle, GlobalScanningProbeCallback, CallbackParameter((void*)this), dummyReply.GetRef());
	if (rslt != GCodeResult::ok)
	{
		return rslt;
	}

	if (lastValue != 0 && lastValue != ScanningSensorBadReadingVal)		// zero and ScanningSensorBadReadingVal indicate errors reading the probe
	{
		const float reading = ConvertReadingToHeightDifference((int32_t)lastValue);
		if (!std::isnan(reading) && !std::isinf(reading) && fabsf(reading) < diveHeights[1])
		{
			val = reading;
			return GCodeResult::ok;
		}
	}
	return GCodeResult::error;
}

// Get the height reading corresponding to the latest reading received. Use to return the height n the object model.
float RemoteZProbe::GetLatestHeight() const noexcept
{
	if (lastValue != 0 && lastValue != ScanningSensorBadReadingVal)		// zero and ScanningSensorBadReadingVal indicate errors reading the probe
	{
		const float heightDiff = ConvertReadingToHeightDifference((int32_t)lastValue);
		if (!std::isnan(heightDiff) && !std::isinf(heightDiff))
		{
			// The nominal trigger height is -offsets[Z_AXIS] so we need t add this to heightDiff.
			// To get a more accurate height, subtract the amount by which the trigger height increases dur to temperature compensation.
			return heightDiff - offsets[Z_AXIS] - GetTriggerHeightCompensation();
		}
	}
	return 0.0;
}

// Tell the probe to calibrate its drive level
GCodeResult RemoteZProbe::CalibrateDriveLevel(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	uint32_t param;
	if (gb.Seen('S'))
	{
		const int32_t driveLevel = gb.GetLimitedIValue('S', -1, 31);
		if (driveLevel < 0)
		{
			param = CanMessageChangeInputMonitorNew::paramAutoCalibrateDriveLevelAndReport;
		}
		else
		{
			uint32_t offset = 0;
			bool dummy = false;
			gb.TryGetLimitedUIValue('R', offset, dummy, CanMessageChangeInputMonitorNew::maxParamOffset + 1);
			param = (offset << CanMessageChangeInputMonitorNew::paramOffsetShift) | (uint32_t)driveLevel;
		}
	}
	else
	{
		param = CanMessageChangeInputMonitorNew::paramReportDriveLevel;
	}
	uint8_t returnedDriveLevel;
	return CanInterface::SetHandleDriveLevel(boardAddress, handle, param, returnedDriveLevel, reply);
}

// Callback function for digital Z probes
void RemoteZProbe::HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool newState, uint32_t reading) noexcept
{
	if (src == boardAddress)
	{
		state = newState;
		lastValue = reading;
	}
}

// Process a remote reading that relates to this Z probe
void RemoteZProbe::UpdateRemoteReading(CanAddress src, uint8_t handleMinor, uint32_t reading) noexcept
{
	if (src == boardAddress)
	{
		lastValue = reading;
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
