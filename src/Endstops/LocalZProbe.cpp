/*
 * LocalZProbe.cpp
 *
 *  Created on: 14 Sep 2019
 *      Author: David
 */

#include "LocalZProbe.h"

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>

#include <AnalogIn.h>
using
#if SAME5x
	AnalogIn
#else
	LegacyAnalogIn
#endif
	::AdcBits;

// Members of class LocalZProbe
LocalZProbe::~LocalZProbe() noexcept
{
	inputPort.Release();
	modulationPort.Release();
}

GCodeResult LocalZProbe::Configure(GCodeBuffer& gb, const StringRef &reply, bool& seen) THROWS(GCodeException)
{
	// We must get and set the Z probe type first before setting the dive height etc. because different probe types need different port settings
	if (gb.Seen('P'))
	{
		seen = true;
		type = (ZProbeType)gb.GetUIValue();
	}

	// Determine the required pin access
	PinAccess access[2];
	switch (type)
	{
	case ZProbeType::analog:
	case ZProbeType::dumbModulated:
		access[0] = PinAccess::readAnalog;
		access[1] = PinAccess::write1;
		break;

	case ZProbeType::alternateAnalog:
		access[0] = PinAccess::readAnalog;
		access[1] = PinAccess::write0;
		break;

	default:
		access[0] = PinAccess::read;
		access[1] = PinAccess::write0;
		break;
	}

	if (gb.Seen('C'))										// input channel
	{
		seen = true;
		IoPort* const ports[] = { &inputPort, &modulationPort };

		if (!IoPort::AssignPorts(gb, reply, PinUsedBy::zprobe, 2, ports, access))
		{
			return GCodeResult::error;
		}
	}
	else if (seen)											// the type may have changed, so set the correct pin modes
	{
		(void)inputPort.SetMode(access[0]);
		(void)modulationPort.SetMode(access[1]);
	}

	if (seen)
	{
		reprap.GetPlatform().InitZProbeFilters();
	}

	return ZProbe::Configure(gb, reply, seen);
}

#if ALLOCATE_DEFAULT_PORTS

bool LocalZProbe::AssignPorts(const char* pinNames, const StringRef& reply) noexcept
{
	IoPort* const ports[] = { &inputPort, &modulationPort };
	const PinAccess access[] = { PinAccess::read, PinAccess::write0 };
	return IoPort::AssignPorts(pinNames, reply, PinUsedBy::zprobe, 2, ports, access);
}

#endif

// This is called by the tick ISR to get the raw Z probe reading to feed to the filter
uint16_t LocalZProbe::GetRawReading() const noexcept
{
	constexpr uint16_t MaxReading = 1000;
	switch (type)
	{
	case ZProbeType::analog:
	case ZProbeType::dumbModulated:
	case ZProbeType::alternateAnalog:
		return min<uint16_t>(inputPort.ReadAnalog() >> (AdcBits - 10), MaxReading);

	case ZProbeType::digital:
	case ZProbeType::unfilteredDigital:
	case ZProbeType::blTouch:
		return (inputPort.ReadDigital()) ? MaxReading : 0;

	default:
		return MaxReading;
	}
}

bool LocalZProbe::SetProbing(bool isProbing) noexcept
{
	// For Z probe types other than 1/2/3 and bltouch we set the modulation pin high at the start of a probing move and low at the end
	// Don't do this for bltouch because on the Maestro, the MOD pin is normally used as the servo control output
	if (type > ZProbeType::alternateAnalog && type != ZProbeType::blTouch)
	{
		modulationPort.WriteDigital(isProbing);
	}
	return true;
}

GCodeResult LocalZProbe::AppendPinNames(const StringRef& str) noexcept
{
	if (type != ZProbeType::zMotorStall && type != ZProbeType::none)
	{
		str.cat(", input pin ");
		inputPort.AppendPinName(str);
		str.cat(", output pin ");
		modulationPort.AppendPinName(str);
	}
	return GCodeResult::ok;
}

// Z probe programming functions

/*static*/ void LocalZProbe::TimerInterrupt(CallbackParameter param) noexcept
{
	static_cast<LocalZProbe*>(param.vp)->Interrupt();
}

// Kick off sending some program bytes
GCodeResult LocalZProbe::SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept
{
	timer.CancelCallback();										// make quite certain that this timer isn't already pending

	for (size_t i = 0; i < len; ++i)
	{
		progBytes[i] = (uint8_t)zProbeProgram[i];
	}
	numBytes = len;
	bytesSent = 0;
	bitsSent = 0;
	bitTime = StepTimer::GetTickRate()/bitsPerSecond;

	modulationPort.WriteDigital(false);				// start with 2 bits of zero
	startTime = StepTimer::GetTimerTicks();
	timer.SetCallback(LocalZProbe::TimerInterrupt, CallbackParameter(this));
	timer.ScheduleCallback(startTime + 2 * bitTime);

	// TODO wait until all bytes sent or some error occurs, but for now we return immediately
	return GCodeResult::ok;
}

void LocalZProbe::Interrupt() noexcept
{
	if (timer.ScheduleCallbackFromIsr())
	{
		// The data format is:
		// [0 0 1 0 b7 b6 b5 b4 /b4 b3 b2 b1 b0 /b0] repeated for each byte, where /b4 = inverse of b4, /b0 = inverse of b0
		// After the last byte the line returns to 0
		bool nextBit;
		switch(bitsSent++)
		{
		case 0:		// We sent 00, now send 1
			nextBit = true;
			break;

		case 1:	// We sent 001, now send 0
		default:
			nextBit = false;
			break;

		case 2:
		case 3:
		case 4:
		case 5:
			nextBit = (((progBytes[bytesSent] >> (10 - bitsSent)) & 1) != 0);
			break;

		case 6:
			nextBit = (((progBytes[bytesSent] >> 4) & 1) == 0);
			break;

		case 7:
		case 8:
		case 9:
		case 10:
			nextBit = (((progBytes[bytesSent] >> (11 - bitsSent)) & 1) != 0);
			break;

		case 11:
			nextBit = ((progBytes[bytesSent] & 1) == 0);
			break;

		case 12:		// We sent 0010 + 10 data bits, now send 0
			nextBit = false;
			bitsSent = 0;
			++bytesSent;
			break;
		}

		modulationPort.WriteDigital(nextBit);
		if (bytesSent < numBytes)
		{
			timer.ScheduleCallbackFromIsr(startTime + ((bytesSent * 14) + bitsSent + 2) * bitTime);
			return;
		}

		bytesSent = numBytes = 0;
	}
}

// End
