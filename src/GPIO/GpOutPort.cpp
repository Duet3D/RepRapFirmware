/*
 * GpOutPort.cpp
 *
 *  Created on: 11 Feb 2020
 *      Author: David
 */

#include "GpOutPort.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
# include <CAN/CanMessageGenericConstructor.h>
# include <CanMessageGenericTables.h>
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(GpOutputPort, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(GpOutputPort, _condition,__VA_ARGS__)

constexpr ObjectModelTableEntry GpOutputPort::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	{ "pwm",	OBJECT_MODEL_FUNC(self->lastPwm, 2),	ObjectModelEntryFlags::live },
};

constexpr uint8_t GpOutputPort::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE(GpOutputPort)

#endif

// Return true if the port is not configured
bool GpOutputPort::IsUnused() const noexcept
{
	return
#if SUPPORT_CAN_EXPANSION
		boardAddress == CanInterface::GetCanAddress() &&
#endif
		!port.IsValid();
}

GCodeResult GpOutputPort::Configure(uint32_t gpioNumber, bool isServo, GCodeBuffer &gb, const StringRef &reply)
{
	PwmFrequency freq = 0;
	const bool seenFreq = gb.Seen('Q');
	if (seenFreq)
	{
		freq = gb.GetPwmFrequency();
	}

	if (gb.Seen('C'))
	{
		String<StringLength50> pinName;
		gb.GetReducedString(pinName.GetRef());

		// Remove any existing assignment
#if SUPPORT_CAN_EXPANSION
		if (boardAddress != CanInterface::GetCanAddress())
		{
			CanMessageGenericConstructor cons(M950GpioParams);
			cons.AddUParam('P', gpioNumber);
			cons.AddStringParam('C', NoPinName);
			if (cons.SendAndGetResponse(CanMessageType::m950Gpio, boardAddress, reply) != GCodeResult::ok)
			{
				reprap.GetPlatform().Message(WarningMessage, reply.c_str());
				reply.Clear();
			}
			boardAddress = CanInterface::GetCanAddress();
		}
#endif
		port.Release();

		if (!seenFreq)
		{
			freq = (isServo) ? DefaultServoRefreshFrequency : DefaultPinWritePwmFreq;
		}

		GCodeResult rslt;

#if SUPPORT_CAN_EXPANSION
		boardAddress = IoPort::RemoveBoardAddress(pinName.GetRef());
		if (boardAddress != CanInterface::GetCanAddress())
		{
			CanMessageGenericConstructor cons(M950GpioParams);
			cons.AddUParam('P', gpioNumber);
			cons.AddUParam('Q', freq);
			cons.AddUParam('S', (isServo) ? 1 : 0);
			cons.AddStringParam('C', pinName.c_str());
			rslt = cons.SendAndGetResponse(CanMessageType::m950Gpio, boardAddress, reply);
		}
		else
#endif
		{
			rslt = (port.AssignPort(pinName.c_str(), reply, PinUsedBy::gpout, (isServo) ? PinAccess::servo : PinAccess::pwm))
					? GCodeResult::ok
						: GCodeResult::error;
		}

		if (Succeeded(rslt))
		{
			port.SetFrequency(freq);			// we need to set the frequency even if it is a remote port because M280 uses it
			reprap.StateUpdated();
		}
		return rslt;
	}

	// If we get here then there was no port name parameter
	if (seenFreq)
	{
		GCodeResult rslt;
#if SUPPORT_CAN_EXPANSION
		if (boardAddress != CanInterface::GetCanAddress())
		{
			CanMessageGenericConstructor cons(M950GpioParams);
			cons.AddUParam('P', gpioNumber);
			cons.AddUParam('Q', freq);
			reprap.StateUpdated();
			rslt = cons.SendAndGetResponse(CanMessageType::m950Gpio, boardAddress, reply);
		}
		else
#endif
		{
			rslt = GCodeResult::ok;
		}

		if (Succeeded(rslt))
		{
			port.SetFrequency(freq);
			reprap.StateUpdated();
		}
		return rslt;
	}

	// If we get here then we have neither a port name nor a frequency, so just print the port details
#if SUPPORT_CAN_EXPANSION
	if (boardAddress != CanInterface::GetCanAddress())
	{
		CanMessageGenericConstructor cons(M950GpioParams);
		cons.AddUParam('P', gpioNumber);
		return cons.SendAndGetResponse(CanMessageType::m950Gpio, boardAddress, reply);
	}
#endif

	reply.printf("GPIO/servo port %" PRIu32, gpioNumber);
	port.AppendFullDetails(reply);
	return GCodeResult::ok;
}

GCodeResult GpOutputPort::WriteAnalog(uint32_t gpioPortNumber, bool isServo, float pwm, const GCodeBuffer& gb, const StringRef& reply) noexcept
{
	lastPwm = pwm;
#if SUPPORT_CAN_EXPANSION
	if (boardAddress != CanInterface::GetCanAddress())
	{
		return CanInterface::WriteGpio(boardAddress, gpioPortNumber, pwm, isServo, &gb, reply);
	}
#endif
	port.WriteAnalog(pwm);
	return GCodeResult::ok;
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult GpOutputPort::AssignFromRemote(uint32_t gpioPortNumber, const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool isServo;
	if (!parser.GetBoolParam('S', isServo))
	{
		isServo = false;
	}

	PwmFrequency freq;
	const bool seenFreq = parser.GetUintParam('Q', freq);
	if (!seenFreq)
	{
		freq = (isServo) ? DefaultServoRefreshFrequency : DefaultPinWritePwmFreq;
	}

	String<StringLength50> pinName;
	if (parser.GetStringParam('C', pinName.GetRef()))
	{
		// Creating or destroying a port
		boardAddress = CanInterface::GetCanAddress();
		const bool ok = port.AssignPort(pinName.c_str(), reply, PinUsedBy::gpout, (isServo) ? PinAccess::servo : PinAccess::pwm);
		if (ok && port.IsValid())
		{
			port.SetFrequency(freq);
		}
		return (ok) ? GCodeResult::ok : GCodeResult::error;
	}
	else
	{
		// Changing frequency, or reporting on a port
		if (!port.IsValid() || boardAddress != CanInterface::GetCanAddress())
		{
			reply.printf("Board %u does not have GPIO/servo port %" PRIu32, CanInterface::GetCanAddress(), gpioPortNumber);
			return GCodeResult::error;
		}

		if (seenFreq)
		{
			port.SetFrequency(freq);
		}
		else
		{
			reply.printf("GPIO/servo port %" PRIu32, gpioPortNumber);
			port.AppendFullDetails(reply);
		}
		return GCodeResult::ok;
	}
}

void GpOutputPort::WriteAnalog(float pwm) noexcept
{
	if (boardAddress == CanInterface::GetCanAddress())
	{
		lastPwm = pwm;
		port.WriteAnalog(pwm);
	}
}

#endif

#ifdef PCCB

// Function used to assign default GPOUT devices
void GpOutputPort::Assign(const char *pinName) noexcept
{
	String<1> dummy;
	port.AssignPort(pinName, dummy.GetRef(), PinUsedBy::gpout, PinAccess::pwm);
}

#endif

// End
