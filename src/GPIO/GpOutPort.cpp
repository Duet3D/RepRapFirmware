/*
 * GpOutPort.cpp
 *
 *  Created on: 11 Feb 2020
 *      Author: David
 */

#include "GpOutPort.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <RepRap.h>
#include <Platform.h>

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
# include <CAN/CanMessageGenericConstructor.h>
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
		boardAddress == CanId::MasterAddress &&
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
		if (boardAddress != CanId::MasterAddress)
		{
			CanMessageGenericConstructor cons(M950GpioParams);
			cons.AddUParam('P', gpioNumber);
			cons.AddStringParam('C', NoPinName);
			if (cons.SendAndGetResponse(CanMessageType::m950Gpio, boardAddress, reply) != GCodeResult::ok)
			{
				reprap.GetPlatform().Message(WarningMessage, reply.c_str());
				reply.Clear();
			}
			boardAddress = CanId::MasterAddress;
		}
#endif
		port.Release();

		if (!seenFreq)
		{
			freq = (isServo) ? ServoRefreshFrequency : DefaultPinWritePwmFreq;
		}

		GCodeResult rslt;

#if SUPPORT_CAN_EXPANSION
		boardAddress = IoPort::RemoveBoardAddress(pinName.GetRef());
		if (boardAddress != CanId::MasterAddress)
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
			if (port.AssignPort(pinName.c_str(), reply, PinUsedBy::gpout, (isServo) ? PinAccess::servo : PinAccess::pwm))
			{
				rslt = GCodeResult::ok;
				port.SetFrequency(freq);
			}
			else
			{
				rslt = GCodeResult::error;
			}
		}

		reprap.StateUpdated();
		return rslt;
	}
	else if (seenFreq)
	{
#if SUPPORT_CAN_EXPANSION
		if (boardAddress != CanId::MasterAddress)
		{
			CanMessageGenericConstructor cons(M950GpioParams);
			cons.AddUParam('P', gpioNumber);
			cons.AddUParam('Q', freq);
			reprap.StateUpdated();
			return cons.SendAndGetResponse(CanMessageType::m950Gpio, boardAddress, reply);
		}
#endif
		port.SetFrequency(freq);
		reprap.StateUpdated();
	}
	else
	{
#if SUPPORT_CAN_EXPANSION
		if (boardAddress != CanId::MasterAddress)
		{
			CanMessageGenericConstructor cons(M950GpioParams);
			cons.AddUParam('P', gpioNumber);
			return cons.SendAndGetResponse(CanMessageType::m950Gpio, boardAddress, reply);
		}
#endif
		reply.printf("GPIO/servo port %" PRIu32, gpioNumber);
		port.AppendDetails(reply);
	}
	return GCodeResult::ok;
}

GCodeResult GpOutputPort::WriteAnalog(uint32_t gpioPortNumber, bool isServo, float pwm, const GCodeBuffer& gb, const StringRef& reply) noexcept
{
	lastPwm = pwm;
#if SUPPORT_CAN_EXPANSION
	if (boardAddress != CanId::MasterAddress)
	{
		return CanInterface::WriteGpio(boardAddress, gpioPortNumber, pwm, isServo, gb, reply);
	}
#endif
	port.WriteAnalog(pwm);
	return GCodeResult::ok;
}

#ifdef PCCB

// Function used to assign default GPOUT devices
void GpOutputPort::Assign(const char *pinName) noexcept
{
	String<1> dummy;
	port.AssignPort(pinName, dummy.GetRef(), PinUsedBy::gpout, PinAccess::pwm);
}

#endif

// End
