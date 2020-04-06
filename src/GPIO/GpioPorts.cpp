/*
 * GpioPorts.cpp
 *
 *  Created on: 11 Feb 2020
 *      Author: David
 */

#include "GpioPorts.h"
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
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(GpInputPort, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(GpInputPort, _condition,__VA_ARGS__)

constexpr ObjectModelTableEntry GpInputPort::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. sensors members
	{ "configured",			OBJECT_MODEL_FUNC(!self->IsUnused()),							ObjectModelEntryFlags::none },
	{ "value",				OBJECT_MODEL_FUNC_IF(!self->IsUnused(), self->GetState()),		ObjectModelEntryFlags::live },
};

constexpr uint8_t GpInputPort::objectModelTableDescriptor[] = { 1, 2 };

DEFINE_GET_OBJECT_MODEL_TABLE(GpInputPort)

#endif

bool GpInputPort::GetState() const noexcept
{
	// Temporary implementation until we use interrupts to track input pin state changes
#if SUPPORT_CAN_EXPANSION
	if (boardAddress != CanId::MasterAddress)
	{
		return currentState;
	}
#endif
	return port.Read();
}

// Return true if the port is not configured
bool GpInputPort::IsUnused() const noexcept
{
	return
#if SUPPORT_CAN_EXPANSION
		boardAddress == CanId::MasterAddress &&
#endif
		!port.IsValid();
}

GCodeResult GpInputPort::Configure(uint32_t gpinNumber, GCodeBuffer &gb, const StringRef &reply)
{
	if (gb.Seen('C'))
	{
		String<StringLength50> pinName;
		gb.GetReducedString(pinName.GetRef());

		// Remove any existing assignment
#if SUPPORT_CAN_EXPANSION
		if (boardAddress != CanId::MasterAddress)
		{
			const GCodeResult rslt = CanInterface::DeleteHandle(boardAddress, handle, reply);
			if (rslt != GCodeResult::ok)
			{
				reply.cat('\n');
				const MessageType mtype = (rslt == GCodeResult::warning) ? AddWarning(gb.GetResponseMessageType()) : AddError(gb.GetResponseMessageType());
				reprap.GetPlatform().Message(mtype, reply.c_str());
				reply.Clear();
			}
			boardAddress = CanId::MasterAddress;
		}
#endif
		port.Release();
		currentState = false;

		GCodeResult rslt;

#if SUPPORT_CAN_EXPANSION
		const CanAddress newBoard = IoPort::RemoveBoardAddress(pinName.GetRef());
		if (newBoard != CanId::MasterAddress)
		{
			handle.Set(RemoteInputHandle::typeGpIn, gpinNumber, 0);
			rslt = CanInterface::CreateHandle(newBoard, handle, pinName.c_str(), 0, MinimumGpinReportInterval, currentState, reply);
			if (rslt == GCodeResult::ok)
			{
				boardAddress = newBoard;
			}
			else
			{
				currentState = false;
			}
		}
		else
#endif
		{
			if (port.AssignPort(pinName.c_str(), reply, PinUsedBy::gpin, PinAccess::read))
			{
				currentState = port.Read();
				rslt = GCodeResult::ok;
			}
			else
			{
				rslt = GCodeResult::error;
			}
		}

		reprap.InputsUpdated();
		return rslt;
	}
	else
	{
		// Report the pin details
#if SUPPORT_CAN_EXPANSION
		if (boardAddress != CanId::MasterAddress)
		{
			const GCodeResult rslt = CanInterface::GetHandlePinName(boardAddress, handle, currentState, reply);
			if (rslt != GCodeResult::ok)
			{
				return rslt;
			}
			reply.Prepend("Pin ");
		}
		else
#endif
		{
			reply.copy("Pin ");
			port.AppendPinName(reply);
		}
		reply.catf(", active: %s", (GetState()) ? "true" : "false");
	}
	return GCodeResult::ok;
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

		// GPOut pins are not yet in the object model
		// reprap.OutputsUpdated();
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
			// reprap.OutputsUpdated();
			return cons.SendAndGetResponse(CanMessageType::m950Gpio, boardAddress, reply);
		}
#endif
		port.SetFrequency(freq);
		// reprap.OutputsUpdated();
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

GCodeResult GpOutputPort::WriteAnalog(uint32_t gpioPortNumber, bool isServo, float pwm, const GCodeBuffer& gb, const StringRef& reply) const noexcept
{
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
