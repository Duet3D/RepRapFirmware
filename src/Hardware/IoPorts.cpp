/*
 * IoPort.cpp
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#include "IoPorts.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#ifdef DUET_NG
# include <DuetNG/DueXn.h>
#endif

#include <AnalogOut.h>
#include <Interrupts.h>
#include <AnalogIn.h>
using
#if SAME5x
	AnalogIn
#else
	LegacyAnalogIn
#endif
	::AdcBits;

#if SUPPORT_CAN_EXPANSION
# include <CanId.h>
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CAN/CanInterface.h>
#endif

// Read a port name parameter and assign some ports. Caller must call gb.Seen() with the appropriate letter and get 'true' returned before calling this.
// Return the number of ports allocated, or 0 if there was an error with the error message in 'reply'.
/*static*/ size_t IoPort::AssignPorts(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort* const ports[], const PinAccess access[]) THROWS(GCodeException)
{
	// Get the full port names string
	String<StringLength100> portNames;				// 50 characters wasn't always enough when passing 4 duex endstop input names in a M574 command
	gb.GetReducedString(portNames.GetRef());
	return AssignPorts(portNames.c_str(), reply, neededFor, numPorts, ports, access);
}

// Read a port name parameter and assign one port. Caller must call gb.Seen() with the appropriate letter and get 'true' returned before calling this.
// If successful, return true; else return false with the error message in 'reply'.
bool IoPort::AssignPort(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, PinAccess access) THROWS(GCodeException)
{
	IoPort* const p = this;
	return AssignPorts(gb, reply, neededFor, 1, &p, &access) == 1;
}

// Try to assign ports, returning the number of ports successfully assigned
/*static*/ size_t IoPort::AssignPorts(const char* pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort* const ports[], const PinAccess access[]) noexcept
{
	// Release any existing assignments
	for (size_t i = 0; i < numPorts; ++i)
	{
		ports[i]->Release();
	}

	// Parse the string into individual port names
	size_t index = 0;
	for (size_t i = 0; i < numPorts; ++i)
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
		const CanAddress boardAddress = RemoveBoardAddress(pn.GetRef());
		if (boardAddress != CanInterface::GetCanAddress())
		{
			reply.lcat("Port must be on main board");
#else
		if (!RemoveBoardAddress(pn.GetRef()))
		{
			reply.lcat("Board address of port must be 0");
#endif
			for (size_t j = 0; j < i; ++j)
			{
				ports[j]->Release();
			}
			return 0;
		}

		// Try to allocate the port
		if (!ports[i]->Allocate(pn.c_str(), reply, neededFor, access[i]))
		{
			for (size_t j = 0; j < i; ++j)
			{
				ports[j]->Release();
			}
			return 0;
		}

		if (c != '+')
		{
			return i + 1;
		}
		++index;					// skip the "+"
	}
	return numPorts;
}

bool IoPort::AssignPort(const char* pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access) noexcept
{
	IoPort* const p = this;
	return AssignPorts(pinName, reply, neededFor, 1, &p, &access) == 1;
}

/*static*/ const char* IoPort::TranslatePinAccess(PinAccess access) noexcept
{
	switch (access)
	{
	case PinAccess::read:			return "digital read";
	case PinAccess::readWithPullup_InternalUseOnly:	return "digital read (pullup resistor enabled)";
	case PinAccess::readAnalog:		return "analog read";
	case PinAccess::write0:			return "write (initially low)";
	case PinAccess::write1:			return "write (initially high)";
	case PinAccess::pwm:			return "write PWM";
	case PinAccess::servo:			return "servo write";
	default:						return "[unknown]";
	}
}

// Members of class IoPort

PinUsedBy IoPort::portUsedBy[NumNamedPins];
int8_t IoPort::logicalPinModes[NumNamedPins];	// what mode each logical pin is set to - would ideally be class PinMode not int8_t

/*static*/ void IoPort::Init() noexcept
{
	for (PinUsedBy& p : portUsedBy)
	{
		p = PinUsedBy::unused;
	}
	for (int8_t& p : logicalPinModes)
	{
		p = PIN_MODE_NOT_CONFIGURED;
	}
}

IoPort::IoPort() noexcept : logicalPin(NoLogicalPin), hardwareInvert(false), totalInvert(false), isSharedInput(false)
{
}

void IoPort::Release() noexcept
{
	if (IsValid() && !isSharedInput)
	{
#ifdef __LPC17xx__
		// Release PWM/Servo from pin if needed
		if (logicalPinModes[logicalPin] == OUTPUT_SERVO_HIGH || logicalPinModes[logicalPin] == OUTPUT_SERVO_LOW)
		{
			ReleaseServoPin(GetPinNoCheck());
		}
		if (logicalPinModes[logicalPin] == OUTPUT_PWM_HIGH || logicalPinModes[logicalPin] == OUTPUT_PWM_LOW)
		{
			ReleasePWMPin(GetPinNoCheck());
		}
#endif
		detachInterrupt(GetPinNoCheck());
		portUsedBy[logicalPin] = PinUsedBy::unused;
		logicalPinModes[logicalPin] = PIN_MODE_NOT_CONFIGURED;
	}
	logicalPin = NoLogicalPin;
	hardwareInvert = totalInvert = false;
}

// Attach an interrupt to the pin. Not permitted if we allocated the pin in shared input mode.
bool IoPort::AttachInterrupt(StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) const noexcept
{
	return IsValid() && !isSharedInput && attachInterrupt(GetPinNoCheck(), callback, mode, param);
}

void IoPort::DetachInterrupt() const noexcept
{
	if (IsValid() && !isSharedInput)
	{
		detachInterrupt(GetPinNoCheck());
	}
}

#if SAME5x

bool IoPort::SetAnalogCallback(AnalogInCallbackFunction fn, CallbackParameter cbp, uint32_t ticksPerCall) noexcept
{
	return AnalogIn::SetCallback(GetAnalogChannel(), fn, cbp, ticksPerCall, false);
}

#endif

// Allocate the specified logical pin, returning true if successful
bool IoPort::Allocate(const char *pn, const StringRef& reply, PinUsedBy neededFor, PinAccess access) noexcept
{
	Release();

	bool inverted = false;
	for (;;)
	{
		if (*pn == '!')
		{
			inverted = !inverted;
		}
		else if (*pn == '^')
		{
			if (access == PinAccess::read)
			{
				access = PinAccess::readWithPullup_InternalUseOnly;
			}
		}
		else if (*pn == '*')
		{
			alternateConfig = true;
		}
		else
		{
			break;
		}
		++pn;
	}

	const char *const fullPinName = pn;			// the full pin name less the inversion and pullup flags

#if SUPPORT_CAN_EXPANSION
	if (isdigit(*pn))
	{
		const uint32_t expansionNumber = StrToU32(pn, &pn);
		if (*pn != '.')
		{
			reply.printf("Bad pin name '%s'", fullPinName);
			return false;
		}
		if (expansionNumber != 0)
		{
			reply.printf("Pin '%s': only main board pins allowed here", fullPinName);
			return false;
		}
	}
#endif

	LogicalPin lp;
	bool hwInvert;
	if (!LookupPinName(pn, lp, hwInvert))
	{
		reply.printf("Unknown pin name '%s'", fullPinName);
		return false;
	}

	if (lp != NoLogicalPin)					// if not assigning "nil"
	{
		bool doSetMode = true;
		if (portUsedBy[lp] == PinUsedBy::unused || (portUsedBy[lp] == PinUsedBy::temporaryInput && neededFor != PinUsedBy::temporaryInput))
		{
			portUsedBy[lp] = neededFor;
		}
		else
		{
			const PinMode pm = (PinMode)logicalPinModes[lp];
			if (   neededFor != PinUsedBy::temporaryInput
				|| (pm != INPUT && pm != INPUT_PULLUP)
			   )
			{
				reply.printf("Pin '%s' is not free", fullPinName);
				return false;
			}
			doSetMode = false;
		}
		logicalPin = lp;
		hardwareInvert = hwInvert;
		isSharedInput = (neededFor == PinUsedBy::temporaryInput);
		SetInvert(inverted);

		if (doSetMode && !SetMode(access))
		{
			reply.printf("Pin '%s' does not support mode %s", fullPinName, TranslatePinAccess(access));
			Release();
			return false;
		}
	}

	return true;
}

// Set the specified pin mode returning true if successful
bool IoPort::SetMode(PinAccess access) noexcept
{
	if (!IsValid())
	{
		return false;
	}

	// Check that the pin mode has been defined suitably
	PinMode desiredMode;
	switch (access)
	{
	case PinAccess::write0:
		desiredMode = (totalInvert) ? OUTPUT_HIGH : OUTPUT_LOW;
		break;
	case PinAccess::write1:
		desiredMode = (totalInvert) ? OUTPUT_LOW : OUTPUT_HIGH;
		break;
#ifdef __LPC17xx__
	case PinAccess::pwm:
		desiredMode = (totalInvert) ? OUTPUT_PWM_HIGH : OUTPUT_PWM_LOW;
		break;
	case PinAccess::servo:
		desiredMode = (totalInvert) ? OUTPUT_SERVO_HIGH : OUTPUT_SERVO_LOW;
		break;
#else
	case PinAccess::pwm:
	case PinAccess::servo:
		desiredMode = (totalInvert) ? OUTPUT_PWM_HIGH : OUTPUT_PWM_LOW;
		break;
#endif
	case PinAccess::readAnalog:
		desiredMode = AIN;
		break;
	case PinAccess::readWithPullup_InternalUseOnly:
		desiredMode = INPUT_PULLUP;
		break;
	case PinAccess::read:
	default:
		desiredMode = INPUT;
		break;
	}

	if (logicalPinModes[logicalPin] != (int8_t)desiredMode)
	{
		const AnalogChannelNumber chan = PinToAdcChannel(GetPinNoCheck());
		if (chan != NO_ADC)
		{
			if (access == PinAccess::readAnalog)
			{
				IoPort::SetPinMode(GetPinNoCheck(), AIN);		// SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
				AnalogInEnableChannel(chan, true);
				logicalPinModes[logicalPin] = (int8_t)desiredMode;
				return true;
			}
			else
			{
				AnalogInEnableChannel(chan, false);
			}
		}
		else if (access == PinAccess::readAnalog)
		{
			return false;
		}
#ifdef __LPC17xx__
		if (access == PinAccess::servo)
		{
			if (!IsServoCapable(GetPinNoCheck)) //check that we have slots free to provide Servo
			{
				return false;
			}
		}
		else if (access == PinAccess::pwm)
		{
			if (!IsPwmCapable(GetPinNoCheck)) //Check if there is enough slots free for PWM
			{
				return false;
			}
		}
#endif
		IoPort::SetPinMode(GetPinNoCheck(), desiredMode);
		logicalPinModes[logicalPin] = (int8_t)desiredMode;
	}
	return true;
}

bool IoPort::GetInvert() const noexcept
{
	return (hardwareInvert) ? !totalInvert : totalInvert;
}

void IoPort::SetInvert(bool pInvert) noexcept
{
	totalInvert = (hardwareInvert) ? !pInvert : pInvert;
}

void IoPort::ToggleInvert(bool pInvert) noexcept
{
	if (pInvert)
	{
		totalInvert = !totalInvert;
	}
}

void IoPort::AppendBasicDetails(const StringRef& str) const noexcept
{
	if (IsValid())
	{
		str.catf(" pin ");
		AppendPinName(str);
		if (logicalPinModes[logicalPin] == INPUT_PULLUP)
		{
			str.cat(", pullup enabled");
		}
		else if (logicalPinModes[logicalPin] == INPUT)
		{
			str.cat(", pullup disabled");
		}
	}
	else
	{
		str.cat(" has no pin");
	}
}

// Append the names of the pin to a string, picking only those that have the correct hardware invert status
void IoPort::AppendPinName(const StringRef& str) const noexcept
{
	if (IsValid())
	{
#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			str.catf("%u.", CanInterface::GetCanAddress());
		}
#endif
		if (GetInvert())
		{
			str.cat('!');
		}
		const size_t insertPoint = str.strlen();
		const char *pn = PinTable[logicalPin].GetNames();
		unsigned int numPrinted = 0;
		do
		{
			bool inverted = (*pn == '!');
			if (inverted)
			{
				++pn;
			}
			if (hardwareInvert)
			{
				inverted = !inverted;
			}
			if (inverted)
			{
				// skip this one
				while (*pn != 0 && *pn != ',')
				{
					++pn;
				}
			}
			else
			{
				// Include this one
				if (numPrinted != 0)
				{
					str.cat(',');
				}
				++numPrinted;
				while (*pn != 0 && *pn != ',')
				{
					str.cat(*pn);
					++pn;
				}
			}

		} while (*pn++ == ',');

		if (numPrinted > 1)
		{
			str.Insert(insertPoint, '(');
			str.cat(')');
		}
	}
	else
	{
		str.cat(NoPinName);
	}
}

/*static*/ void IoPort::AppendPinNames(const StringRef& str, size_t numPorts, const IoPort * const ports[]) noexcept
{
	for (size_t i = 0; i < numPorts; ++i)
	{
		if (ports[i]->IsValid())
		{
			if (i != 0)
			{
				str.cat('+');
			}
			ports[i]->AppendPinName(str);
		}
		else
		{
			if (i == 0)
			{
				str.cat(NoPinName);
			}
			break;
		}
	}
}

void IoPort::WriteDigital(bool high) const noexcept
{
	if (IsValid())
	{
		WriteDigital(GetPinNoCheck(), (totalInvert) ? !high : high);
	}
}

Pin IoPort::GetPin() const noexcept
{
	return (IsValid()) ? GetPinNoCheck() : NoPin;
}

bool IoPort::ReadDigital() const noexcept
{
	if (IsValid())
	{
		const bool b = ReadPin(GetPinNoCheck());
		return (totalInvert) ? !b : b;
	}
	return false;
}

uint16_t IoPort::ReadAnalog() const noexcept
{
	const uint16_t val = AnalogInReadChannel(GetAnalogChannel());
	return (totalInvert) ? ((1u << AdcBits) - 1) - val : val;
}

#if SUPPORT_CAN_EXPANSION
// Remove the board address from a port name string and return it
/*static*/ CanAddress IoPort::RemoveBoardAddress(const StringRef& portName) noexcept
#else
// Remove the board address if present, returning true if it was zero or not present
/*static*/ bool IoPort::RemoveBoardAddress(const StringRef& portName) noexcept
#endif
{
	size_t prefix = 0;
	while (portName[prefix] == '!' || portName[prefix] == '^' || portName[prefix] == '*')
	{
		++prefix;
	}

	size_t numToSkip = prefix;
	unsigned int boardAddress = 0;
	while (isdigit(portName[numToSkip]))
	{
		boardAddress = (boardAddress * 10) + (portName[numToSkip] - '0');
		++numToSkip;
	}
#if SUPPORT_CAN_EXPANSION
	if (numToSkip != prefix && portName[numToSkip] == '.' && boardAddress <= CanId::MaxCanAddress)
	{
		portName.Erase(prefix, numToSkip - prefix + 1);			// remove the board address prefix
		return (CanAddress)boardAddress;
	}
	return CanInterface::GetCanAddress();
#else
	if (numToSkip != prefix && portName[numToSkip] == '.')
	{
		if (boardAddress != 0)
		{
			return false;
		}
		portName.Erase(prefix, numToSkip - prefix + 1);			// remove the board address prefix
	}
	return true;
#endif
}

// Low level pin access methods

/*static*/ void IoPort::SetPinMode(Pin pin, PinMode mode) noexcept
{
#ifdef DUET_NG
	if (pin >= DueXnExpansionStart)
	{
		// Note: the SX1509B I/O expander chip doesn't seem to work if you set PWM mode and then set digital output mode.
		DuetExpansion::SetPinMode(pin, mode);
	}
	else
	{
		pinMode(pin, mode);
	}
#else
	pinMode(pin, mode);
#endif
}

/*static*/ bool IoPort::ReadPin(Pin pin) noexcept
{
#ifdef DUET_NG
	if (pin >= DueXnExpansionStart)
	{
		return DuetExpansion::DigitalRead(pin);
	}
	else
	{
		return digitalRead(pin);
	}
#else
	return digitalRead(pin);
#endif
}

/*static*/ void IoPort::WriteDigital(Pin pin, bool high) noexcept
{
#ifdef DUET_NG
	if (pin >= DueXnExpansionStart)
	{
		DuetExpansion::DigitalWrite(pin, high);
	}
	else
	{
		digitalWrite(pin, high);
	}
#else
	digitalWrite(pin, high);
#endif
}

/*static*/ void IoPort::WriteAnalog(Pin pin, float pwm, uint16_t freq) noexcept
{
#if defined(DUET_NG)
	if (pin >= DueXnExpansionStart)
	{
		DuetExpansion::AnalogOut(pin, pwm);
	}
	else
#endif
	{
		AnalogOut::Write(pin, pwm, freq);
	}
}

// Members of class PwmPort
PwmPort::PwmPort() noexcept
{
	frequency = DefaultPinWritePwmFreq;
}

// Append the frequency if the port is valid
void PwmPort::AppendFrequency(const StringRef& str) const noexcept
{
	if (IsValid())
	{
		str.catf(" frequency %uHz", frequency);
	}
}

void PwmPort::AppendFullDetails(const StringRef& str) const noexcept
{
	AppendBasicDetails(str);
	AppendFrequency(str);
}

void PwmPort::WriteAnalog(float pwm) const noexcept
{
	if (IsValid())
	{
		IoPort::WriteAnalog(GetPinNoCheck(), ((totalInvert) ? 1.0 - pwm : pwm), frequency);
	}
}

// End
