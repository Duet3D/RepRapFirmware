/*
 * IoPort.cpp
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#include "IoPorts.h"
#include "RepRap.h"
#include "Platform.h"
#include "Configuration.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#ifdef DUET_NG
# include "DuetNG/DueXn.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include <CanId.h>
#endif

// Read a port name parameter and assign some ports. Caller must call gb.Seen() with the appropriate letter and get 'true' returned before calling this.
// Return the number of ports allocated, or 0 if there was an error with the error message in 'reply'.
size_t IoPort::AssignPorts(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort* const ports[], const PinAccess access[])
{
	// Get the full port names string
	String<StringLength20> portNames;
	if (!gb.GetReducedString(portNames.GetRef()))
	{
		reply.copy("Missing port name string");
		return 0;
	}

	return AssignPorts(portNames.c_str(), reply, neededFor, numPorts, ports, access);
}

// Read a port name parameter and assign one port. Caller must call gb.Seen() with the appropriate letter and get 'true' returned before calling this.
// If successful, return true; else return false with the error message in 'reply'.
bool IoPort::AssignPort(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, PinAccess access)
{
	IoPort* const p = this;
	return AssignPorts(gb, reply, neededFor, 1, &p, &access) == 1;
}

// Try to assign ports, returning the number of ports successfully assigned
size_t IoPort::AssignPorts(const char* pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort* const ports[], const PinAccess access[])
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
		String<StringLength20> pn;
		char c;
		while ((c = pinNames[index]) != 0 && c != '+')
		{
			pn.cat(c);
			++index;
		}

#if SUPPORT_CAN_EXPANSION
		const CanAddress boardAddress = RemoveBoardAddress(pn.GetRef());
		if (boardAddress != 0)
		{
			reply.lcat("Remote ports not yet supported by this command");
			for (size_t j = 0; j < i; ++j)
			{
				ports[j]->Release();
			}
			return 0;
		}
#endif

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

bool IoPort::AssignPort(const char* pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access)
{
	IoPort* const p = this;
	return AssignPorts(pinName, reply, neededFor, 1, &p, &access) == 1;
}

/*static*/ const char* IoPort::TranslatePinAccess(PinAccess access)
{
	switch (access)
	{
	case PinAccess::read:			return "digital read";
	case PinAccess::readWithPullup:	return "digital read (pullup resistor enabled)";
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

/*static*/ void IoPort::Init()
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

IoPort::IoPort() : logicalPin(NoLogicalPin), analogChannel(NO_ADC), hardwareInvert(false), totalInvert(false), isSharedInput(false)
{
}

void IoPort::Release()
{
	if (IsValid() && !isSharedInput)
	{
		detachInterrupt(PinTable[logicalPin].pin);
		portUsedBy[logicalPin] = PinUsedBy::unused;
		logicalPinModes[logicalPin] = PIN_MODE_NOT_CONFIGURED;
	}
	logicalPin = NoLogicalPin;
	analogChannel = NO_ADC;
	hardwareInvert = totalInvert = false;
}

// Attach an interrupt to the pin. Nor permitted if we allocated the pin in shared input mode.
bool IoPort::AttachInterrupt(StandardCallbackFunction callback, enum InterruptMode mode, CallbackParameter param) const
{
	return IsValid() && !isSharedInput && attachInterrupt(PinTable[logicalPin].pin, callback, mode, param);
}

void IoPort::DetachInterrupt() const
{
	if (IsValid() && !isSharedInput)
	{
		detachInterrupt(PinTable[logicalPin].pin);
	}
}

// Allocate the specified logical pin, returning true if successful
bool IoPort::Allocate(const char *pn, const StringRef& reply, PinUsedBy neededFor, PinAccess access)
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
				access = PinAccess::readWithPullup;
			}
		}
		else
		{
			break;
		}
		++pn;
	}

	const char *const fullPinName = pn;			// the full pin name less the inversion and pullup flags

#if defined(DUET3)
	uint32_t expansionNumber;
	if (isdigit(*pn))
	{
		expansionNumber = SafeStrtoul(pn, &pn);
		if (*pn != '.')
		{
			reply.printf("Bad pin name '%s'", fullPinName);
			return false;
		}
	}
	else
	{
		expansionNumber = 0;
	}

	//TODO use expansionNumber as part of the logical pin number
	(void)expansionNumber;
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
bool IoPort::SetMode(PinAccess access)
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
	case PinAccess::pwm:
	case PinAccess::servo:
		desiredMode = (totalInvert) ? OUTPUT_PWM_HIGH : OUTPUT_PWM_LOW;
		break;
	case PinAccess::readAnalog:
		desiredMode = AIN;
		break;
	case PinAccess::readWithPullup:
		desiredMode = INPUT_PULLUP;
		break;
	case PinAccess::read:
	default:
		desiredMode = INPUT;
		break;
	}

	if (logicalPinModes[logicalPin] != (int8_t)desiredMode)
	{
		const AnalogChannelNumber chan = PinToAdcChannel(PinTable[logicalPin].pin);
		if (chan != NO_ADC)
		{
			if (access == PinAccess::readAnalog)
			{
				IoPort::SetPinMode(PinTable[logicalPin].pin, AIN);		// SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
				AnalogInEnableChannel(chan, true);
				analogChannel = chan;
				logicalPinModes[logicalPin] = (int8_t)desiredMode;
				return true;
			}
			else
			{
				AnalogInEnableChannel(chan, false);
				analogChannel = NO_ADC;
			}
		}
		else if (access == PinAccess::readAnalog)
		{
			return false;
		}
		IoPort::SetPinMode(PinTable[logicalPin].pin, desiredMode);
		logicalPinModes[logicalPin] = (int8_t)desiredMode;
	}
	return true;
}

bool IoPort::GetInvert() const
{
	return (hardwareInvert) ? !totalInvert : totalInvert;
}

void IoPort::SetInvert(bool pInvert)
{
	totalInvert = (hardwareInvert) ? !pInvert : pInvert;
}

void IoPort::ToggleInvert(bool pInvert)
{
	if (pInvert)
	{
		totalInvert = !totalInvert;
	}
}

void IoPort::AppendDetails(const StringRef& str) const
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
void IoPort::AppendPinName(const StringRef& str) const
{
	if (IsValid())
	{
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

/*static*/ void IoPort::AppendPinNames(const StringRef& str, size_t numPorts, IoPort * const ports[])
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
				str.cat("nil");
			}
			break;
		}
	}
}

void IoPort::WriteDigital(bool high) const
{
	if (IsValid())
	{
		WriteDigital(PinTable[logicalPin].pin, (totalInvert) ? !high : high);
	}
}

Pin IoPort::GetPin() const
{
	return (IsValid()) ? PinTable[logicalPin].pin : NoPin;
}

bool IoPort::Read() const
{
	if (IsValid())
	{
		const bool b = ReadPin(PinTable[logicalPin].pin);
		return (totalInvert) ? !b : b;
	}
	return false;
}

// Note, for speed when this is called from the ISR we do not apply 'invert' to the analog reading
uint16_t IoPort::ReadAnalog() const
{
	return (analogChannel != NO_ADC) ? AnalogInReadChannel((AnalogChannelNumber)analogChannel) : 0;
}

#if SUPPORT_CAN_EXPANSION

// Remove the board address form a port name string and return it
/*static*/ CanAddress IoPort::RemoveBoardAddress(const StringRef& portName)
{
	size_t prefix = 0;
	while (portName[prefix] == '!' || portName[prefix] == '^')
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
	if (numToSkip != prefix && portName[numToSkip] == '.' && boardAddress <= CanId::MaxNormalAddress)
	{
		portName.Erase(prefix, numToSkip - prefix + 1);			// remove the board address prefix
		return (CanAddress)boardAddress;
	}
	return CanId::MasterAddress;
}

#endif

	// Low level pin access methods

/*static*/ void IoPort::SetPinMode(Pin pin, PinMode mode)
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

/*static*/ bool IoPort::ReadPin(Pin pin)
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

/*static*/ void IoPort::WriteDigital(Pin pin, bool high)
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

/*static*/ void IoPort::WriteAnalog(Pin pin, float pwm, uint16_t freq)
{
#ifdef DUET_NG
	if (pin >= DueXnExpansionStart)
	{
		DuetExpansion::AnalogOut(pin, pwm);
	}
	else
	{
		AnalogOut(pin, pwm, freq);
	}
#else
	AnalogOut(pin, pwm, freq);
#endif
}

// Members of class PwmPort
PwmPort::PwmPort()
{
	frequency = DefaultPinWritePwmFreq;
}

void PwmPort::AppendDetails(const StringRef& str) const
{
	IoPort::AppendDetails(str);
	if (IsValid())
	{
		str.catf(" frequency %uHz", frequency);
	}
}

void PwmPort::WriteAnalog(float pwm) const
{
	if (IsValid())
	{
		IoPort::WriteAnalog(PinTable[logicalPin].pin, ((totalInvert) ? 1.0 - pwm : pwm), frequency);
	}
}

// End
