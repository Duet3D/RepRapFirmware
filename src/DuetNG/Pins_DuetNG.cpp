/*
 * Pins_DuetNG.cpp
 *
 *  Created on: 31 Mar 2019
 *      Author: David
 */

#include "RepRapFirmware.h"

// Hardware-dependent pins functions

// Function to look up a pin name pass back the corresponding index into the pin table
// On this platform, the mapping from pin names to pins is fixed, so this is a simple lookup
bool LookupPinName(const char*pn, LogicalPin& lpin, bool& hardwareInverted)
{
	if (StringEqualsIgnoreCase(pn, NoPinName))
	{
		lpin = NoLogicalPin;
		return true;
	}

	for (size_t lp = 0; lp < ARRAY_SIZE(LogicalPinTable); ++lp)
	{
		const char *q = LogicalPinTable[lp].names;
		while (*q != 0)
		{
			// Try the next alias in the list of names for this pin
			const char *p = pn;
			bool hwInverted = (*q == '!');
			if (hwInverted)
			{
				++q;
			}
			while (*q != ',' && *q != 0 && *p == *q)
			{
				++p;
				++q;
			}
			if (*p == 0 && (*q == 0 || *q == ','))
			{
				// Found a match
				lpin = (LogicalPin)lp;
				hardwareInverted = hwInverted;
				return true;
			}

			// Skip to the start of the next alias
			while (*q != 0 && *q != ',')
			{
				++q;
			}
			if (*q == ',')
			{
				++q;
			}
		}
	}
	return false;
}

// Return true if the pin can be used for the specified function
bool PinEntry::CanDo(PinAccess access) const
{
	switch (access)
	{
	case PinAccess::read:
	case PinAccess::readWithPullup:
		return ((uint8_t)cap & (uint8_t)PinCapability::read) != 0;

	case PinAccess::readAnalog:
		return ((uint8_t)cap & (uint8_t)PinCapability::ain) != 0;

	case PinAccess::write0:
	case PinAccess::write1:
		return ((uint8_t)cap & (uint8_t)PinCapability::write) != 0;

	case PinAccess::pwm:
		return ((uint8_t)cap & (uint8_t)PinCapability::pwm) != 0;

	case PinAccess::servo:
		return pin < 200 && ((uint8_t)cap & (uint8_t)PinCapability::pwm) != 0;

	default:
		return false;
	}
}

// End
