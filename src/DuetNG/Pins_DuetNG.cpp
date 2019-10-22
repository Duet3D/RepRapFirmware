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
		hardwareInverted = false;
		return true;
	}

	for (size_t lp = 0; lp < ARRAY_SIZE(PinTable); ++lp)
	{
		const char *q = PinTable[lp].names;
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

// End
