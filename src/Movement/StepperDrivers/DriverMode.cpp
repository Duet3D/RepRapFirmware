/*
 * DriverMode.cpp
 *
 *  Created on: 27 Apr 2018
 *      Author: David
 */

#include <RepRapFirmware.h>
#include "DriverMode.h"

// This table must be in the same order as enum DriverMode
static const char *_ecv_array const DriverModeStrings[] =
{
	"constant off-time",
	"random off-time",
	"spreadCycle",
	"stealthChop",
#if SUPPORT_TMC51xx && (SUPPORT_CLOSED_LOOP || SUPPORT_PHASE_STEPPING)
	"direct",
#endif
	"unknown"
};

static_assert(ARRAY_SIZE(DriverModeStrings) == (unsigned int)DriverMode::unknown + 1, "bad driver mode string table");

const char *_ecv_array TranslateDriverMode(unsigned int mode) noexcept
{
	const unsigned int imode = min<unsigned int>(mode, (unsigned int)DriverMode::unknown);
	return DriverModeStrings[imode];
}

// End
