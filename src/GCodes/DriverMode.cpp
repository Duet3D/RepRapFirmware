/*
 * DriverMode.cpp
 *
 *  Created on: 27 Apr 2018
 *      Author: David
 */

#include "DriverMode.h"
#include "RepRapFirmware.h"

// This table must be in the same order as enum DriverMode
static const char * const DriverModeStrings[] =
{
	"constant off-time",
	"random off-time",
	"spreadCycle",
	"stealthChop",
	"unknown"
};

static_assert(ARRAY_SIZE(DriverModeStrings) == (unsigned int)DriverMode::unknown + 1, "bad driver mode string table");

const char* TranslateDriverMode(unsigned int mode)
{
	const unsigned int imode = min<unsigned int>(mode, (unsigned int)DriverMode::unknown);
	return DriverModeStrings[imode];
}

// End
