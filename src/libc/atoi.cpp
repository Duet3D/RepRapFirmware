/*
 * atoi.cpp
 *
 *  Created on: 8 Jan 2023
 *      Author: David
 *
 *  This replaces the atoi function newlib. We redefine it to avoid calling strtol, which needs a reent_struct.
 *  atoi is called by lwip.
 */

#include <cstdlib>
#include <General/SafeStrtod.h>

#undef atoi				// SafeStrtod.h defines this as a macro to tell you not to use it
extern "C" int atoi(const char *s) noexcept
{
	return (int)StrToI32(s,nullptr);
}

// End
