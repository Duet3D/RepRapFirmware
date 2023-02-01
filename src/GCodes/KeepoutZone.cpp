/*
 * KeepoutZone.cpp
 *
 *  Created on: 1 Feb 2023
 *      Author: David
 */

#include "KeepoutZone.h"

#if SUPPORT_KEEPOUT_ZONES

GCodeResult KeepoutZone::Configure(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	return GCodeResult::errorNotSupported;
}

void KeepoutZone::CheckPointIsOutside(const GCodeBuffer &gb, const float *coords) THROWS(GCodeException)
{
}

void KeepoutZone::CheckLineIsOutside(const GCodeBuffer &gb, const float *startCoords, const float *endCoords) THROWS(GCodeException)
{
}

#endif

// End
