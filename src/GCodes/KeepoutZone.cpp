/*
 * KeepoutZone.cpp
 *
 *  Created on: 1 Feb 2023
 *      Author: David
 */

#include "KeepoutZone.h"

#if SUPPORT_KEEPOUT_ZONES

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

GCodeResult KeepoutZone::Configure(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	// See if any axes have been given
	bool seenAxis = false;
	const char *const axisNames = reprap.GetGCodes().GetAxisLetters();
	const char *p = axisNames;
	while (*p != 0)
	{
		if (gb.Seen(*p))
		{
			seenAxis = true;
			size_t numCoords = 2;
			const size_t axisNumber = p - axisNames;
			axesChecked.ClearBit(axisNumber);								// in case the following call throws
			gb.GetFloatArray(coords[axisNumber], numCoords, false);
			axesChecked.SetOrClearBit(axisNumber, coords[axisNumber][1] > coords[axisNumber][0]);
		}
		++p;
	}

	// See if enabled or disabled
	if (gb.Seen('S'))
	{
		active = (gb.GetUIValue() != 0);
	}
	else if (seenAxis)
	{
		active = true;
	}
	else if (axesChecked.IsEmpty())
	{
		reply.copy("No keepout zone");
	}
	else
	{
		// No axes or S parameter, so report current configuration
		reply.printf("Keepout zone %s, limits:", (active) ? "enabled" : "disabled");
		axesChecked.Iterate([this, axisNames, reply](unsigned int axis, unsigned int)
							{
								reply.catf(" %c (%.1f to %.1f)", axisNames[axis], (double)coords[axis][0], (double)coords[axis][1]);
							});
	}
	return GCodeResult::ok;
}

bool KeepoutZone::CheckPointIsOutside(const GCodeBuffer &gb, const float *pointCoords) noexcept
{
	if (!active || axesChecked.IsEmpty())
	{
		return true;
	}

	bool outside = false;
	axesChecked.IterateWhile([this, pointCoords, &outside](unsigned int axis, unsigned int)->bool
							{
								if (pointCoords[axis] <= coords[axis][0] || pointCoords[axis] >= coords[axis][1])
								{
									outside = true;
								};
								return !outside;
							});
	return outside;
}

bool KeepoutZone::CheckLineIsOutside(const GCodeBuffer &gb, const float *startCoords, const float *endCoords) noexcept
{
	if (!active || axesChecked.IsEmpty())
	{
		return true;
	}

	// Algorithm:
	// For any axis A, the coordinate is given by a = p * a_end + (1-p) * a_start
	// For each axis specified in the keepout zone, calculate the interval of P for which the coordinate is within the coordinates of the zone.
	// To do this we compute the values of P at which the line crosses the start and end of the zone.
	// If there is a value of P that is in all these ranges and also in 0..1 then the line intrudes into the zone.
	float pLow = 0.0, pHigh = 1.0;
	return !axesChecked.IterateWhile([this, startCoords, endCoords, &pLow, &pHigh](unsigned int axis, unsigned int)->bool
									{
										const float diff = endCoords[axis] - startCoords[axis];
										if (fabsf(diff) < 0.01)
										{
											// This axis moves very little if at all, so check whether its coordinate is inside the zone
											return endCoords[axis] >= coords[axis][0] && endCoords[axis] <= coords[axis][1];
										}
										float thisPLow = (coords[axis][0] - startCoords[axis])/diff;
										float thisPHigh = (coords[axis][1] - startCoords[axis])/diff;
										if (thisPLow > thisPHigh)
										{
											std::swap(thisPLow, thisPHigh);
										}
										pLow = max<float>(pLow, thisPLow);
										pHigh = min<float>(pHigh, thisPHigh);
										return pHigh >= pLow;
									});
}

#endif

// End
