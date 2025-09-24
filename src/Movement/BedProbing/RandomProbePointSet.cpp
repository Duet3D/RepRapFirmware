/*
 * RandomProbePointSet.cpp
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#include "RandomProbePointSet.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RandomProbePointSet, __VA_ARGS__)

constexpr ObjectModelTableEntry RandomProbePointSet::objectModelTable[] =
{
	// These entries must be in alphabetical order
	{ "numPointsProbed", OBJECT_MODEL_FUNC((int32_t)self->numBedCompensationPoints), ObjectModelEntryFlags::none }
};

constexpr uint8_t RandomProbePointSet::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE(RandomProbePointSet)

#endif

RandomProbePointSet::RandomProbePointSet() noexcept : numBedCompensationPoints(0)
{
	for (size_t point = 0; point < MaxProbePoints; point++)
	{
		probePointSet[point] = unset;
		zBedProbePoints[point] = 0.0;		// so that the M122 report looks tidy
	}
}

// Record the X and Y coordinates of a probe point
void RandomProbePointSet::SetXYBedProbePoint(size_t index, float x, float y) noexcept
{
	xBedProbePoints[index] = x;
	yBedProbePoints[index] = y;
	probePointSet[index] |= xySet;
}

// Record the Z coordinate of a probe point
void RandomProbePointSet::SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError) noexcept
{
	zBedProbePoints[index] = z;
	probePointSet[index] |= zSet;

	if (wasXyCorrected)
	{
		probePointSet[index] |= xyCorrected;
	}
	else
	{
		probePointSet[index] &= ~xyCorrected;
	}

	if (wasError)
	{
		probePointSet[index] |= probeError;
	}
	else
	{
		probePointSet[index] &= ~probeError;
	}
}

size_t RandomProbePointSet::NumberOfProbePoints() const noexcept
{
	for (size_t i = 0; i < MaxProbePoints; i++)
	{
		if ((probePointSet[i] & (xySet | zSet)) != (xySet | zSet))
		{
			return i;
		}
	}
	return MaxProbePoints;
}

// Clear out the Z heights so that we don't re-use old points
void RandomProbePointSet::ClearProbeHeights() noexcept
{
	for (size_t i = 0; i < MaxProbePoints; ++i)
	{
		probePointSet[i] &= ~zSet;
	}
}

// Check whether the specified set of points has been successfully defined and probed
bool RandomProbePointSet::GoodProbePoints(size_t numPoints) const noexcept
{
	for (size_t i = 0; i < numPoints; ++i)
	{
		if ((probePointSet[i] & (xySet | zSet | probeError)) != (xySet | zSet))
		{
			return false;
		}
	}
	return true;
}

// Print out the probe heights and any errors
void RandomProbePointSet::ReportProbeHeights(size_t numPoints, const StringRef& reply) const noexcept
{
	reply.copy("G32 bed probe heights:");
	float sum = 0.0;
	float sumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		if ((probePointSet[i] & (xySet | zSet)) != (xySet | zSet))
		{
			reply.cat(" not set");
		}
		else if ((probePointSet[i] & probeError) != 0)
		{
			reply.cat(" probing failed");
		}
		else
		{
			reply.catf(" %.3f", (double)zBedProbePoints[i]);
			sum += zBedProbePoints[i];
			sumOfSquares += fsquare(zBedProbePoints[i]);
		}
	}
	const float mean = sum/numPoints;
	// In the following, if there is only 1 point we may try to take the square root of a negative number due to rounding error, hence the 'max' call
	const float stdDev = fastSqrtf(max<float>(sumOfSquares/numPoints - fsquare(mean), 0.0));
	reply.catf(", mean %.3f, deviation from mean %.3f", (double)mean, (double)stdDev);
}

void RandomProbePointSet::DebugPrint(size_t numPoints) const noexcept
{
	debugPrintf("Z probe offsets:");
	float sum = 0.0;
	float sumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		debugPrintf(" %.3f", (double)zBedProbePoints[i]);
		sum += zBedProbePoints[i];
		sumOfSquares += fsquare(zBedProbePoints[i]);
	}
	const float mean = sum/numPoints;
	debugPrintf(", mean %.3f, deviation from mean %.3f\n", (double)mean, (double)fastSqrtf(sumOfSquares/numPoints - fsquare(mean)));
}

// End
