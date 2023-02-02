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

#define ARC_DEBUG	0

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

#if 0	// not used

// Check whether a point is inside the keepout zone, returning true if it is
bool KeepoutZone::IsPointInside(const GCodeBuffer &gb, const float *pointCoords) const noexcept
{
	if (!active || axesChecked.IsEmpty())
	{
		return false;
	}

	bool inside = true;
	axesChecked.IterateWhile([this, pointCoords, &outside](unsigned int axis, unsigned int)->bool
							{
								if (pointCoords[axis] <= coords[axis][0] || pointCoords[axis] >= coords[axis][1])
								{
									inside = false;
								};
								return inside;
							});
	return inside;
}

#endif

// Check whether a straight line move lies fully outside the keepout zone, returning true if it is
bool KeepoutZone::DoesLineIntrude(const float startCoords[MaxAxes], const float endCoords[MaxAxes]) const noexcept
{
	if (!active || axesChecked.IsEmpty())
	{
		return false;
	}

	// Algorithm:
	// For any axis A, the coordinate is given by a = p * a_end + (1-p) * a_start
	// For each axis specified in the keepout zone, calculate the interval of P for which the coordinate is within the coordinates of the zone.
	// To do this we compute the intervals of P at which the line crosses the start and end of the zone.
	// If there is a value of P that is in all these intervals and also in the interval 0..1 then the line intrudes into the zone.
	float pLow = 0.0, pHigh = 1.0;
	return axesChecked.IterateWhile([this, startCoords, endCoords, &pLow, &pHigh](unsigned int axis, unsigned int)->bool
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

// Class used to keep track of ranges of arcs that line inside the keepout zone
class ArcIntervals
{
public:
	// Construct an initial range from startAnle to endAngle
	ArcIntervals(float startAngle, float endAngle) noexcept;

	// Limit the existing range(s) by a new one
	bool AddRangeLimit(float startAngle, float endAngle) noexcept;

private:
	// Print the ranges to debug
	void DebugPrint(const char *str) noexcept;

	struct IntervalSet
	{
		float lowAngle[4];
		float highAngle[4];
	};
	IntervalSet intervals[2];
	size_t numIntervals;
	unsigned int currentSetNumber;
};

// Construct an initial range from startAngle to endAngle. Both are in the range -Pi to 3*Pi and the difference is no more than 2 * Pi.
ArcIntervals::ArcIntervals(float startAngle, float endAngle) noexcept
{
	currentSetNumber = 0;

	if (endAngle < startAngle)
	{
		// The range crosses Pi so split it into two
		intervals[0].lowAngle[0] = startAngle;
		intervals[0].highAngle[0] = Pi;
		intervals[0].lowAngle[1] = -Pi;
		intervals[0].highAngle[1] = endAngle - Pi;
		numIntervals = 2;
	}
	else
	{
		if (startAngle >= Pi)
		{
			intervals[0].lowAngle[0] = startAngle - TwoPi;
			intervals[0].highAngle[0] = endAngle - TwoPi;
		}
		else
		{
			intervals[0].lowAngle[0] = startAngle;
			intervals[0].highAngle[0] = endAngle;
		}
		numIntervals = 1;
	}

#if ARC_DEBUG
	DebugPrint("init");
#endif
}

// Limit the existing range(s) by a new one. startAngle and endAngle are both in the range -Pi to Pi.
// Return true if there are no intervals left.
bool ArcIntervals::AddRangeLimit(float startAngle, float endAngle) noexcept
{
	const IntervalSet& currentSet = intervals[currentSetNumber];
	currentSetNumber ^= 1;
	IntervalSet& nextSet = intervals[currentSetNumber];
	size_t j = 0;
	if (endAngle < startAngle)
	{
		// This interval crosses Pi, so split it into two intervals and apply them separately to the existing intervals
		for (size_t i = 0; i < numIntervals; ++i)
		{
			{
				const float nextLowAngle = currentSet.lowAngle[i];
				const float nextHighAngle = min<float>(currentSet.highAngle[i], startAngle);
				if (nextLowAngle < nextHighAngle)
				{
					// Copy this interval across with its new bounds
					RRF_ASSERT(j < 4);
					nextSet.lowAngle[j] = nextLowAngle;
					nextSet.highAngle[j] = nextHighAngle;
					++j;
				}
			}
			{
				const float nextLowAngle = max<float>(currentSet.lowAngle[i], endAngle);
				const float nextHighAngle = currentSet.highAngle[i];
				if (nextLowAngle < nextHighAngle)
				{
					// Copy this interval across with its new bounds
					RRF_ASSERT(j < 4);
					nextSet.lowAngle[j] = nextLowAngle;
					nextSet.highAngle[j] = nextHighAngle;
					++j;
				}
			}
		}
	}
	else
	{
		// This range limit does not cross Pi, so just apply it to the existing intervals
		for (size_t i = 0; i < numIntervals; ++i)
		{
			const float nextLowAngle = max<float>(currentSet.lowAngle[i], startAngle);
			const float nextHighAngle = min<float>(currentSet.highAngle[i], endAngle);
			if (nextLowAngle < nextHighAngle)
			{
				// Copy this interval across with its new bounds
				nextSet.lowAngle[j] = nextLowAngle;
				nextSet.highAngle[j] = nextHighAngle;
				++j;
			}
		}
	}
	numIntervals = j;

#if ARC_DEBUG
	DebugPrint("add");
#endif

	return j == 0;
}

// Print the ranges to debug
void ArcIntervals::DebugPrint(const char *str) noexcept
{
	const IntervalSet& currentSet = intervals[currentSetNumber];
	debugPrintf("Intervals %s:", str);
	for (size_t i = 0; i < numIntervals; ++i)
	{
		debugPrintf(" %.2f:%.2f", (double)currentSet.lowAngle[i], (double)currentSet.highAngle[i]);
	}
	debugPrintf("\n");
}

// Check whether an arc move intrudes into the keepout zone, returning true if it does. See CheckLineIsOutside for the algorithm.
bool KeepoutZone::DoesArcIntrude(const float startCoords[MaxAxes], const float endCoords[MaxAxes],
									float startAngle, float endAngle,
									const float arcCentres[MaxAxes], float arcRadius,
									AxesBitmap cosineAxes, AxesBitmap sineAxes,
									bool clockwise, bool wholeCircle) const noexcept
{
	if (!active || axesChecked.IsEmpty())
	{
		return false;
	}

	// First check the linear axes because that needs less computation
	float pLow = 0.0, pHigh = 1.0;
	const AxesBitmap linearAxes = axesChecked - (cosineAxes | sineAxes);
	if (   !linearAxes.IsEmpty()
		&& !linearAxes.IterateWhile([this, startCoords, endCoords, &pLow, &pHigh](unsigned int axis, unsigned int)->bool
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
										return pHigh > pLow;
									}))
	{
		return false;
	}

	// From now on, work on angles not on move proportion. There may be up to 4 angle intervals in which the circle falls inside the rectangle.
	// Convert the 'p' interval to one or two angle intervals. We handle the clockwise/anticlockwise distinction here.
	float tempLowAngle, tempHighAngle;
	if (clockwise)
	{
		// Angle is decreasing from startAngle to endAngle
		if (wholeCircle)
		{
			// If we are describing a whole circle then we might be passed the same angle twice or they might already differ by 2*pi
			startAngle = endAngle + TwoPi;
		}
		else if (endAngle > startAngle)
		{
			// The arc does cross the boundary between negative and positive angles
			startAngle += TwoPi;				// make startAngle > endAngle
		}
		tempHighAngle = pLow * endAngle + (1.0 - pLow) * startAngle;
		tempLowAngle = pHigh * endAngle + (1.0 - pHigh) * startAngle;
	}
	else
	{
		// Angle is increasing from startAngle to endAngle
		if (wholeCircle)
		{
			// If we are describing a whole circle then we might be passed the same angle twice or they might already differ by 2*pi
			endAngle = startAngle + TwoPi;
		}
		else if (endAngle < startAngle)
		{
			// The arc does cross the boundary between negative and positive angles
			endAngle += TwoPi;				// make endAngle > startAngle
		}
		tempLowAngle = pLow * endAngle + (1.0 - pLow) * startAngle;
		tempHighAngle = pHigh * endAngle + (1.0 - pHigh) * startAngle;
	}


#if ARC_DEBUG
	debugPrintf("pLow=%.3f pHigh=%.3f whole=%u tempLow=%.2f tempHigh=%.2F\n", (double)pLow, (double)pHigh, wholeCircle, (double)tempLowAngle, (double)tempHighAngle);
#endif

	ArcIntervals angleIntervals(tempLowAngle, tempHighAngle);

	// Handle the axis whose coordinates vary with the cosine of the angle
	return cosineAxes.IterateWhile([this, arcCentres, arcRadius, &angleIntervals](unsigned int axis, unsigned int)->bool
									{
										// Check for the circle being wholly outside the keepout zone
										if (arcCentres[axis] + arcRadius <= coords[axis][0] || arcCentres[axis] - arcRadius >= coords[axis][1])
										{
											return false;		// fully outside, so stop iterating
										}

										// Check for the circle being wholly inside the keepout zone
										if (arcCentres[axis] + arcRadius <= coords[axis][1] && arcCentres[axis] - arcRadius >= coords[axis][0])
										{
											return true;		// fully inside, so no need to check this axis further
										}

										const float cos1 = (coords[axis][0] - arcCentres[axis])/arcRadius;
										if (cos1 < 1.0 && cos1 > -1.0)
										{
											const float aLow = acosf(cos1);
											// The circle crosses the lower limit at angles between -aLow up to +aLow
											if (angleIntervals.AddRangeLimit(-aLow, aLow))
											{
												return false;
											}
										}

										const float cos2 = (coords[axis][1] - arcCentres[axis])/arcRadius;
										if (cos2 < 1.0 && cos2 > -1.0)
										{
											const float aHigh = acosf(cos2);
											// The circle crosses the upper limit at angles between -pi and -aHigh and between aHigh and pi
											if (angleIntervals.AddRangeLimit(aHigh, -aHigh))
											{
												return false;
											}
										}
										return true;
									})
		&& sineAxes.IterateWhile([this, arcCentres, arcRadius, &angleIntervals](unsigned int axis, unsigned int)->bool
									{
										// Check for the circle being wholly outside the keepout zone
										if (arcCentres[axis] + arcRadius <= coords[axis][0] || arcCentres[axis] - arcRadius >= coords[axis][1])
										{
											return false;		// fully outside, so stop iterating
										}

										// Check for the circle being wholly inside the keepout zone
										if (arcCentres[axis] + arcRadius <= coords[axis][1] && arcCentres[axis] - arcRadius >= coords[axis][0])
										{
											return true;		// fully inside, so no need to check this axis further
										}

										const float sin1 = (coords[axis][0] - arcCentres[axis])/arcRadius;
										if (sin1 < 1.0 && sin1 > -1.0)
										{
											const float aLow = asinf(sin1);
											// The circle crosses the lower limit angle between aLow and (pi - aLow)
											const float aLow2 = ((aLow < 0.0) ? -Pi : Pi) - aLow;
											if (angleIntervals.AddRangeLimit(aLow, aLow2))
											{
												return false;
											}
										}

										const float sin2 = (coords[axis][1] - arcCentres[axis])/arcRadius;
										if (sin2 < 1.0 && sin2 > -1.0)
										{
											const float aHigh = asinf(sin2);
											// The circle crosses the upper limit at angles between -pi and -aHigh and between aHigh and pi
											float aHigh2 = ((aHigh < 0.0) ? -Pi : Pi) - aHigh;
											if (angleIntervals.AddRangeLimit(aHigh2, aHigh))
											{
												return false;
											}
										}
										return true;
									});
}

#endif

// End
