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

#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(KeepoutZone, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(KeepoutZone, _condition, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry KeepoutZone::objectModelArrayTable[] =
{
	// 0. Axis coordinates
	{
		nullptr,					// no lock needed
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetTotalAxes(); },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
					{ return (((const KeepoutZone*)self)->axesChecked.IsBitSet(context.GetLastIndex())) ? ExpressionValue(self, 1) : ExpressionValue(nullptr); }
	},
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(KeepoutZone)

constexpr ObjectModelTableEntry KeepoutZone::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. KeepoutZone members
	{ "active",		OBJECT_MODEL_FUNC(self->active),											ObjectModelEntryFlags::none },
	{ "coords",		OBJECT_MODEL_FUNC_ARRAY(0),													ObjectModelEntryFlags::none },
	// 1. KeepoutZone.coords[] members
	{ "max",		OBJECT_MODEL_FUNC(self->coords[context.GetLastIndex()][1]),					ObjectModelEntryFlags::none },
	{ "min",		OBJECT_MODEL_FUNC(self->coords[context.GetLastIndex()][0]),					ObjectModelEntryFlags::none },
};

constexpr uint8_t KeepoutZone::objectModelTableDescriptor[] =
{
	2,				// number of sections
	2,
	2
};

DEFINE_GET_OBJECT_MODEL_TABLE(KeepoutZone)

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
			if (!seenAxis)
			{
				axesChecked.Clear();
				seenAxis = true;
			}
			size_t numCoords = 2;
			const size_t axisNumber = p - axisNames;
			gb.GetFloatArray(coords[axisNumber], numCoords, false);
			axesChecked.SetOrClearBit(axisNumber, coords[axisNumber][1] > coords[axisNumber][0]);
		}
		++p;
	}

	// See if enabled or disabled
	bool seen = false;
	if (gb.Seen('S'))
	{
		seen = true;
		active = (gb.GetUIValue() != 0);
	}
	else if (seenAxis)
	{
		active = true;
	}

	if (seenAxis || seen)
	{
		reprap.MoveUpdated();
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

// Class used to keep track of ranges of arcs that lie inside the keepout zone
class ArcIntervals
{
public:
	// Construct an initial range from startAngle to endAngle
	ArcIntervals(float startAngle, float endAngle) noexcept;

	// Take a copy of the current interval set, limit each interval to a range, and add it to nextSet
	bool AddRangeLimit(float startAngle, float endAngle) noexcept;

	// Make nextSet current and clear nextSet
	void SwitchIntervals() noexcept;

	// Check whether we have any intervals left
	bool HaveInterval() const noexcept { return numCurrentIntervals != 0; }

	// Clear this out
	void Clear() noexcept { numCurrentIntervals = 0; }

private:
	// Print the ranges to debug
	void DebugPrint(const char *str) const noexcept;

	static constexpr size_t MaxIntervals = 4;

	// Struct to represent up to four prohibited angle ranges
	struct IntervalSet
	{
		float lowAngle[MaxIntervals];
		float highAngle[MaxIntervals];
	};

	// Two interval sets so that we can switch between them - allows us to restrict the intervals without needing to copy an interval set
	IntervalSet intervals[2];
	size_t numCurrentIntervals;						// how many intervals are in currentSet
	unsigned int currentSetNumber;					// which set is the the current set, always 0 or 1
};

// Construct an initial range from startAngle to endAngle. Both are in the range -Pi to Pi.
ArcIntervals::ArcIntervals(float startAngle, float endAngle) noexcept
{
	currentSetNumber = 0;

	if (endAngle < startAngle)
	{
		// The range crosses Pi so split it into two
		intervals[0].lowAngle[0] = startAngle;
		intervals[0].highAngle[0] = Pi;
		intervals[0].lowAngle[1] = -Pi;
		intervals[0].highAngle[1] = endAngle;
		numCurrentIntervals = 2;
	}
	else
	{
		intervals[0].lowAngle[0] = startAngle;
		intervals[0].highAngle[0] = endAngle;
		numCurrentIntervals = 1;
	}

#if ARC_DEBUG
	DebugPrint("init");
#endif
}

// Limit the existing range(s) by a new one. startAngle and endAngle are both in the range -Pi to Pi.
// Return true if there are no intervals left.
bool ArcIntervals::AddRangeLimit(float startAngle, float endAngle) noexcept
{
#if ARC_DEBUG
	debugPrintf("Adding range limit %.3f to %.3f\n", (double)startAngle, (double)endAngle);
#endif
	const IntervalSet& currentSet = intervals[currentSetNumber];
	currentSetNumber ^= 1;
	IntervalSet& nextSet = intervals[currentSetNumber];
	size_t numNextIntervals = 0;
	if (endAngle < startAngle)
	{
		// This interval crosses Pi, so split it into two intervals and apply them separately to the existing intervals.
		// This may cause one or more existing intervals to be split into two.
		for (size_t i = 0; i < numCurrentIntervals; ++i)
		{
			{
				const float nextLowAngle = currentSet.lowAngle[i];
				const float nextHighAngle = min<float>(currentSet.highAngle[i], endAngle);
				if (nextLowAngle < nextHighAngle)
				{
					// Copy this interval across with its new bounds
					RRF_ASSERT(numNextIntervals < MaxIntervals);
					nextSet.lowAngle[numNextIntervals] = nextLowAngle;
					nextSet.highAngle[numNextIntervals] = nextHighAngle;
					++numNextIntervals;
#if ARC_DEBUG
					{
						debugPrintf("Next set (1):");
						for (size_t k = 0; k < numNextIntervals; ++k) { debugPrintf(" [%.3f:%.3f]", (double)nextSet.lowAngle[k], (double)nextSet.highAngle[k]); }
						debugPrintf("\n");
						delay(2);
					}
#endif
				}
			}
			{
				const float nextLowAngle = max<float>(currentSet.lowAngle[i], startAngle);
				const float nextHighAngle = currentSet.highAngle[i];
				if (nextLowAngle < nextHighAngle)
				{
					// Copy this interval across with its new bounds
					RRF_ASSERT(numNextIntervals < MaxIntervals);
					nextSet.lowAngle[numNextIntervals] = nextLowAngle;
					nextSet.highAngle[numNextIntervals] = nextHighAngle;
					++numNextIntervals;
#if ARC_DEBUG
					{
						debugPrintf("Next set (2):");
						for (size_t k = 0; k < numNextIntervals; ++k) { debugPrintf(" [%.3f:%.3f]", (double)nextSet.lowAngle[k], (double)nextSet.highAngle[k]); }
						debugPrintf("\n");
						delay(2);
					}
#endif
				}
			}
		}
	}
	else
	{
		// This range limit does not cross Pi, so just apply it to the existing intervals
		for (size_t i = 0; i < numCurrentIntervals; ++i)
		{
			const float nextLowAngle = max<float>(currentSet.lowAngle[i], startAngle);
			const float nextHighAngle = min<float>(currentSet.highAngle[i], endAngle);
			if (nextLowAngle < nextHighAngle)
			{
				// Copy this interval across with its new bounds
				RRF_ASSERT(numNextIntervals < MaxIntervals);
				nextSet.lowAngle[numNextIntervals] = nextLowAngle;
				nextSet.highAngle[numNextIntervals] = nextHighAngle;
				++numNextIntervals;
#if ARC_DEBUG
				{
					debugPrintf("Next set (3):");
					for (size_t k = 0; k < numNextIntervals; ++k) { debugPrintf(" [%.3f:%.3f]", (double)nextSet.lowAngle[k], (double)nextSet.highAngle[k]); }
					debugPrintf("\n");
					delay(2);
				}
#endif
			}
		}
	}
	numCurrentIntervals = numNextIntervals;

#if ARC_DEBUG
	DebugPrint("add");
#endif

	return numNextIntervals == 0;
}

// Print the ranges to debug
void ArcIntervals::DebugPrint(const char *str) const noexcept
{
	const IntervalSet& currentSet = intervals[currentSetNumber];
	debugPrintf("Intervals after %s:", str);
	for (size_t i = 0; i < numCurrentIntervals; ++i)
	{
		debugPrintf(" %.3f:%.3f", (double)currentSet.lowAngle[i], (double)currentSet.highAngle[i]);
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

#if ARC_DEBUG
	debugPrintf("Checking arc: centre X%.3f Y%.3f R%.3f start %.3f end %.3f clockwise %u\n",
					(double)arcCentres[0], (double)arcCentres[1], (double)arcRadius, (double)startAngle, (double)endAngle, clockwise);
#endif

	// From now on, work on angles not on move proportion. There may be up to 4 angle intervals in which the circle falls inside the rectangle.
	// Convert the 'p' interval to one or two angle intervals
	float initLowAngle, initHighAngle;
	if (wholeCircle && pLow == 0.0 && pHigh == 1.0)
	{
		// We can special case this
		initLowAngle = -Pi;
		initHighAngle = Pi;
	}
	else
	{
		// If it's a clockwise movement (angle decreasing) then replace it by an anticlockwise movement (angle increasing)
		if (clockwise)
		{
			std::swap(startAngle, endAngle);
		}

		if (endAngle <= startAngle)
		{
			// The arc crosses the boundary between negative and positive angles. Add 2*Pi to the end angle to avoid a discontinuity.
			endAngle += TwoPi;					// make endAngle > startAngle (this could make endAngle up to 3*pi)
		}
		initLowAngle = pLow * endAngle + (1.0 - pLow) * startAngle;
		initHighAngle = pHigh * endAngle + (1.0 - pHigh) * startAngle;

		// Re-normalise the angles to be in the range -pi to pi
		if (initLowAngle > Pi) { initLowAngle -= TwoPi; }
		if (initHighAngle > Pi) { initHighAngle -= TwoPi; }
	}

#if ARC_DEBUG
	debugPrintf("pLow=%.3f pHigh=%.3f whole=%u initLow=%.3f initHigh=%.3f\n", (double)pLow, (double)pHigh, wholeCircle, (double)initLowAngle, (double)initHighAngle);
#endif

	ArcIntervals angleIntervals(initLowAngle, initHighAngle);

#if ARC_DEBUG
	debugPrintf("Iterating cosine axes\n");
#endif

	// Handle the axis whose coordinates vary with the cosine of the angle
	cosineAxes.IterateWhile([this, arcCentres, arcRadius, &angleIntervals](unsigned int axis, unsigned int)->bool
								{
									// Check for the circle being wholly outside the keepout zone
									if (arcCentres[axis] + arcRadius <= coords[axis][0] || arcCentres[axis] - arcRadius >= coords[axis][1])
									{
										angleIntervals.Clear();			// fully outside, so there is no angle interval that violates the keepout zone
										return false;					// stop iterating
									}

									// Check for the circle being wholly inside the keepout zone
									if (arcCentres[axis] + arcRadius <= coords[axis][1] && arcCentres[axis] - arcRadius >= coords[axis][0])
									{
										return true;					// fully inside, so no need to check this axis further, but check other axes
									}

									const float cos1 = (coords[axis][0] - arcCentres[axis])/arcRadius;
									if (cos1 < 1.0 && cos1 > -1.0)
									{
										const float aLow = acosf(cos1);
										// The circle is to the right of the left hand limit at angles between -aLow up to +aLow
										if (angleIntervals.AddRangeLimit(-aLow, aLow))
										{
											return false;				// if no angle intervals left then we can stop iterating
										}
									}

									const float cos2 = (coords[axis][1] - arcCentres[axis])/arcRadius;
									if (cos2 < 1.0 && cos2 > -1.0)
									{
										const float aHigh = acosf(cos2);
										// The circle is to the left of the right hand limit at angles between aHigh and pi and between -pi and -aHigh
										if (angleIntervals.AddRangeLimit(aHigh, -aHigh))
										{
											return false;				// if no angle intervals left then we can stop iterating
										}
									}
									return true;						// carry on iterating axes
								});

	if (angleIntervals.HaveInterval())
	{
#if ARC_DEBUG
		debugPrintf("Iterating sine axes\n");
#endif
		sineAxes.IterateWhile([this, arcCentres, arcRadius, &angleIntervals](unsigned int axis, unsigned int)->bool
								{
									// Check for the circle being wholly outside the keepout zone
									if (arcCentres[axis] + arcRadius <= coords[axis][0] || arcCentres[axis] - arcRadius >= coords[axis][1])
									{
										angleIntervals.Clear();			// fully outside, so there is no angle interval that violates the keepout zone
										return false;					// stop iterating
									}

									// Check for the circle being wholly inside the keepout zone
									if (arcCentres[axis] + arcRadius <= coords[axis][1] && arcCentres[axis] - arcRadius >= coords[axis][0])
									{
										return true;					// fully inside, so no need to check this axis further but check other axes
									}

									const float sin1 = (coords[axis][0] - arcCentres[axis])/arcRadius;
									if (sin1 < 1.0 && sin1 > -1.0)
									{
										const float aLow = asinf(sin1);
										// The circle is above the lower limit angle between aLow and (pi - aLow)
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
										// The circle is below the upper limit at angles between (pi - aHigh) and aHigh
										float aHigh2 = ((aHigh < 0.0) ? -Pi : Pi) - aHigh;
										if (angleIntervals.AddRangeLimit(aHigh2, aHigh))
										{
											return false;
										}
									}
									return true;
								});
	}

	return angleIntervals.HaveInterval();
}

#endif

// End
