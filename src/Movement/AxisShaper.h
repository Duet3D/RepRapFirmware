/*
 * InputShaper.h
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_AXISSHAPER_H_
#define SRC_MOVEMENT_AXISSHAPER_H_

#define SUPPORT_DAA		(1)

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>
#include <ObjectModel/ObjectModel.h>
#include "InputShaperPlan.h"

// These names must be in alphabetical order and lowercase
NamedEnum(InputShaperType, uint8_t,
	custom,
#if SUPPORT_DAA
	daa,
#endif
	ei2,
	ei3,
	none,
	zvd,
	zvdd,
);

class DDA;
class BasicPrepParams;
class MoveSegment;

class AxisShaper INHERIT_OBJECT_MODEL
{
public:
	AxisShaper() noexcept;

	float GetFrequency() const noexcept { return frequency; }
	float GetDamping() const noexcept { return zeta; }
	InputShaperType GetType() const noexcept { return type; }
	InputShaperPlan PlanShaping(DDA& dda, BasicPrepParams& params, bool shapingEnabled) const noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M593

	static MoveSegment *GetUnshapedSegments(DDA& dda, const BasicPrepParams& params) noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	MoveSegment *GetAccelerationSegments(const DDA& dda, const BasicPrepParams& params, InputShaperPlan& plan) const noexcept;
	MoveSegment *GetDecelerationSegments(const DDA& dda, const BasicPrepParams& params, InputShaperPlan& plan) const noexcept;
	MoveSegment *FinishSegments(const DDA& dda, const BasicPrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept;
	float GetExtraAccelStartDistance(const DDA& dda) const noexcept;
	float GetExtraAccelEndDistance(const DDA& dda) const noexcept;
	float GetExtraDecelStartDistance(const DDA& dda) const noexcept;
	float GetExtraDecelEndDistance(const DDA& dda) const noexcept;

	static constexpr unsigned int MaxExtraImpulses = 4;
	static constexpr float DefaultFrequency = 40.0;
	static constexpr float DefaultDamping = 0.1;
	static constexpr float DefaultMinimumAcceleration = 10.0;

	unsigned int numExtraImpulses;						// the number of extra impulses
	float frequency;									// the undamped frequency in Hz
	float zeta;											// the damping ratio, see https://en.wikipedia.org/wiki/Damping. 0 = undamped, 1 = critically damped.
	float minimumAcceleration;							// the minimum value that we reduce average acceleration to in mm/sec^2
	float coefficients[MaxExtraImpulses];				// the coefficients of all the impulses
	float durations[MaxExtraImpulses];					// the duration in step clocks of each impulse
	float totalShapingClocks;							// the total input shaping time in step clocks
	float extraClocksAtStart;							// the extra time needed to shape the start of acceleration or deceleration
	float extraClocksAtEnd;								// the extra time needed to shape the end of acceleration or deceleration
	float extraDistanceAtStart;							// the extra distance per unit acceleration to shape the start of acceleration or deceleration, less the initial velocity contribution
	float extraDistanceAtEnd;							// the extra distance per unit acceleration to shape the end of acceleration or deceleration, less the final velocity contribution
	float overlappedDurations[2 * MaxExtraImpulses];	// the duration in step clocks of each impulse of an overlapped acceleration or deceleration
	float overlappedCoefficients[2 * MaxExtraImpulses];	// the coefficients if we use a shaped start immediately followed by a shaped end
	float overlappedShapingClocks;						// the acceleration or deceleration duration when we use overlapping, in step clocks
	float overlappedDeltaVPerA;							// the effective acceleration time (velocity change per unit acceleration) when we use overlapping, in step clocks
	float overlappedDistancePerA;						// the distance needed by an overlapped acceleration or deceleration, less the initial velocity contribution
	InputShaperType type;
};

#endif /* SRC_MOVEMENT_AXISSHAPER_H_ */
