/*
 * DynamicBedPlane.h
 *
 * Shared-Y two-nozzle to multi-leadscrew bed-plane transform.
 * This module contains no motion-planner or hardware side effects.
 */

#ifndef SRC_MOVEMENT_BEDPLANE_DYNAMICBEDPLANE_H_
#define SRC_MOVEMENT_BEDPLANE_DYNAMICBEDPLANE_H_

#include <cstddef>
#include <cstdint>

enum class DynamicBedPlaneResult : uint8_t
{
	ok,
	nonFiniteInput,
	nozzleSeparationTooSmall,
	tooManyLeadscrews,
	slopeLimitExceeded,
	motorCorrectionLimitExceeded,
	motorSpreadLimitExceeded
};

struct DynamicBedPlaneCoefficients
{
	float intercept;			// correction at X=0
	float xSlope;			// correction change in mm per mm of X travel
};

struct DynamicBedPlaneLimits
{
	float minimumNozzleSeparation;
	float maximumAbsoluteXSlope;
	float maximumAbsoluteMotorCorrection;
	float maximumMotorSpread;
};

class DynamicBedPlane final
{
public:
	static constexpr size_t MaxLeadscrews = 4;

	// Solve p(x) = intercept + xSlope*x such that it passes through the
	// desired correction at both nozzles. The dynamic Y slope is held at zero.
	static DynamicBedPlaneResult SolveSharedY(float leftX,
										 float leftCorrection,
										 float rightX,
										 float rightCorrection,
										 const DynamicBedPlaneLimits& limits,
										 DynamicBedPlaneCoefficients& coefficients) noexcept;

	// Evaluate a previously solved dynamic plane at a physical X coordinate.
	static float CorrectionAtX(const DynamicBedPlaneCoefficients& coefficients, float x) noexcept;

	// Calculate ordered per-leadscrew corrections and enforce displacement
	// limits. leadscrewX must use the same order as M584 Z and M671.
	static DynamicBedPlaneResult CalculateLeadscrewCorrections(
		const DynamicBedPlaneCoefficients& coefficients,
		const float leadscrewX[],
		size_t numLeadscrews,
		const DynamicBedPlaneLimits& limits,
		float corrections[]) noexcept;

private:
	static bool IsFinite(float value) noexcept;
};

#endif /* SRC_MOVEMENT_BEDPLANE_DYNAMICBEDPLANE_H_ */
