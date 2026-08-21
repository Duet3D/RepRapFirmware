/*
 * DynamicBedPlane.cpp
 */

#include "DynamicBedPlane.h"

#include <cmath>

bool DynamicBedPlane::IsFinite(float value) noexcept
{
	return std::isfinite(value);
}

DynamicBedPlaneResult DynamicBedPlane::SolveSharedY(float leftX,
																	float leftCorrection,
																	float rightX,
																	float rightCorrection,
																	const DynamicBedPlaneLimits& limits,
																	DynamicBedPlaneCoefficients& coefficients) noexcept
{
	if (!IsFinite(leftX) || !IsFinite(leftCorrection)
		|| !IsFinite(rightX) || !IsFinite(rightCorrection)
		|| !IsFinite(limits.minimumNozzleSeparation)
		|| !IsFinite(limits.maximumAbsoluteXSlope))
	{
		return DynamicBedPlaneResult::nonFiniteInput;
	}

	const float separation = rightX - leftX;
	if (fabsf(separation) < limits.minimumNozzleSeparation)
	{
		return DynamicBedPlaneResult::nozzleSeparationTooSmall;
	}

	const float xSlope = (rightCorrection - leftCorrection) / separation;
	const float intercept = leftCorrection - xSlope * leftX;
	if (!IsFinite(xSlope) || !IsFinite(intercept))
	{
		return DynamicBedPlaneResult::nonFiniteInput;
	}
	if (fabsf(xSlope) > limits.maximumAbsoluteXSlope)
	{
		return DynamicBedPlaneResult::slopeLimitExceeded;
	}

	coefficients.intercept = intercept;
	coefficients.xSlope = xSlope;
	return DynamicBedPlaneResult::ok;
}

float DynamicBedPlane::CorrectionAtX(const DynamicBedPlaneCoefficients& coefficients, float x) noexcept
{
	return coefficients.intercept + coefficients.xSlope * x;
}

DynamicBedPlaneResult DynamicBedPlane::CalculateLeadscrewCorrections(
	const DynamicBedPlaneCoefficients& coefficients,
	const float leadscrewX[],
	size_t numLeadscrews,
	const DynamicBedPlaneLimits& limits,
	float corrections[]) noexcept
{
	if (numLeadscrews > MaxLeadscrews)
	{
		return DynamicBedPlaneResult::tooManyLeadscrews;
	}
	if (!IsFinite(coefficients.intercept) || !IsFinite(coefficients.xSlope)
		|| !IsFinite(limits.maximumAbsoluteMotorCorrection)
		|| !IsFinite(limits.maximumMotorSpread))
	{
		return DynamicBedPlaneResult::nonFiniteInput;
	}

	float minimum = 0.0;
	float maximum = 0.0;
	for (size_t i = 0; i < numLeadscrews; ++i)
	{
		if (!IsFinite(leadscrewX[i]))
		{
			return DynamicBedPlaneResult::nonFiniteInput;
		}
		const float correction = CorrectionAtX(coefficients, leadscrewX[i]);
		if (!IsFinite(correction))
		{
			return DynamicBedPlaneResult::nonFiniteInput;
		}
		if (fabsf(correction) > limits.maximumAbsoluteMotorCorrection)
		{
			return DynamicBedPlaneResult::motorCorrectionLimitExceeded;
		}

		corrections[i] = correction;
		if (i == 0 || correction < minimum)
		{
			minimum = correction;
		}
		if (i == 0 || correction > maximum)
		{
			maximum = correction;
		}
	}

	if (maximum - minimum > limits.maximumMotorSpread)
	{
		return DynamicBedPlaneResult::motorSpreadLimitExceeded;
	}
	return DynamicBedPlaneResult::ok;
}

DynamicBedPlaneResult DynamicBedPlane::CalculateTelemetry(
	float leftX,
	float leftCorrection,
	float rightX,
	float rightCorrection,
	const float leadscrewX[],
	size_t numLeadscrews,
	const DynamicBedPlaneLimits& limits,
	Telemetry& telemetry) noexcept
{
	Telemetry proposed = {};
	proposed.numLeadscrews = numLeadscrews;

	DynamicBedPlaneResult result = SolveSharedY(leftX, leftCorrection,
													 rightX, rightCorrection,
													 limits, proposed.coefficients);
	if (result != DynamicBedPlaneResult::ok)
	{
		return result;
	}

	result = CalculateLeadscrewCorrections(proposed.coefficients, leadscrewX,
													 numLeadscrews, limits,
													 proposed.leadscrewCorrections);
	if (result != DynamicBedPlaneResult::ok)
	{
		return result;
	}

	if (numLeadscrews != 0)
	{
		proposed.minimumLeadscrewCorrection = proposed.leadscrewCorrections[0];
		proposed.maximumLeadscrewCorrection = proposed.leadscrewCorrections[0];
		for (size_t i = 1; i < numLeadscrews; ++i)
		{
			if (proposed.leadscrewCorrections[i] < proposed.minimumLeadscrewCorrection)
			{
				proposed.minimumLeadscrewCorrection = proposed.leadscrewCorrections[i];
			}
			if (proposed.leadscrewCorrections[i] > proposed.maximumLeadscrewCorrection)
			{
				proposed.maximumLeadscrewCorrection = proposed.leadscrewCorrections[i];
			}
		}
		proposed.leadscrewCorrectionSpread = proposed.maximumLeadscrewCorrection
			- proposed.minimumLeadscrewCorrection;
	}

	telemetry = proposed;
	return DynamicBedPlaneResult::ok;
}
