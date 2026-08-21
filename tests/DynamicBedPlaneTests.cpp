#include "../src/Movement/BedPlane/DynamicBedPlane.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace
{
	constexpr float Tolerance = 0.000001f;

	bool NearlyEqual(float left, float right) noexcept
	{
		return fabsf(left - right) <= Tolerance;
	}
}

int main()
{
	const DynamicBedPlaneLimits permissiveLimits =
	{
		1.0f,		// minimum nozzle separation
		0.01f,		// maximum absolute X slope
		1.0f,		// maximum absolute motor correction
		2.0f		// maximum motor spread
	};

	DynamicBedPlaneCoefficients coefficients;
	auto result = DynamicBedPlane::SolveSharedY(-97.5f, -0.164f, 97.5f, 0.164f,
																 permissiveLimits, coefficients);
	assert(result == DynamicBedPlaneResult::ok);
	assert(NearlyEqual(DynamicBedPlane::CorrectionAtX(coefficients, -97.5f), -0.164f));
	assert(NearlyEqual(DynamicBedPlane::CorrectionAtX(coefficients, 97.5f), 0.164f));

	const float screwX[] = { 222.5f, 0.0f, -222.5f };
	float motorCorrections[3];
	result = DynamicBedPlane::CalculateLeadscrewCorrections(coefficients, screwX, 3,
																				 permissiveLimits, motorCorrections);
	assert(result == DynamicBedPlaneResult::ok);
	assert(NearlyEqual(motorCorrections[1], 0.0f));
	assert(NearlyEqual(motorCorrections[0], -motorCorrections[2]));
	assert(NearlyEqual(motorCorrections[0] - motorCorrections[2], (0.328f / 195.0f) * 445.0f));

	result = DynamicBedPlane::SolveSharedY(-97.5f, 0.12f, 97.5f, 0.12f,
																 permissiveLimits, coefficients);
	assert(result == DynamicBedPlaneResult::ok);
	result = DynamicBedPlane::CalculateLeadscrewCorrections(coefficients, screwX, 3,
																				 permissiveLimits, motorCorrections);
	assert(result == DynamicBedPlaneResult::ok);
	assert(NearlyEqual(motorCorrections[0], 0.12f));
	assert(NearlyEqual(motorCorrections[1], 0.12f));
	assert(NearlyEqual(motorCorrections[2], 0.12f));

	result = DynamicBedPlane::SolveSharedY(0.0f, 0.0f, 0.5f, 0.1f,
																 permissiveLimits, coefficients);
	assert(result == DynamicBedPlaneResult::nozzleSeparationTooSmall);

	result = DynamicBedPlane::SolveSharedY(0.0f, 0.0f,
																 std::numeric_limits<float>::quiet_NaN(), 0.1f,
																 permissiveLimits, coefficients);
	assert(result == DynamicBedPlaneResult::nonFiniteInput);

	const DynamicBedPlaneLimits tightSpreadLimits =
	{
		1.0f,
		0.01f,
		1.0f,
		0.5f
	};
	result = DynamicBedPlane::SolveSharedY(-97.5f, -0.164f, 97.5f, 0.164f,
																 tightSpreadLimits, coefficients);
	assert(result == DynamicBedPlaneResult::ok);
	result = DynamicBedPlane::CalculateLeadscrewCorrections(coefficients, screwX, 3,
																				 tightSpreadLimits, motorCorrections);
	assert(result == DynamicBedPlaneResult::motorSpreadLimitExceeded);
}
