/*
 * HangprinterKinematics.cpp
 *
 *  Created on: 24 Nov 2017
 *      Author: David
 */

#include "HangprinterKinematics.h"

#if SUPPORT_HANGPRINTER

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <GCodes/GCodes.h>
#include <Movement/Move.h>
#include <CAN/CanInterface.h>
#include <Math/Matrix.h>

#include <General/Portability.h>
#include <General/String.h>


// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(HangprinterKinematics, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry HangprinterKinematics::objectModelArrayTable[] =
{
	// 0. Coordinates of one anchor
	{
		nullptr,					// no lock needed
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 3; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const HangprinterKinematics *)self)->anchors[context.GetIndex(1)][context.GetLastIndex()], 1); }
	},
	// 1. Anchors
	{
		nullptr,					// no lock needed
		[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return HANGPRINTER_MAX_ANCHORS; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(self, 0, true); }
	}
};

constexpr size_t NumArrayTableEntriesInParents = 0;			// RoundbetKinematics and its parent class Kinematics have no array table entries

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE_WITH_PARENT(HangprinterKinematics, RoundBedKinematics, NumArrayTableEntriesInParents)

constexpr ObjectModelTableEntry HangprinterKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "anchors",		OBJECT_MODEL_FUNC_ARRAY(NumArrayTableEntriesInParents + 1), ObjectModelEntryFlags::none },
	{ "name",			OBJECT_MODEL_FUNC(self->GetName(true)), 					ObjectModelEntryFlags::none },
	{ "printRadius",	OBJECT_MODEL_FUNC(self->printRadius, 1), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t HangprinterKinematics::objectModelTableDescriptor[] = { 1, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(HangprinterKinematics, RoundBedKinematics)

// Constructor
HangprinterKinematics::HangprinterKinematics() noexcept
	: RoundBedKinematics(KinematicsType::hangprinter, SegmentationType(true, true, true))
{
	Init();
}

void HangprinterKinematics::Init() noexcept
{
	/* Naive buildup factor calculation (assumes cylindrical, straight line)
	 * line diameter: 0.5 mm
	 * spool height: 8.0 mm
	 * (line_cross_section_area)/(height*pi): ((0.5/2)*(0.5/2)*pi)/(8.0*pi) = 0.0078 mm
	 * Default buildup factor for 0.50 mm FireLine: 0.0078
	 * Default buildup factor for 0.39 mm FireLine: 0.00475
	 * In practice you might want to compensate a bit more or a bit less */
	constexpr float DefaultSpoolBuildupFactor = 0.007;
	/* Measure and set spool radii with M669 to achieve better accuracy */
	constexpr float DefaultSpoolRadii[HANGPRINTER_MAX_ANCHORS] = { 75.0, 75.0, 75.0, 75.0, 75.0}; // HP4 default
	/* If axis runs lines back through pulley system, set mechanical advantage accordingly with M669 */
	constexpr uint32_t DefaultMechanicalAdvantage[HANGPRINTER_MAX_ANCHORS] = { 2, 2, 2, 2, 4}; // HP4 default
	constexpr uint32_t DefaultLinesPerSpool[HANGPRINTER_MAX_ANCHORS] = { 1, 1, 1, 1, 1}; // HP4 default
	constexpr uint32_t DefaultMotorGearTeeth[HANGPRINTER_MAX_ANCHORS] = {  20,  20,  20,  20,  20}; // HP4 default
	constexpr uint32_t DefaultSpoolGearTeeth[HANGPRINTER_MAX_ANCHORS] = { 255, 255, 255, 255, 255}; // HP4 default
	constexpr uint32_t DefaultFullStepsPerMotorRev[HANGPRINTER_MAX_ANCHORS] = { 25, 25, 25, 25, 25};
	constexpr float DefaultMoverWeight_kg = 0.0F;
	constexpr float DefaultSpringKPerUnitLength = 20000.0F; // Garda 1.1 is somewhere in the range [20000, 100000]
	constexpr float DefaultMinForce_Newton[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	constexpr float DefaultMaxForce_Newton[HANGPRINTER_MAX_ANCHORS] = { 70.0F, 70.0F, 70.0F, 70.0F, 70.0F };
	constexpr float DefaultGuyWireLengths[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	constexpr float DefaultTorqueConstants[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	constexpr float DefaultAnchors[HANGPRINTER_MAX_ANCHORS][3] =
		{{    0.0, -2000.0, -100.0},
		 { 2000.0,  1000.0, -100.0},
		 {-2000.0,  1000.0, -100.0},
		 {    0.0,     0.0, 3000.0},
		 {    0.0,     0.0,    0.0},
		 {    0.0,     0.0,    0.0},
		 {    0.0,     0.0,    0.0},
		 {    0.0,     0.0,    0.0}};
	constexpr float DefaultPrintRadius = 1500.0;

	ARRAY_INIT(anchors, DefaultAnchors);
	anchorMode = HangprinterAnchorMode::None;
	numAnchors = DefaultNumAnchors;
	printRadius = DefaultPrintRadius;
	spoolBuildupFactor = DefaultSpoolBuildupFactor;
	ARRAY_INIT(spoolRadii, DefaultSpoolRadii);
	ARRAY_INIT(mechanicalAdvantage, DefaultMechanicalAdvantage);
	ARRAY_INIT(linesPerSpool, DefaultLinesPerSpool);
	ARRAY_INIT(motorGearTeeth, DefaultMotorGearTeeth);
	ARRAY_INIT(spoolGearTeeth, DefaultSpoolGearTeeth);
	ARRAY_INIT(fullStepsPerMotorRev, DefaultFullStepsPerMotorRev);
	moverWeight_kg = DefaultMoverWeight_kg;
	springKPerUnitLength = DefaultSpringKPerUnitLength;
	ARRAY_INIT(minForce_Newton, DefaultMinForce_Newton);
	ARRAY_INIT(maxForce_Newton, DefaultMaxForce_Newton);
	ARRAY_INIT(guyWireLengths, DefaultGuyWireLengths);
	ARRAY_INIT(torqueConstants, DefaultTorqueConstants);

	Recalc();
}

static inline float hyp3(float const a[3], float const b[3]) {
	return fastSqrtf(fsquare(a[2] - b[2]) + fsquare(a[1] - b[1]) + fsquare(a[0] - b[0]));
}

static inline float norm(float const a[3]) {
	return fastSqrtf(fsquare(a[2]) + fsquare(a[1]) + fsquare(a[0]));
}

// Recalculate the derived parameters
void HangprinterKinematics::Recalc() noexcept
{
	printRadiusSquared = fsquare(printRadius);

	// This is the difference between a "line length" and a "line position"
	// "line length" == ("line position" + "line length in origin")
	for (size_t i = 0; i < numAnchors; ++i)
	{
		distancesOrigin[i] = norm(anchors[i]);
	}

	//// Line buildup compensation
	float stepsPerUnitTimesRTmp[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	Move& move = reprap.GetMove();								 // No const because we want to set drive steps per unit
	for (size_t i = 0; i < numAnchors; ++i)
	{
		const uint8_t driver = move.GetAxisDriversConfig(i).driverNumbers[0].localDriver; // Only supports single driver
		bool dummy;
		stepsPerUnitTimesRTmp[i] =
			(
				(float)(mechanicalAdvantage[i])
				* fullStepsPerMotorRev[i]
				* move.GetMicrostepping(driver, dummy)
				* spoolGearTeeth[i]
			)
			/ (2.0 * Pi * motorGearTeeth[i]);

		const float stepsPerMmThisAxis = stepsPerUnitTimesRTmp[i] / spoolRadii[i];
		stepsPerMmAtOrigin[i] = stepsPerMmThisAxis;

		const float k2Val = -(float)(mechanicalAdvantage[i] * linesPerSpool[i]) * spoolBuildupFactor;
		if (fabsf(k2Val) <= 1.0e-9F)
		{
			k2[i] = 0.0F;
			k0[i] = 0.0F;
			useConstantSpoolModel[i] = true;
		}
		else
		{
			k2[i] = k2Val;
			k0[i] = 2.0 * stepsPerUnitTimesRTmp[i] / k2Val;
			useConstantSpoolModel[i] = false;
		}

		spoolRadiiSq[i] = spoolRadii[i] * spoolRadii[i];

		// Calculate the steps per unit that is correct at the origin
		move.SetDriveStepsPerMm(i, stepsPerMmThisAxis, 0);
	}

#if DUAL_CAN
	// Setting and reading of forces.
	ReadODrive3AxisForce({}, StringRef(nullptr, 0), torqueConstants, mechanicalAdvantage, spoolGearTeeth, motorGearTeeth, spoolRadii);
	SetODrive3TorqueMode({}, 0.0F, StringRef(nullptr, 0), mechanicalAdvantage, spoolGearTeeth, motorGearTeeth, spoolRadii);
#endif
}

const char *_ecv_array HangprinterKinematics::GetName(bool forStatusReport) const noexcept
{
    return "Hangprinter";
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters that affect the geometry. Set 'error' true if there was an error, otherwise leave it alone.
bool HangprinterKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) /*override*/
{
	bool requiresRehome = false;
	if (mCode == 669)
	{
		bool seen = false;
		const bool seenNonGeometry = TryConfigureSegmentation(gb);
		if (gb.Seen('N'))
		{
			numAnchors = gb.GetUIValue();
			seen = true;
		}
		for (size_t i = 0; i < numAnchors; ++i)
		{
			gb.TryGetFloatArray(ANCHOR_CHARS[i], 3, anchors[i], seen);
		}
		if (gb.Seen('P'))
		{
			printRadius = gb.GetPositiveFValue();
			seen = true;
		}

		if (seen)
		{
			Recalc();
			requiresRehome = true;
		}
		else if (!seenNonGeometry && !gb.Seen('K'))
		{
			Kinematics::Configure(mCode, gb, reply, error);
			for (size_t i = 0; i < numAnchors; ++i)
			{
				reply.lcatf("%c:%.2f, %.2f, %.2f",
					    ANCHOR_CHARS[i], (double)anchors[i][X_AXIS], (double)anchors[i][Y_AXIS], (double)anchors[i][Z_AXIS]);
			}
			reply.lcatf("P:Print radius: %.1f", (double)printRadius);
		}
	}
	else if (mCode == 666)
	{
		bool geometryParamSeen = false;
		bool seenFlexParam = false;
		// 0=None, 1=last-top, 2=all-top, 3-half-top, etc
		uint32_t unsignedAnchorMode = (uint32_t)anchorMode;
		if (gb.TryGetUIValue('A', unsignedAnchorMode, geometryParamSeen))
		{
			if (unsignedAnchorMode <= (uint32_t)HangprinterAnchorMode::AllOnTop) {
				anchorMode = (HangprinterAnchorMode)unsignedAnchorMode;
			}
		}
		gb.TryGetFValue('Q', spoolBuildupFactor, geometryParamSeen);
		gb.TryGetFloatArray('R', numAnchors, spoolRadii, geometryParamSeen);
		gb.TryGetUIArray('U', numAnchors, mechanicalAdvantage, geometryParamSeen);
		gb.TryGetUIArray('O', numAnchors, linesPerSpool, geometryParamSeen);
		gb.TryGetUIArray('L', numAnchors, motorGearTeeth, geometryParamSeen);
		gb.TryGetUIArray('H', numAnchors, spoolGearTeeth, geometryParamSeen);
		gb.TryGetUIArray('J', numAnchors, fullStepsPerMotorRev, geometryParamSeen);
		gb.TryGetFValue('W', moverWeight_kg, geometryParamSeen);
		gb.TryGetFValue('S', springKPerUnitLength, geometryParamSeen);
		gb.TryGetFloatArray('I', numAnchors, minForce_Newton, geometryParamSeen);
		gb.TryGetFloatArray('X', numAnchors, maxForce_Newton, geometryParamSeen);
		gb.TryGetFloatArray('Y', numAnchors, guyWireLengths, geometryParamSeen);
		gb.TryGetFloatArray('C', numAnchors, torqueConstants, geometryParamSeen);
		int32_t flexCommand = 0;
		if (gb.TryGetIValue('F', flexCommand, seenFlexParam))
		{
			bool validFlex = true;
			if (flexCommand == 0)
			{
				flexEnabled = false;
			}
			else if (flexCommand == 1)
			{
				flexEnabled = true;
				flexAlgorithm = FlexAlgorithm::Qp;
			}
			else if (flexCommand == 2)
			{
				flexEnabled = true;
				flexAlgorithm = FlexAlgorithm::Tikhonov;
			}
			else
			{
				reply.catf("Unknown flex algorithm: %d\n", (int)flexCommand);
				error = true;
				validFlex = false;
			}
			if (validFlex)
			{
				seenFlexParam = true;
			}
			else
			{
				seenFlexParam = false;
			}
		}
		gb.TryGetBValue('B', ignoreGravity, geometryParamSeen);
		gb.TryGetBValue('P', ignorePretension, geometryParamSeen);
		if (geometryParamSeen || seenFlexParam)
		{
			Recalc();
		}
		else
		{
			reply.printf("M666 A%u Q%.4f\n", (unsigned)anchorMode, (double)spoolBuildupFactor);

			reply.lcatf("R%.2f", (double)spoolRadii[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.2f", (double)spoolRadii[i]);
			}

			reply.lcatf("U%d", (int)mechanicalAdvantage[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%d", (int)mechanicalAdvantage[i]);
			}

			reply.lcatf("O%d", (int)linesPerSpool[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%d", (int)linesPerSpool[i]);
			}

			reply.lcatf("L%d", (int)motorGearTeeth[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%d", (int)motorGearTeeth[i]);
			}

			reply.lcatf("H%d", (int)spoolGearTeeth[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%d", (int)spoolGearTeeth[i]);
			}

			reply.lcatf("J%d", (int)fullStepsPerMotorRev[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%d", (int)fullStepsPerMotorRev[i]);
			}
			reply.lcatf("W%.2f\n", (double)moverWeight_kg);
			reply.lcatf("S%.2f\n", (double)springKPerUnitLength);

			reply.lcatf("I%.1f", (double)minForce_Newton[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.1f", (double)minForce_Newton[i]);
			}

			reply.lcatf("X%.1f", (double)maxForce_Newton[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.1f", (double)maxForce_Newton[i]);
			}

			reply.lcatf("Y%.1f", (double)guyWireLengths[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.1f", (double)guyWireLengths[i]);
			}

			reply.lcatf("C%.4f", (double)torqueConstants[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.4f", (double)torqueConstants[i]);
			}
			const uint32_t flexValue = flexEnabled ? ((flexAlgorithm == FlexAlgorithm::Tikhonov) ? 2u : 1u) : 0u;
			reply.lcatf("F%u B%u P%u", static_cast<unsigned int>(flexValue), ignoreGravity ? 1u : 0u, ignorePretension ? 1u : 0u);
		}
		requiresRehome = geometryParamSeen;
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
	return requiresRehome;
}

// Convert Cartesian coordinates to motor coordinates, returning true if successful
MovementError HangprinterKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
													size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	float distances[HANGPRINTER_MAX_ANCHORS];
	for (size_t i = 0; i < numAnchors; ++i)
	{
		distances[i] = hyp3(machinePos, anchors[i]);
	}

	float linePos[HANGPRINTER_MAX_ANCHORS];
	if (flexEnabled)
	{
		float flex[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
		FlexDistances(machinePos, distances, flex);
		for (size_t i = 0; i < numAnchors; ++i)
		{
			linePos[i] = distances[i] - distancesOrigin[i] + flex[i];
		}
	}
	else
	{
		for (size_t i = 0; i < numAnchors; ++i)
		{
			linePos[i] = distances[i] - distancesOrigin[i];
		}
	}

	MovementError rslt = MovementError::ok;
	for (size_t i = 0; i < numAnchors; ++i)
	{
		if (useConstantSpoolModel[i])
		{
			RoundToInt32(rslt, linePos[i] * stepsPerMmAtOrigin[i], motorPos[i]);
		}
		else
		{
			// This logic should be called LinePosToMotorPos
			RoundToInt32(rslt, k0[i] * (fastSqrtf(spoolRadiiSq[i] + linePos[i] * k2[i]) - spoolRadii[i]), motorPos[i]);
		}
	}

	return MovementError::ok;
}


inline float HangprinterKinematics::MotorPosToLinePos(const int32_t motorPos, size_t axis) const noexcept
{
	if (useConstantSpoolModel[axis])
	{
		return (float)motorPos / stepsPerMmAtOrigin[axis];
	}
	return (fsquare(motorPos / k0[axis] + spoolRadii[axis]) - spoolRadiiSq[axis]) / k2[axis];
}


void HangprinterKinematics::FlexDistances(float const machinePos[3],
                                          float flex[HANGPRINTER_MAX_ANCHORS]) const noexcept {
	float distances[HANGPRINTER_MAX_ANCHORS];
	for (size_t i = 0; i < numAnchors; ++i)
	{
		distances[i] = hyp3(machinePos, anchors[i]);
	}
	FlexDistances(machinePos, distances, flex);
}


void HangprinterKinematics::FlexDistances(float const machinePos[3], float const distances[HANGPRINTER_MAX_ANCHORS],
                                          float flex[HANGPRINTER_MAX_ANCHORS]) const noexcept {
	float springKs[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	for (size_t i = 0; i < numAnchors; ++i) {
		springKs[i] = SpringK(distances[i] * mechanicalAdvantage[i] + guyWireLengths[i]);
	}

	float F[HANGPRINTER_MAX_ANCHORS] = { 0.0F }; // desired force in each direction
	StaticForces(machinePos, F);

	for (size_t i = 0; i < numAnchors; ++i) {
		flex[i] = -F[i] / (springKs[i] * mechanicalAdvantage[i]);
	}
}

// Start of forward kinematics stuff
float HangprinterKinematics::ResidualsAndDerivatives(
                                     const float linePositions[HANGPRINTER_MAX_ANCHORS],
                                     float const pos[3],
                                     float residuals[HANGPRINTER_MAX_ANCHORS],
                                     float jacobian[HANGPRINTER_MAX_ANCHORS][3],
                                     float (*hessians)[3][3]) const noexcept {
	float baseImpact[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float impactPlus[HANGPRINTER_MAX_ANCHORS][3] = { 0.0F };
	float impactMinus[HANGPRINTER_MAX_ANCHORS][3] = { 0.0F };
	constexpr float impactStep = 1e-3F;

	if (flexEnabled) {
		FlexDistances(pos, baseImpact);
		for (size_t axis = 0; axis < 3; ++axis) {
			float shifted[3] = { pos[0], pos[1], pos[2] };
			shifted[axis] += impactStep;
			FlexDistances(shifted, impactPlus[axis]);
			shifted[axis] = pos[axis];
			shifted[axis] -= impactStep;
			FlexDistances(shifted, impactMinus[axis]);
		}
	}

	float cost = 0.0F;
	for (size_t i = 0; i < numAnchors; ++i) {
		float diff[3] = { pos[0] - anchors[i][0], pos[1] - anchors[i][1], pos[2] - anchors[i][2] };
		float distance = norm(diff);
		if (distance < 1e-6F) {
			distance = 1e-6F;
		}
		const float invLen = 1.0F / distance;
		const float invLen3 = invLen * invLen * invLen;

		const float flex = baseImpact[i];
		const float foundLinePos = distance - distancesOrigin[i] + flex;
		residuals[i] = foundLinePos - linePositions[i];

		if (hessians) {
			float (&H)[3][3] = hessians[i];
			for (size_t r = 0; r < 3; ++r) {
				for (size_t c = 0; c < 3; ++c) {
					const float id = (r == c) ? invLen : 0.0F;
					H[r][c] = id - diff[r] * diff[c] * invLen3;
				}
			}
		}

		for (size_t axis = 0; axis < 3; ++axis) {
			const float impactDeriv = (impactPlus[axis][i] - impactMinus[axis][i]) / (2.0F * impactStep);
			jacobian[i][axis] = diff[axis] * invLen + impactDeriv;
		}

		cost += 0.5F * residuals[i] * residuals[i];
	}
	return cost;
}

static bool solveNormalSystem(const float JTJ[3][3], const float rhs[3], float delta[3]) {
	FixedMatrix<float, 3, 4> system{};
	for (size_t r = 0; r < 3; ++r) {
		for (size_t c = 0; c < 3; ++c) {
			system(r, c) = JTJ[r][c];
		}
		system(r, 3) = rhs[r];
	}
	if (!system.GaussJordan(3, 4)) {
		return false;
	}
	delta[0] = system(0, 3);
	delta[1] = system(1, 3);
	delta[2] = system(2, 3);
	return true;
}

void HangprinterKinematics::AccumulateJtJandGrad(
       float const J[HANGPRINTER_MAX_ANCHORS][3],
       float const residuals[HANGPRINTER_MAX_ANCHORS],
       float JTJ[3][3], float grad[3]) const noexcept {
	for (size_t i = 0; i < 3; ++i) {
		grad[i] = 0.0F;
		for (size_t j = 0; j < 3; ++j) {
			JTJ[i][j] = 0.0F;
		}
	}

	for (size_t i = 0; i < numAnchors; ++i) {
		grad[0] += J[i][0] * residuals[i];
		grad[1] += J[i][1] * residuals[i];
		grad[2] += J[i][2] * residuals[i];

		JTJ[0][0] += J[i][0] * J[i][0];
		JTJ[0][1] += J[i][0] * J[i][1];
		JTJ[0][2] += J[i][0] * J[i][2];
		JTJ[1][0] += J[i][1] * J[i][0];
		JTJ[1][1] += J[i][1] * J[i][1];
		JTJ[1][2] += J[i][1] * J[i][2];
		JTJ[2][0] += J[i][2] * J[i][0];
		JTJ[2][1] += J[i][2] * J[i][1];
		JTJ[2][2] += J[i][2] * J[i][2];
	}
}


// A 3T (no rotations) version of Henry Mahnke & Ryan J. Caverly's
// "Fast and Reliable Iterative Cable-Driven Parallel Robot Forward Kinematics: A Quadratic Approximation Approach"
// https://doi.org/10.1007/978-3-031-94608-0_1
// Extensive testing and comparisons to other methods done in
// github.com/tobbelobb/hangprinter-forward-transform
HangprinterKinematics::SolverResult HangprinterKinematics::SolveHybrid(
                                      const float linePositions[HANGPRINTER_MAX_ANCHORS],
                                      float initial[3],
                                      float eta,
                                      float tol,
                                      size_t halleyIters,
                                      size_t maxIters) const noexcept {
	SolverResult result{};
	result.pos[0] = initial[0];
	result.pos[1] = initial[1];
	result.pos[2] = initial[2];
	size_t iter = 0;
	for (; iter < halleyIters && iter < maxIters; ++iter) {
		float residuals[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
		float J[HANGPRINTER_MAX_ANCHORS][3];
		float H[HANGPRINTER_MAX_ANCHORS][3][3];
		result.cost =
		    ResidualsAndDerivatives(linePositions, result.pos, residuals, J, H);

		float JTJ[3][3];
		float grad[3];
		AccumulateJtJandGrad(J, residuals, JTJ, grad);
		JTJ[0][0] += eta;
		JTJ[1][1] += eta;
		JTJ[2][2] += eta;

		float deltaLm[3] = { 0.0F };
		float rhs1[3] = {-grad[0], -grad[1], -grad[2]};
		if (!solveNormalSystem(JTJ, rhs1, deltaLm)) {
			break;
		}

		float Hbar[HANGPRINTER_MAX_ANCHORS][3];
		for (size_t i = 0; i < numAnchors; ++i) {
			Hbar[i][0] = deltaLm[0] * H[i][0][0] + deltaLm[1] * H[i][1][0] + deltaLm[2] * H[i][2][0];
			Hbar[i][1] = deltaLm[0] * H[i][0][1] + deltaLm[1] * H[i][1][1] + deltaLm[2] * H[i][2][1];
			Hbar[i][2] = deltaLm[0] * H[i][0][2] + deltaLm[1] * H[i][1][2] + deltaLm[2] * H[i][2][2];
		}

		float Jbar[HANGPRINTER_MAX_ANCHORS][3];
		for (size_t i = 0; i < numAnchors; ++i) {
			Jbar[i][0] = J[i][0] + 0.5F * Hbar[i][0];
			Jbar[i][1] = J[i][1] + 0.5F * Hbar[i][1];
			Jbar[i][2] = J[i][2] + 0.5F * Hbar[i][2];
		}

		float JTJ2[3][3];
		float grad2[3];
		AccumulateJtJandGrad(Jbar, residuals, JTJ2, grad2);
		JTJ2[0][0] += eta;
		JTJ2[1][1] += eta;
		JTJ2[2][2] += eta;

		float delta[3] = { 0.0F };
		float rhs2[3] = {-grad2[0], -grad2[1], -grad2[2]};
		if (!solveNormalSystem(JTJ2, rhs2, delta)) {
			break;
		}

		result.pos[0] = result.pos[0] + delta[0];
		result.pos[1] = result.pos[1] + delta[1];
		result.pos[2] = result.pos[2] + delta[2];
		result.iterations = iter + 1;
		if (norm(delta) < tol) {
			result.converged = true;
			break;
		}
	}

	for (; iter < maxIters && !result.converged; ++iter) {
		float residuals[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
		float J[HANGPRINTER_MAX_ANCHORS][3];
		result.cost =
		    ResidualsAndDerivatives(linePositions, result.pos, residuals, J, nullptr);

		float JTJ[3][3];
		float grad[3];
		AccumulateJtJandGrad(J, residuals, JTJ, grad);
		JTJ[0][0] += eta;
		JTJ[1][1] += eta;
		JTJ[2][2] += eta;

		float delta[3] = { 0.0F };
		float rhs3[3] = {-grad[0], -grad[1], -grad[2]};
		if (!solveNormalSystem(JTJ, rhs3, delta)) {
			break;
		}

		result.pos[0] = result.pos[0] + delta[0];
		result.pos[1] = result.pos[1] + delta[1];
		result.pos[2] = result.pos[2] + delta[2];
		result.iterations = iter + 1;
		if (norm(delta) < tol) {
			result.converged = true;
			break;
		}
	}

	float residuals[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float Jtmp[HANGPRINTER_MAX_ANCHORS][3];
	result.cost =
	    ResidualsAndDerivatives(linePositions, result.pos, residuals, Jtmp, nullptr);
	return result;
}

// Convert motor coordinates to machine coordinates.
void HangprinterKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	// Define the line positions the solver should try to find
	float linePositions[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	for (size_t i = 0; i < numAnchors; ++i) {
		linePositions[i] = MotorPosToLinePos(motorPos[i], i);
	};

	float guess[3] = { 0.0F };
	SolverResult const res = SolveHybrid(linePositions, guess, 1e-3F, 1e-3F, 3, 30);
	if (!res.converged || res.cost > 10.0F) {
		return;
	}
	machinePos[0] = res.pos[0];
	machinePos[1] = res.pos[1];
	machinePos[2] = res.pos[2];
}

// End of forward kinematics stuff


static bool isSameSide(float const v0[3], float const v1[3], float const v2[3], float const v3[3], float const p[3]){
	float const h0[3] = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
	float const h1[3] = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};

	float const normal[3] = {
		h0[1]*h1[2] - h0[2]*h1[1],
		h0[2]*h1[0] - h0[0]*h1[2],
		h0[0]*h1[1] - h0[1]*h1[0]
	};

	float const dh0[3] = {v3[0] - v0[0], v3[1] - v0[1], v3[2] - v0[2]};
	float const dh1[3] = { p[0] - v0[0],  p[1] - v0[1],  p[2] - v0[2]};

	float const dot0 = dh0[0]*normal[0] + dh0[1]*normal[1] + dh0[2]*normal[2];
	float const dot1 = dh1[0]*normal[0] + dh1[1]*normal[1] + dh1[2]*normal[2];
	return dot0*dot1 > 0.0F;
}

bool HangprinterKinematics::IsInsidePyramidSides(float const coords[3]) const noexcept
{
	bool reachable = true;

	// Check all the planes defined by triangle sides in the pyramid
	for (size_t i = 0; reachable && i < numAnchors - 1; ++i) {
		reachable = reachable && isSameSide(anchors[i], anchors[(i+1) % (numAnchors - 1)], anchors[numAnchors - 1], anchors[(i+2) % (numAnchors - 1)], coords);
	}
	return reachable;
}

bool HangprinterKinematics::IsInsidePrismSides(float const coords[3], unsigned const discount_last) const noexcept
{
	bool reachable = true;

	// For each side of the base, check the plane formed by side and another point bellow them in z.
	for (size_t i = 0; reachable && i < numAnchors - discount_last; ++i) {
		float const lower_point[3] = {anchors[i][0], anchors[i][1], anchors[i][2] - 1};
		reachable = reachable && isSameSide(anchors[i], anchors[(i+1) % (numAnchors - 1)], lower_point, anchors[(i+2) % (numAnchors - 1)], coords);
	}
	return reachable;
}

// For each triangle side in a pseudo-pyramid, check if the point is inside the pyramid (Except for the base)
// Also check that any point below the line between two exterior anchors (all anchors are exterior except for the last one)
// is in the "inside part" all the way down to min_Z, however low it may be.
// To further limit the movements in the X and Y axes one can simply set a smaller print radius.
bool HangprinterKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept /*override*/
{
	float const coords[3] = {axesCoords[X_AXIS], axesCoords[Y_AXIS], axesCoords[Z_AXIS]};
	bool reachable = true;

	switch (anchorMode) {
		case HangprinterAnchorMode::None:
			return true;

		// This reaches a pyramid on top of the lower prism if the bed is below the lower anchors
		case HangprinterAnchorMode::LastOnTop:
		default:
			reachable = IsInsidePyramidSides(coords);
			return reachable && IsInsidePrismSides(coords, 1);

		case HangprinterAnchorMode::AllOnTop:
			return IsInsidePrismSides(coords, 0);
	}

	return reachable;
}

// Limit the Cartesian position that the user wants to move to returning true if we adjusted the position
LimitPositionResult HangprinterKinematics::LimitPosition(float finalCoords[], const float * null initialCoords,
															size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
	bool limited = false;
	if (axesToLimit.Intersects(XyzAxes))
	{
		// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
		// Skip this check if axes have not been homed, so that extruder-only moves are allowed before homing
		// Constrain the move to be within the build radius
		const float diagonalSquared = fsquare(finalCoords[X_AXIS]) + fsquare(finalCoords[Y_AXIS]);
		if (diagonalSquared > printRadiusSquared)
		{
			const float factor = fastSqrtf(printRadiusSquared / diagonalSquared);
			finalCoords[X_AXIS] *= factor;
			finalCoords[Y_AXIS] *= factor;
			limited = true;
		}

		if (applyM208Limits)
		{
			if (finalCoords[Z_AXIS] < reprap.GetMove().AxisMinimum(Z_AXIS))
			{
				finalCoords[Z_AXIS] = reprap.GetMove().AxisMinimum(Z_AXIS);
				limited = true;
			}
			else if (finalCoords[Z_AXIS] > reprap.GetMove().AxisMaximum(Z_AXIS))
			{
				finalCoords[Z_AXIS] = reprap.GetMove().AxisMaximum(Z_AXIS);
				limited = true;
			}
		}
	}

	//TODO check intermediate positions, especially uif.when we support an offset radius
	return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void HangprinterKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap HangprinterKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept
{
	filename.copy("homeall.g");
	return AxesBitmap();
}

float HangprinterKinematics::GetEndstopPosition(size_t drive, bool highEnd) noexcept
{
	// Hangprinter homing is not supported
	return Kinematics::GetEndstopPosition(drive, highEnd);
}

// Return the drivers that control an axis or anchor line
LogicalDrivesBitmap HangprinterKinematics::GetControllingDrives(size_t axis, bool forHoming) const noexcept
{
	return (forHoming || axis >= numAnchors)
			? LogicalDrivesBitmap::MakeFromBits(axis)
				: LogicalDrivesBitmap::MakeLowestNBits(numAnchors);
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap HangprinterKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	// If all of X, Y and Z have been specified then we know the positions of all 4 spool motors, otherwise we don't
	if ((g92Axes & XyzAxes) == XyzAxes)
	{
		for (size_t i = 3; i < numAnchors; ++i)
		{
			g92Axes.SetBit(i);
		}
	}
	else
	{
		g92Axes &= ~XyzAxes;
	}
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap HangprinterKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
	if (axesMoving.Intersects(XyzAxes))
	{
		axesMoving |= XyzAxes;
	}
	return axesMoving;
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Write the parameters to a file, returning true if success
bool HangprinterKinematics::WriteCalibrationParameters(FileStore *f) const noexcept
{
	bool ok = false;
	String<255> scratchString;

	scratchString.printf("; Hangprinter parameters\n");
	scratchString.printf("M669 K6 ");
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf("N%zu", numAnchors);
	for (size_t i = 0; i < numAnchors; ++i)
	{
		scratchString.catf("%c%.3f:%.3f:%.3f ", ANCHOR_CHARS[i], (double)anchors[i][X_AXIS], (double)anchors[i][Y_AXIS], (double)anchors[i][Z_AXIS]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" P%.1f", (double)printRadius);
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf("M666 A%u Q%.6f ", (unsigned)anchorMode, (double)spoolBuildupFactor);
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf("R%.3f", (double)spoolRadii[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.3f", (double)spoolRadii[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" U%.3f", (double)mechanicalAdvantage[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.3f", (double)mechanicalAdvantage[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" O%.3f", (double)linesPerSpool[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.3f", (double)linesPerSpool[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" L%.3f", (double)motorGearTeeth[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.3f", (double)motorGearTeeth[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" H%.3f", (double)spoolGearTeeth[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.3f", (double)spoolGearTeeth[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" J%.3f", (double)fullStepsPerMotorRev[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.3f", (double)fullStepsPerMotorRev[i]);
	}
	ok = f->Write(scratchString.c_str());

	scratchString.printf(" W%.2f S%.2f", (double)moverWeight_kg, (double)springKPerUnitLength);
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" I%.1f", (double)minForce_Newton[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.1f", (double)minForce_Newton[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" X%.1f", (double)maxForce_Newton[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.1f", (double)maxForce_Newton[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" Y%.1f", (double)guyWireLengths[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.1f", (double)guyWireLengths[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" C%.4f", (double)torqueConstants[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.4f", (double)torqueConstants[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	uint32_t flexValue = flexEnabled ? ((flexAlgorithm == FlexAlgorithm::Tikhonov) ? 2u : 1u) : 0u;
	scratchString.printf(" F%u B%u P%u\n", static_cast<unsigned int>(flexValue), ignoreGravity ? 1u : 0u, ignorePretension ? 1u : 0u);
	ok = f->Write(scratchString.c_str());

	return ok;
}

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool HangprinterKinematics::WriteResumeSettings(FileStore *f) const noexcept
{
	return WriteCalibrationParameters(f);
}

#endif

// Print all the parameters for debugging
void HangprinterKinematics::PrintParameters(const StringRef& reply) const noexcept
{
	reply.printf("Anchor coordinates");
	for (size_t i = 0; i < numAnchors; ++i)
	{
		reply.catf(" (%.2f,%.2f,%.2f)", (double)anchors[i][X_AXIS], (double)anchors[i][Y_AXIS], (double)anchors[i][Z_AXIS]);
	}
	reply.cat("\n");
}

#if DUAL_CAN
HangprinterKinematics::ODriveAnswer HangprinterKinematics::GetODrive3MotorCurrent(DriverId driver, const StringRef& reply) THROWS(GCodeException)
{
	const uint8_t cmd = CANSimple::MSG_GET_IQ;
	CanMessageBuffer * buf = CanInterface::ODrive::PrepareSimpleMessage(driver, reply);
	if (buf == nullptr)
	{
		return {};
	}
	buf->id = CanInterface::ODrive::ArbitrationId(driver, cmd);
	buf->remote = true; // Indicates that we expect an answer
	CanInterface::ODrive::FlushCanReceiveHardware();
	CanInterface::SendPlainMessageNoFree(buf);
	bool ok = CanInterface::ODrive::GetExpectedSimpleMessage(buf, driver, cmd, reply);
	float motorCurrent = 0.0;
	if (ok)
	{
		size_t const expectedResponseLength = 8;
		ok = (buf->dataLength == expectedResponseLength);
		if (ok)
		{
			motorCurrent = LoadLEF32(&(buf->msg.raw[4]));
		}
		else
		{
			reply.printf("Unexpected response length: %zu", buf->dataLength);
		}
	}
	CanMessageBuffer::Free(buf);
	if (ok)
	{
		return {true, motorCurrent};
	}
	return {};
}
#endif // DUAL_CAN

#if DUAL_CAN
HangprinterKinematics::ODriveAnswer HangprinterKinematics::GetODrive3EncoderEstimate(DriverId const driver, bool const makeReference, const StringRef& reply, bool const subtractReference) THROWS(GCodeException)
{
	const uint8_t cmd = CANSimple::MSG_GET_ENCODER_ESTIMATES;
	static CanAddress seenDrives[HANGPRINTER_MAX_ANCHORS] = { 0 };
	static float referencePositions[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	static size_t numSeenDrives = 0;
	size_t thisDriveIdx = 0;

	while (thisDriveIdx < numSeenDrives && seenDrives[thisDriveIdx] != driver.boardAddress)
	{
		thisDriveIdx++;
	}
	bool const newOne = (thisDriveIdx == numSeenDrives);
	if (newOne)
	{
		if (numSeenDrives < numAnchors)
		{
			seenDrives[thisDriveIdx] = driver.boardAddress;
			numSeenDrives++;
		}
		else // we don't have space for a new one
		{
			reply.printf("Max CAN addresses we can reference is %zu. Can't reference board %d.", numAnchors, driver.boardAddress);
			numSeenDrives = numAnchors;
			return {};
		}
	}

	CanMessageBuffer * buf = CanInterface::ODrive::PrepareSimpleMessage(driver, reply);
	if (buf == nullptr)
	{
		return {};
	}

	buf->id = CanInterface::ODrive::ArbitrationId(driver, cmd);
	buf->remote = true; // Indicates that we expect an answer
	CanInterface::ODrive::FlushCanReceiveHardware();

	CanInterface::SendPlainMessageNoFree(buf);

	bool ok = CanInterface::ODrive::GetExpectedSimpleMessage(buf, driver, cmd, reply);
	float encoderEstimate = 0.0;
	if (ok)
	{
		size_t const expectedResponseLength = 8;
		ok = (buf->dataLength == expectedResponseLength);
		if (ok)
		{
			encoderEstimate = LoadLEF32(buf->msg.raw);
			if (makeReference)
			{
				referencePositions[thisDriveIdx] = encoderEstimate;
			}
			// Subtract reference value
			if (subtractReference)
			{
				encoderEstimate = encoderEstimate - referencePositions[thisDriveIdx];
			}
		}
		else
		{
			reply.printf("Unexpected response length: %zu", buf->dataLength);
		}
	}
	CanMessageBuffer::Free(buf);

	if (newOne && !ok)
	{
		seenDrives[thisDriveIdx] = 0;
		numSeenDrives--;
	}

	if (ok)
	{
		return {true, encoderEstimate};
	}

	return {};
}
#endif // DUAL_CAN

namespace
{
bool TryGetDriverDirectionForwards(DriverId driver, bool& forwards) noexcept
{
	const Move& move = reprap.GetMove();
	if (driver.IsLocal() && driver.localDriver >= move.GetNumActualDirectDrivers())
	{
		return false;
	}
	forwards = move.GetDirectionValue(driver);
	return true;
}
}

#if DUAL_CAN
GCodeResult HangprinterKinematics::ReadODrive3AxisForce(DriverId const driver, const StringRef& reply, float setTorqueConstants[], uint32_t setMechanicalAdvantage[], uint32_t setSpoolGearTeeth[], uint32_t setMotorGearTeeth[], float setSpoolRadii[]) THROWS(GCodeException)
{
	static float torqueConstants_[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	static uint32_t mechanicalAdvantage_[HANGPRINTER_MAX_ANCHORS] = { 0 };
	static uint32_t spoolGearTeeth_[HANGPRINTER_MAX_ANCHORS] = { 0 };
	static uint32_t motorGearTeeth_[HANGPRINTER_MAX_ANCHORS] = { 0 };
	static float spoolRadii_[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	if (setTorqueConstants != nullptr && setMechanicalAdvantage != nullptr && setSpoolGearTeeth != nullptr &&
			setMotorGearTeeth != nullptr && setSpoolRadii != nullptr) {
		for(size_t i{0}; i < numAnchors; ++i) {
			torqueConstants_[i] = setTorqueConstants[i];
			mechanicalAdvantage_[i] = setMechanicalAdvantage[i];
			spoolGearTeeth_[i] = setSpoolGearTeeth[i];
			motorGearTeeth_[i] = setMotorGearTeeth[i];
			spoolRadii_[i] = setSpoolRadii[i];
		}
		return GCodeResult::ok;
	}

	HangprinterKinematics::ODriveAnswer const motorCurrent = GetODrive3MotorCurrent(driver, reply);
	if (motorCurrent.valid)
	{
		size_t const boardIndex = driver.boardAddress - 40;
		if (boardIndex < 0 or boardIndex > 9) {
			reply.catf("Board address not between 40 and 49: %d", driver.boardAddress);
			return GCodeResult::error;
		}
		// This force calculation if very rough, assuming perfect data from ODrive,
		// perfect transmission between motor gear and spool gear,
		// the exact same line buildup on spool as we have at the origin,
		// and no losses from any of the bearings or eyelets in the motion system.
		float motorTorque_Nm = motorCurrent.value * torqueConstants_[boardIndex];
		bool directionForwards = true;
		if (TryGetDriverDirectionForwards(driver, directionForwards) && directionForwards)
		{
			motorTorque_Nm = -motorTorque_Nm;
		}
		float const lineTension = 1000.0 * (motorTorque_Nm * (spoolGearTeeth_[boardIndex]/motorGearTeeth_[boardIndex])) / spoolRadii_[boardIndex];
		float const force = lineTension * mechanicalAdvantage_[boardIndex];
		reply.catf("%.2f, ", (double)(force));
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}
#endif // DUAL_CAN

#if DUAL_CAN
GCodeResult HangprinterKinematics::ReadODrive3Encoder(DriverId const driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	HangprinterKinematics::ODriveAnswer const estimate = GetODrive3EncoderEstimate(driver, gb.Seen('S'), reply, true);
	if (estimate.valid)
	{
		float directionCorrectedEncoderValue = estimate.value;
		bool directionForwards = true;
		if (TryGetDriverDirectionForwards(driver, directionForwards) && !directionForwards)
		{
			directionCorrectedEncoderValue *= -1.0F;
		}
		reply.catf("%.2f, ", (double)(directionCorrectedEncoderValue * 360.0));
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}
#endif // DUAL_CAN

namespace {
GCodeResult ComputeODrive3TorqueFromForceInternal(
	DriverId const driver, float const force_Newton,
	uint32_t const mechanicalAdvantage[], uint32_t const spoolGearTeeth[], uint32_t const motorGearTeeth[],
	float const spoolRadii[], float& motorTorque_Nm, bool& positionMode, const StringRef& reply) noexcept
{
	constexpr float MIN_TORQUE_N = 0.001F;
	positionMode = false;
	motorTorque_Nm = 0.0F;
	if (fabsf(force_Newton) < MIN_TORQUE_N)
	{
		positionMode = true;
		return GCodeResult::ok;
	}

	const int boardIndex = (int)driver.boardAddress - 40;
	if (boardIndex < 0 || boardIndex > 9)
	{
		reply.catf("Board address not between 40 and 49: %d", driver.boardAddress);
		return GCodeResult::error;
	}

	float const lineTension_N = force_Newton / mechanicalAdvantage[boardIndex];
	float const spoolTorque_Nm = lineTension_N * spoolRadii[boardIndex] * 0.001F;
	float motorTorque = spoolTorque_Nm * motorGearTeeth[boardIndex] / spoolGearTeeth[boardIndex];
	motorTorque = std::abs(motorTorque);
	bool directionForwards = true;
	if (TryGetDriverDirectionForwards(driver, directionForwards) && directionForwards)
	{
		motorTorque = -motorTorque;
	}
	motorTorque_Nm = motorTorque;
	return GCodeResult::ok;
}
} // namespace

GCodeResult HangprinterKinematics::ComputeODrive3TorqueFromForce(DriverId const driver, float force_Newton,
																																	float& motorTorque_Nm, bool& positionMode,
																																	const StringRef& reply) const noexcept
{
	return ComputeODrive3TorqueFromForceInternal(
		driver, force_Newton,
		mechanicalAdvantage, spoolGearTeeth, motorGearTeeth, spoolRadii,
		motorTorque_Nm, positionMode, reply);
}

#if DUAL_CAN
GCodeResult HangprinterKinematics::SetODrive3TorqueModeInner(DriverId const driver, float const torque_Nm, const StringRef& reply) noexcept
{
	// Get a buffer
	CanMessageBuffer * buf = CanInterface::ODrive::PrepareSimpleMessage(driver, reply);
	if (buf == nullptr)
	{
		return GCodeResult::noCanBuffer;
	}

	// Set the right target torque
	buf->id = CanInterface::ODrive::ArbitrationId(driver, CANSimple::MSG_SET_INPUT_TORQUE);
	buf->dataLength = 4;
	buf->remote = false;
	memcpy(buf->msg.raw, &torque_Nm, sizeof(torque_Nm));
	CanInterface::SendPlainMessageNoFree(buf);

	// Enable Torque Control Mode
	buf->id = CanInterface::ODrive::ArbitrationId(driver, CANSimple::MSG_SET_CONTROLLER_MODES);
	buf->dataLength = 8;
	buf->remote = false;
	buf->msg.raw32[0] = CANSimple::CONTROL_MODE_TORQUE_CONTROL;
	buf->msg.raw32[1] = CANSimple::INPUT_MODE_PASSTHROUGH;
	CanInterface::SendPlainMessageNoFree(buf);

	CanMessageBuffer::Free(buf);
	return GCodeResult::ok;
}
#endif // DUAL_CAN

#if DUAL_CAN
GCodeResult HangprinterKinematics::SetODrive3PosMode(DriverId const driver, const StringRef& reply) noexcept
{
	HangprinterKinematics::ODriveAnswer const estimate = GetODrive3EncoderEstimate(driver, false, reply, false);
	if (estimate.valid)
	{
		float const desiredPos = estimate.value;
		CanMessageBuffer* buf = CanInterface::ODrive::PrepareSimpleMessage(driver, reply);
		if (buf == nullptr)
		{
			return GCodeResult::noCanBuffer;
		}
		buf->id = CanInterface::ODrive::ArbitrationId(driver, CANSimple::MSG_SET_INPUT_POS);
		buf->dataLength = 8;
		buf->remote = false;
		memset(buf->msg.raw32, 0, buf->dataLength); // four last bytes are velocity and torque setpoints. Zero them.
		memcpy(buf->msg.raw32, &desiredPos, sizeof(desiredPos));
		CanInterface::SendPlainMessageNoFree(buf);

		// Enable Position Control Mode
		buf->id = CanInterface::ODrive::ArbitrationId(driver, CANSimple::MSG_SET_CONTROLLER_MODES);
		buf->dataLength = 8;
		buf->remote = false;
		buf->msg.raw32[0] = CANSimple::CONTROL_MODE_POSITION_CONTROL;
		buf->msg.raw32[1] = CANSimple::INPUT_MODE_PASSTHROUGH;
		CanInterface::SendPlainMessageNoFree(buf);

		CanMessageBuffer::Free(buf);
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}
#endif // DUAL_CAN

#if DUAL_CAN
GCodeResult HangprinterKinematics::SetODrive3TorqueMode(DriverId const driver, float force_Newton, const StringRef& reply,
																												uint32_t setMechanicalAdvantage[], uint32_t setSpoolGearTeeth[],
																												uint32_t setMotorGearTeeth[], float setSpoolRadii[]) noexcept
{
	static uint32_t mechanicalAdvantage_[HANGPRINTER_MAX_ANCHORS] = { 0 };
	static uint32_t spoolGearTeeth_[HANGPRINTER_MAX_ANCHORS] = { 0 };
	static uint32_t motorGearTeeth_[HANGPRINTER_MAX_ANCHORS] = { 0 };
	static float spoolRadii_[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	if (setMechanicalAdvantage != nullptr && setSpoolGearTeeth != nullptr &&
			setMotorGearTeeth != nullptr && setSpoolRadii != nullptr) {
		for(size_t i{0}; i < numAnchors; ++i) {
			mechanicalAdvantage_[i] = setMechanicalAdvantage[i];
			spoolGearTeeth_[i] = setSpoolGearTeeth[i];
			motorGearTeeth_[i] = setMotorGearTeeth[i];
			spoolRadii_[i] = setSpoolRadii[i];
		}
		return GCodeResult::ok;
	}

	float motorTorque_Nm = 0.0F;
	bool positionMode = false;
	GCodeResult res = ComputeODrive3TorqueFromForceInternal(
		driver, force_Newton,
		mechanicalAdvantage_, spoolGearTeeth_, motorGearTeeth_, spoolRadii_,
		motorTorque_Nm, positionMode, reply);
	if (res != GCodeResult::ok)
	{
		return res;
	}

	if (positionMode)
	{
		res = SetODrive3PosMode(driver, reply);
		if (res == GCodeResult::ok)
		{
			reply.cat("pos_mode, ");
		}
		return res;
	}

	res = SetODrive3TorqueModeInner(driver, motorTorque_Nm, reply);
	if (res == GCodeResult::ok)
	{
		reply.catf("%.6f Nm, ", (double)motorTorque_Nm);
	}
	return res;
}
#endif // DUAL_CAN


float HangprinterKinematics::SpringK(float const springLength) const noexcept {
	return springKPerUnitLength / springLength;
}


void HangprinterKinematics::StaticForces(float const machinePos[3], float F[HANGPRINTER_MAX_ANCHORS]) const noexcept {
	StaticForcesConfig cfg;
	cfg.ignoreGravity = ignoreGravity;
	cfg.ignorePretension = ignorePretension;
	cfg.massKg = moverWeight_kg;
	cfg.lambda = 0.001f;
	cfg.tol = 1e-3f;
	cfg.stepDamp = 0.75f;
	cfg.maxItersTarget = 100;
	cfg.Tmax = const_cast<float *>(maxForce_Newton);
	cfg.Tmin = const_cast<float *>(minForce_Newton);

	StaticForcesResult result;
	result.tensions = F;

	if (flexAlgorithm == FlexAlgorithm::Tikhonov) {
		StaticForcesTikhonov(machinePos, cfg, result);
	} else {
		StaticForcesQp(machinePos, cfg, result);
	}
}

static inline void unit_or_zero(const float v[3], float ret[3]) {
	const float n = norm(v);
	if (n > 0.0F) {
		const float inv = 1.0F / n;
		ret[0] = v[0]*inv;
		ret[1] = v[1]*inv;
		ret[2] = v[2]*inv;
	} else {
		ret[0] = 0.0F;
		ret[1] = 0.0F;
		ret[2] = 0.0F;
	}
}

static inline void build_direction_matrix(const float mover[3], const float anchors[HANGPRINTER_MAX_ANCHORS][3], int N, float *A) {
	for (int j = 0; j < N; ++j) {
		float diff[3] = { 0.0 };
		diff[0] = anchors[j][0] - mover[0];
		diff[1] = anchors[j][1] - mover[1];
		diff[2] = anchors[j][2] - mover[2];
		float unit[3] = { 0.0 };
		unit_or_zero(diff, unit);
		A[0 * N + j] = unit[0];
		A[1 * N + j] = unit[1];
		A[2 * N + j] = unit[2];
	}
}

static inline bool invert3x3(const float M[3][3], float Minv[3][3], float eps = 1e-9f) {
	const float a = M[0][0], b = M[0][1], c = M[0][2];
	const float d = M[1][0], e = M[1][1], f = M[1][2];
	const float g = M[2][0], h = M[2][1], i = M[2][2];

	const float A = (e * i - f * h);
	const float B = -(d * i - f * g);
	const float C = (d * h - e * g);
	const float D = -(b * i - c * h);
	const float E = (a * i - c * g);
	const float F = -(a * h - b * g);
	const float G = (b * f - c * e);
	const float H = -(a * f - c * d);
	const float I = (a * e - b * d);

	const float det = a * A + b * B + c * C;
	if (std::fabs(det) < eps) {
		return false;
	}
	const float invdet = 1.0f / det;

	Minv[0][0] = A * invdet;
	Minv[0][1] = D * invdet;
	Minv[0][2] = G * invdet;
	Minv[1][0] = B * invdet;
	Minv[1][1] = E * invdet;
	Minv[1][2] = H * invdet;
	Minv[2][0] = C * invdet;
	Minv[2][1] = F * invdet;
	Minv[2][2] = I * invdet;
	return true;
}

static inline void solve_min_norm_T(const float *A, int N, const float Fext[3], float lambda, float *T) {
	float S[3][3] = {{lambda, 0.0f, 0.0f}, {0.0f, lambda, 0.0f}, {0.0f, 0.0f, lambda}};
	for (int j = 0; j < N; ++j) {
		const float ax = A[0 * N + j], ay = A[1 * N + j], az = A[2 * N + j];
		S[0][0] += ax * ax;
		S[0][1] += ax * ay;
		S[0][2] += ax * az;
		S[1][0] += ay * ax;
		S[1][1] += ay * ay;
		S[1][2] += ay * az;
		S[2][0] += az * ax;
		S[2][1] += az * ay;
		S[2][2] += az * az;
	}

	float Sinv[3][3] = { 0.0f };
	if (!invert3x3(S, Sinv)) {
		S[0][0] += 1e-6f;
		S[1][1] += 1e-6f;
		S[2][2] += 1e-6f;
		invert3x3(S, Sinv);
	}
	const float y0 = Sinv[0][0] * Fext[0] + Sinv[0][1] * Fext[1] + Sinv[0][2] * Fext[2];
	const float y1 = Sinv[1][0] * Fext[0] + Sinv[1][1] * Fext[1] + Sinv[1][2] * Fext[2];
	const float y2 = Sinv[2][0] * Fext[0] + Sinv[2][1] * Fext[1] + Sinv[2][2] * Fext[2];

	for (int j = 0; j < N; ++j) {
		const float ax = A[0 * N + j], ay = A[1 * N + j], az = A[2 * N + j];
		T[j] = ax * y0 + ay * y1 + az * y2;
	}
}

static inline void build_null_projector(const float *A, int N, float lambda, float *P) {
	float S[3][3] = {{lambda, 0.0f, 0.0f}, {0.0f, lambda, 0.0f}, {0.0f, 0.0f, lambda}};
	for (int j = 0; j < N; ++j) {
		const float ax = A[0 * N + j], ay = A[1 * N + j], az = A[2 * N + j];
		S[0][0] += ax * ax;
		S[0][1] += ax * ay;
		S[0][2] += ax * az;
		S[1][0] += ay * ax;
		S[1][1] += ay * ay;
		S[1][2] += ay * az;
		S[2][0] += az * ax;
		S[2][1] += az * ay;
		S[2][2] += az * az;
	}

	float Sinv[3][3] = { 0.0f };
	if (!invert3x3(S, Sinv)) {
		S[0][0] += 1e-6f;
		S[1][1] += 1e-6f;
		S[2][2] += 1e-6f;
		invert3x3(S, Sinv);
	}

	for (int r = 0; r < N; ++r) {
		for (int c = 0; c < N; ++c) {
			const float ax = A[0 * N + c], ay = A[1 * N + c], az = A[2 * N + c];
			const float B0 = Sinv[0][0] * ax + Sinv[0][1] * ay + Sinv[0][2] * az;
			const float B1 = Sinv[1][0] * ax + Sinv[1][1] * ay + Sinv[1][2] * az;
			const float B2 = Sinv[2][0] * ax + Sinv[2][1] * ay + Sinv[2][2] * az;
			const float arx = A[0 * N + r], ary = A[1 * N + r], arz = A[2 * N + r];
			const float Mrc = arx * B0 + ary * B1 + arz * B2;
			P[r * N + c] = (r == c ? 1.0f : 0.0f) - Mrc;
		}
	}
}

static inline void proj_nullspace(const float *P, int N, const float *v, float *out) {
	for (int r = 0; r < N; ++r) {
		float acc = 0.0f;
		for (int c = 0; c < N; ++c) {
			acc += P[r * N + c] * v[c];
		}
		out[r] = acc;
	}
}

static inline void applyA(const float* A, int N, const float* T, float res[3])
{
	float fx = 0.0F;
	float fy = 0.0F;
	float fz = 0.0F;
	for (int j = 0; j < N; ++j){
		fx += A[0*N + j]*T[j];
		fy += A[1*N + j]*T[j];
		fz += A[2*N + j]*T[j];
	}
	res[0] = fx;
	res[1] = fy;
	res[2] = fz;
}

static inline bool chol_decompose(float *G, int k) {
	const float eps = 1e-7;
	for (int i = 0; i < k; ++i) {
		for (int j = 0; j <= i; ++j) {
			float s = G[i * k + j];
			for (int p = 0; p < j; ++p) {
				s -= G[i * k + p] * G[j * k + p];
			}
			if (i == j) {
				if (s <= eps) {
					s = eps;
				}
				G[i * k + j] = std::sqrt(s);
			} else {
				G[i * k + j] = s / G[j * k + j];
			}
		}
		for (int j = i + 1; j < k; ++j) {
			G[i * k + j] = 0.0;
		}
	}
	return true;
}

static inline void chol_solve(const float *L, int k, const float *b, float *x) {
	float y[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	for (int i = 0; i < k; ++i) {
		float s = b[i];
		for (int p = 0; p < i; ++p) {
			s -= L[i * k + p] * y[p];
		}
		y[i] = s / L[i * k + i];
	}
	std::fill_n(x, k, 0.0);
	for (int i = k - 1; i >= 0; --i) {
		float s = y[i];
		for (int p = i + 1; p < k; ++p) {
			s -= L[p * k + i] * x[p];
		}
		x[i] = s / L[i * k + i];
	}
}

static inline void solve_box_ridge_ls(const float *A, int N, const float F[3], float lambda, const float *L, const float *U, int max_iters, float tol, float *T_out) {

	float H[HANGPRINTER_MAX_ANCHORS * HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	float f[HANGPRINTER_MAX_ANCHORS] = { 0.0 };

	for (int i = 0; i < N; ++i) {
		const float aix = A[0 * N + i], aiy = A[1 * N + i], aiz = A[2 * N + i];
		f[i] = aix * F[0] + aiy * F[1] + aiz * F[2];
		for (int j = 0; j <= i; ++j) {
			const float ajx = A[0 * N + j], ajy = A[1 * N + j], ajz = A[2 * N + j];
			const float dot = aix * ajx + aiy * ajy + aiz * ajz;
			const float v = dot + (i == j ? lambda : 0.0);
			H[i * N + j] = v;
			H[j * N + i] = v;
		}
	}

	float Lfull[HANGPRINTER_MAX_ANCHORS * HANGPRINTER_MAX_ANCHORS];
	std::size_t count = static_cast<std::size_t>(N) * N;
	std::copy_n(H, count, Lfull);
	chol_decompose(Lfull, N);
	float t[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	chol_solve(Lfull, N, f, t);
	for (int i = 0; i < N; ++i) {
		float li = L ? L[i] : 0.0;
		float ui = U ? U[i] : std::numeric_limits<float>::infinity();
		if (ui < li) {
			ui = li;
		}
		t[i] = min(max(t[i], li), ui);
	}

	int free_idx[HANGPRINTER_MAX_ANCHORS];
	int free_idx_count = 0;
	float g[HANGPRINTER_MAX_ANCHORS];

	auto projected_grad_norm = [&](const float *x) {
		float s2 = 0.0;
		for (int i = 0; i < N; ++i) {
			float li = L ? L[i] : 0.0;
			float ui = U ? U[i] : std::numeric_limits<float>::infinity();
			if (ui < li) {
				ui = li;
			}
			float gi = 0.0;
			for (int j = 0; j < N; ++j) {
				gi += H[i * N + j] * x[j];
			}
			gi -= f[i];
			const bool atL = (x[i] <= li + 1e-5);
			const bool atU = (x[i] >= ui - 1e-5);
			float pgi = gi;
			if (atL && gi > 0) {
				pgi = 0.0;
			}
			if (atU && gi < 0) {
				pgi = 0.0;
			}
			s2 += pgi * pgi;
		}
		return std::sqrt(s2);
	};

	for (int it = 0; it < max_iters; ++it) {
		for (int i = 0; i < N; ++i) {
			float gi = 0.0;
			for (int j = 0; j < N; ++j) {
				gi += H[i * N + j] * t[j];
			}
			g[i] = gi - f[i];
		}
		free_idx_count = 0;
		for (int i = 0; i < N; ++i) {
			float li = L ? L[i] : 0.0;
			float ui = U ? U[i] : std::numeric_limits<float>::infinity();
			if (ui < li) {
				ui = li;
			}
			const bool atL = (t[i] <= li + 1e-5);
			const bool atU = (t[i] >= ui - 1e-5);
			const bool violateL = atL && (g[i] < -tol);
			const bool violateU = atU && (g[i] > tol);
			if ((!atL && !atU) || violateL || violateU) {
				free_idx[free_idx_count++] = i;
			}
		}
		if (projected_grad_norm(t) <= tol) {
			break;
		}
		if (free_idx_count == 0) {
			int best = 0;
			float bestViol = 0.0;
			for (int i = 0; i < N; ++i) {
				float li = L ? L[i] : 0.0;
				float ui = U ? U[i] : std::numeric_limits<float>::infinity();
				if (ui < li) {
					ui = li;
				}
				const bool atL = (t[i] <= li + 1e-5);
				const bool atU = (t[i] >= ui - 1e-5);
				float viol = 0.0;
				if (atL) {
					viol = max((float)0.0, -g[i]);
				}
				if (atU) {
					viol = max(viol, g[i]);
				}
				if (viol > bestViol) {
					bestViol = viol;
					best = i;
				}
			}
			free_idx[free_idx_count++] = best;
		}
		const int k = free_idx_count;
		float Hff[HANGPRINTER_MAX_ANCHORS * HANGPRINTER_MAX_ANCHORS] = { 0.0 };
		float gf[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
		float pf[HANGPRINTER_MAX_ANCHORS] = { 0.0 };

		for (int p = 0; p < k; ++p) {
			const int ip = free_idx[p];
			gf[p] = g[ip];
			for (int q = 0; q < k; ++q) {
				const int iq = free_idx[q];
				Hff[p * k + q] = H[ip * N + iq];
			}
		}
		chol_decompose(Hff, k);
		for (int i = 0; i < k; ++i) {
			gf[i] = -gf[i];
		}
		chol_solve(Hff, k, gf, pf);
		float alpha = 1.0;
		for (int idx = 0; idx < k; ++idx) {
			const int i = free_idx[idx];
			const float pi = pf[idx];
			if (std::abs(pi) < 1e-8) {
				continue;
			}
			float li = L ? L[i] : 0.0;
			float ui = U ? U[i] : std::numeric_limits<float>::infinity();
			if (ui < li) {
				ui = li;
			}
			if (pi > 0.0) {
				const float amax = (ui - t[i]) / pi;
				if (amax < alpha) {
					alpha = max((float)0.0, amax);
				}
			} else {
				const float amax = (li - t[i]) / pi;
				if (amax < alpha) {
					alpha = max((float)0.0, amax);
				}
			}
		}
		for (int idx = 0; idx < k; ++idx) {
			const int i = free_idx[idx];
			t[i] += alpha * pf[idx];
		}
		for (int i = 0; i < N; ++i) {
			float li = L ? L[i] : 0.0;
			float ui = U ? U[i] : std::numeric_limits<float>::infinity();
			if (ui < li) {
				ui = li;
			}
			if (t[i] < li) {
				t[i] = li;
			}
			if (t[i] > ui) {
				t[i] = ui;
			}
		}
	}

	for (int i = 0; i < N; ++i) {
		T_out[i] = t[i];
	}
}

void HangprinterKinematics::StaticForcesTikhonov(
	const float mover[3],
	const StaticForcesConfig &cfg,
	StaticForcesResult &out) const noexcept
{
	float *T = out.tensions;
	float A[3 * HANGPRINTER_MAX_ANCHORS] = {0.0f};
	build_direction_matrix(mover, anchors, numAnchors, A);

	out.requestedForce[0] = 0.0F;
	out.requestedForce[1] = 0.0F;
	out.requestedForce[2] = 0.0F;
	for (size_t i = 0; i < numAnchors; ++i) {
		T[i] = 0.0f;
	}

	if (!cfg.ignoreGravity) {
		out.requestedForce[2] = cfg.massKg * cfg.g;
		solve_min_norm_T(A, numAnchors, out.requestedForce, cfg.lambda, T);
	}

	if (!cfg.ignorePretension) {
		float P[HANGPRINTER_MAX_ANCHORS * HANGPRINTER_MAX_ANCHORS] = {0.0f};
		build_null_projector(A, numAnchors, cfg.lambda, P);
		for (int it = 0; it < cfg.maxItersTarget; ++it) {
			float gradient[HANGPRINTER_MAX_ANCHORS] = {0.0f};
			for (size_t i = 0; i < numAnchors; ++i) {
				float target_grad = 0.1f * (T[i] - (cfg.Tmin ? cfg.Tmin[i] : 0.0f));
				if (cfg.Tmax && T[i] > cfg.Tmax[i]) {
					target_grad += T[i] - cfg.Tmax[i];
				}
				if (cfg.Tmin && T[i] < cfg.Tmin[i]) {
					target_grad += T[i] - cfg.Tmin[i];
				}
				gradient[i] = target_grad;
			}
			float d[HANGPRINTER_MAX_ANCHORS] = {0.0f};
			proj_nullspace(P, numAnchors, gradient, d);
			float dn = 0.0f;
			for (size_t i = 0; i < numAnchors; ++i) {
				dn += d[i] * d[i];
			}
			if (dn < cfg.tol * cfg.tol) {
				break;
			}
			for (size_t i = 0; i < numAnchors; ++i) {
				T[i] -= cfg.stepDamp * d[i];
			}
		}
		for (size_t i = 0; i < numAnchors; ++i) {
			if (T[i] < 0.0f) {
				T[i] = 0.0f;
			}
			if (cfg.Tmax && T[i] > cfg.Tmax[i]) {
				T[i] = cfg.Tmax[i];
			}
		}
	}

	applyA(A, numAnchors, T, out.achievedForce);
	out.residual[0] = out.requestedForce[0] - out.achievedForce[0];
	out.residual[1] = out.requestedForce[1] - out.achievedForce[1];
	out.residual[2] = out.requestedForce[2] - out.achievedForce[2];

	out.supportedGravityFrac = 0.0f;
	if (!cfg.ignoreGravity && out.requestedForce[2] > 1e-9f) {
		out.supportedGravityFrac = out.achievedForce[2] / out.requestedForce[2];
	}
}

void HangprinterKinematics::StaticForcesQp(
	const float mover[3],
	const StaticForcesConfig &cfg,
	StaticForcesResult &out) const noexcept
{
	float *T = out.tensions;
	float A[3 * HANGPRINTER_MAX_ANCHORS] = {0.0f};
	build_direction_matrix(mover, anchors, numAnchors, A);

	out.requestedForce[0] = 0.0F;
	out.requestedForce[1] = 0.0F;
	out.requestedForce[2] = 0.0F;
	if (!cfg.ignoreGravity) {
		out.requestedForce[2] = cfg.massKg * cfg.g;
	}

	float L[HANGPRINTER_MAX_ANCHORS];
	float U[HANGPRINTER_MAX_ANCHORS];
	std::fill_n(L, numAnchors, 0.0);
	std::fill_n(U, numAnchors, std::numeric_limits<float>::infinity());
	for (size_t i = 0; i < numAnchors; ++i) {
		const float li = cfg.ignorePretension ? 0.0 : (cfg.Tmin ? cfg.Tmin[i] : 0.0);
		float ui = (cfg.Tmax ? cfg.Tmax[i] : std::numeric_limits<float>::infinity());
		if (ui < li) {
			ui = li;
		}
		L[i] = li;
		U[i] = ui;
	}

	solve_box_ridge_ls(A, numAnchors, out.requestedForce, cfg.lambda, L, U, cfg.maxItersTarget, cfg.tol, T);

	applyA(A, numAnchors, T, out.achievedForce);
	out.residual[0] = out.requestedForce[0] - out.achievedForce[0];
	out.residual[1] = out.requestedForce[1] - out.achievedForce[1];
	out.residual[2] = out.requestedForce[2] - out.achievedForce[2];
	out.supportedGravityFrac = 0.0f;
	if (!cfg.ignoreGravity && out.requestedForce[2] > 1e-9f) {
		out.supportedGravityFrac = out.achievedForce[2] / out.requestedForce[2];
	}
}

#endif // SUPPORT_HANGPRINTER

// End
