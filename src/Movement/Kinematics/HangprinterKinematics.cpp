/*
 * HangprinterKinematics.cpp
 *
 *  Created on: 24 Nov 2017
 *      Author: David
 */

#include "HangprinterKinematics.h"

#if SUPPORT_HANGPRINTER

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/Move.h>
#include <CAN/CanInterface.h>
#include <Math/Matrix.h>

#include <General/Portability.h>

constexpr float DefaultAnchors[5][3] = {{    0.0, -2000.0, -100.0},
                                        { 2000.0,  1000.0, -100.0},
                                        {-2000.0,  1000.0, -100.0},
                                        {    0.0,     0.0, 3000.0},
                                        {    0.0,     0.0,    0.0}};
constexpr float DefaultPrintRadius = 1500.0;


#if SUPPORT_OBJECT_MODEL

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

#endif

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
	constexpr float DefaultMoverWeight_kg = 0.0F;          // Zero disables flex compensation feature.
	constexpr float DefaultSpringKPerUnitLength = 20000.0F; // Garda 1.1 is somewhere in the range [20000, 100000]
	constexpr float DefaultMinPlannedForce_Newton[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	constexpr float DefaultMaxPlannedForce_Newton[HANGPRINTER_MAX_ANCHORS] = { 70.0F, 70.0F, 70.0F, 70.0F, 70.0F };
	constexpr float DefaultGuyWireLengths[HANGPRINTER_MAX_ANCHORS] = { -1.0F }; // If one of these are negative they will be calculated in Recalc() instead
	constexpr float DefaultTargetForce_Newton = 20.0F; // 20 chosen quite arbitrarily
	constexpr float DefaultTorqueConstants[HANGPRINTER_MAX_ANCHORS] = { 0.0F };

	ARRAY_INIT(anchors, DefaultAnchors);
	anchorMode = HangprinterAnchorMode::LastOnTop;
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
	ARRAY_INIT(minPlannedForce_Newton, DefaultMinPlannedForce_Newton);
	ARRAY_INIT(maxPlannedForce_Newton, DefaultMaxPlannedForce_Newton);
	ARRAY_INIT(guyWireLengths, DefaultGuyWireLengths);
	targetForce_Newton = DefaultTargetForce_Newton;
	ARRAY_INIT(torqueConstants, DefaultTorqueConstants);

	Recalc();
}

static inline float hyp3(float const a[3], float const b[3]) {
	return fastSqrtf(fsquare(a[2] - b[2]) + fsquare(a[1] - b[1]) + fsquare(a[0] - b[0]));
}

// Recalculate the derived parameters
void HangprinterKinematics::Recalc() noexcept
{
	printRadiusSquared = fsquare(printRadius);

	// This is the difference between a "line length" and a "line position"
	// "line length" == ("line position" + "line length in origin")
	for (size_t i = 0; i < numAnchors; ++i)
	{
		distancesOrigin[i] = fastSqrtf(fsquare(anchors[i][0]) + fsquare(anchors[i][1]) + fsquare(anchors[i][2]));
	}

	//// Line buildup compensation
	float stepsPerUnitTimesRTmp[HANGPRINTER_MAX_ANCHORS] = { 0.0 };
	Platform& platform = reprap.GetPlatform(); // No const because we want to set drive steper per unit
	for (size_t i = 0; i < numAnchors; ++i)
	{
		const uint8_t driver = platform.GetAxisDriversConfig(i).driverNumbers[0].localDriver; // Only supports single driver
		bool dummy;
		stepsPerUnitTimesRTmp[i] =
			(
				(float)(mechanicalAdvantage[i])
				* fullStepsPerMotorRev[i]
				* platform.GetMicrostepping(driver, dummy)
				* spoolGearTeeth[i]
			)
			/ (2.0 * Pi * motorGearTeeth[i]);

		k2[i] = -(float)(mechanicalAdvantage[i] * linesPerSpool[i]) * spoolBuildupFactor;
		k0[i] = 2.0 * stepsPerUnitTimesRTmp[i] / k2[i];
		spoolRadiiSq[i] = spoolRadii[i] * spoolRadii[i];

		// Calculate the steps per unit that is correct at the origin
		platform.SetDriveStepsPerUnit(i, stepsPerUnitTimesRTmp[i] / spoolRadii[i], 0);
	}

	//// Flex compensation
	bool oneGuyWireNegative = false;
	for (size_t i{0}; i < numAnchors; ++i) {
		if (guyWireLengths[i] < 0.0F) {
			oneGuyWireNegative = true;
			break;
		}
	}

	if (anchorMode == HangprinterAnchorMode::LastOnTop) {
		// If no guy wire lengths are configured, assume a default configuration
		// with all spools stationary located at the last anchor,
		// and that the last anchor is a top anchor
		if (oneGuyWireNegative) {
			for (size_t i{0}; i < numAnchors - 1; ++i) {
				guyWireLengths[i] = hyp3(anchors[i], anchors[numAnchors - 1]);
			}
			guyWireLengths[numAnchors - 1] = 0.0F;
		}
	} else {
		// Assumes no guyWires in all other cases
		for (size_t i{0}; i < numAnchors; ++i) {
			guyWireLengths[i] = 0.0F;
		}
	}

	for (size_t i{0}; i < numAnchors; ++i) {
		springKsOrigin[i] = SpringK(distancesOrigin[i] * mechanicalAdvantage[i] + guyWireLengths[i]);
	}
	float constexpr origin[3] = { 0.0F, 0.0F, 0.0F };
	StaticForces(origin, fOrigin);
	for (size_t i{0}; i < numAnchors; ++i) {
		relaxedSpringLengthsOrigin[i] = distancesOrigin[i] - fOrigin[i] / (springKsOrigin[i] * mechanicalAdvantage[i]);
	}

#if DUAL_CAN
	// Setting and reading of forces.
	ReadODrive3AxisForce({}, StringRef(nullptr, 0), torqueConstants, mechanicalAdvantage, spoolGearTeeth, motorGearTeeth, spoolRadii);
	SetODrive3TorqueMode({}, 0.0F, StringRef(nullptr, 0), mechanicalAdvantage, spoolGearTeeth, motorGearTeeth, spoolRadii);
#endif
}

// Return the name of the current kinematics
const char *HangprinterKinematics::GetName(bool forStatusReport) const noexcept
{
	return "Hangprinter";
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters that affect the geometry. Set 'error' true if there was an error, otherwise leave it alone.
bool HangprinterKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) /*override*/
{
	bool seen = false;
	if (mCode == 669)
	{
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
		// 0=None, 1=last-top, 2=all-top, 3-half-top, etc
		uint32_t unsignedAnchorMode = (uint32_t)anchorMode;
		gb.TryGetUIValue('A', unsignedAnchorMode, seen);
		if (unsignedAnchorMode <= (uint32_t)HangprinterAnchorMode::AllOnTop) {
			anchorMode = (HangprinterAnchorMode)unsignedAnchorMode;
		}
		gb.TryGetFValue('Q', spoolBuildupFactor, seen);
		gb.TryGetFloatArray('R', numAnchors, spoolRadii, seen);
		gb.TryGetUIArray('U', numAnchors, mechanicalAdvantage, seen);
		gb.TryGetUIArray('O', numAnchors, linesPerSpool, seen);
		gb.TryGetUIArray('L', numAnchors, motorGearTeeth, seen);
		gb.TryGetUIArray('H', numAnchors, spoolGearTeeth, seen);
		gb.TryGetUIArray('J', numAnchors, fullStepsPerMotorRev, seen);
		gb.TryGetFValue('W', moverWeight_kg, seen);
		gb.TryGetFValue('S', springKPerUnitLength, seen);
		gb.TryGetFloatArray('I', numAnchors, minPlannedForce_Newton, seen);
		gb.TryGetFloatArray('X', numAnchors, maxPlannedForce_Newton, seen);
		gb.TryGetFloatArray('Y', numAnchors, guyWireLengths, seen);
		gb.TryGetFloatArray('C', numAnchors, torqueConstants, seen);
		if (seen)
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

			reply.lcatf("I%.1f", (double)minPlannedForce_Newton[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.1f", (double)minPlannedForce_Newton[i]);
			}

			reply.lcatf("X%.1f", (double)maxPlannedForce_Newton[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.1f", (double)maxPlannedForce_Newton[i]);
			}

			reply.lcatf("Y%.1f", (double)guyWireLengths[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.1f", (double)guyWireLengths[i]);
			}
			reply.lcatf("T%.1f\n", (double)targetForce_Newton);

			reply.lcatf("C%.4f", (double)torqueConstants[0]);
			for (size_t i = 1; i < numAnchors; ++i)
			{
				reply.catf(":%.4f", (double)torqueConstants[i]);
			}

		}
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
	return seen;
}

// Convert Cartesian coordinates to motor coordinates, returning true if successful
bool HangprinterKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
													size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	float distances[numAnchors];
	for (size_t i = 0; i < numAnchors; ++i) {
		distances[i] = hyp3(machinePos, anchors[i]);
	}

	float springKs[numAnchors];
	for (size_t i = 0; i < numAnchors; ++i) {
		springKs[i] = SpringK(distances[i] * mechanicalAdvantage[i] + guyWireLengths[i]);
	}

	float F[numAnchors] = { 0.0F }; // desired force in each direction
	StaticForces(machinePos, F);

	float relaxedSpringLengths[numAnchors];
	for (size_t i{0}; i < numAnchors; ++i) {
		relaxedSpringLengths[i] = distances[i] - F[i] / (springKs[i] * mechanicalAdvantage[i]);
		// The second term there is the mover's movement in mm due to flex
	}

	float linePos[numAnchors];
	for (size_t i = 0; i < numAnchors; ++i) {
		linePos[i] = relaxedSpringLengths[i] - relaxedSpringLengthsOrigin[i];
	}

	for (size_t i = 0; i < numAnchors; ++i)
	{
		motorPos[i] = lrintf(k0[i] * (fastSqrtf(spoolRadiiSq[i] + linePos[i] * k2[i]) - spoolRadii[i]));
	}

	return true;
}


inline float HangprinterKinematics::MotorPosToLinePos(const int32_t motorPos, size_t axis) const noexcept
{
	return (fsquare(motorPos / k0[axis] + spoolRadii[axis]) - spoolRadiiSq[axis]) / k2[axis];
}


void HangprinterKinematics::flexDistances(float const machinePos[3], float const distances[HANGPRINTER_MAX_ANCHORS],
                                          float flex[HANGPRINTER_MAX_ANCHORS]) const noexcept {
	float springKs[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	for (size_t i = 0; i < numAnchors; ++i) {
		springKs[i] = SpringK(distances[i] * mechanicalAdvantage[i] + guyWireLengths[i]);
	}

	float F[HANGPRINTER_MAX_ANCHORS] = { 0.0F }; // desired force in each direction
	StaticForces(machinePos, F);

	float relaxedSpringLengths[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	for (size_t i = 0; i < numAnchors; ++i) {
		relaxedSpringLengths[i] = distances[i]- F[i] / (springKs[i] * mechanicalAdvantage[i]);
	};

	float linePos[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	for (size_t i = 0; i < numAnchors; ++i) {
		linePos[i] = relaxedSpringLengths[i] - relaxedSpringLengthsOrigin[i];
	};

	float distanceDifferences[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	for (size_t i = 0; i < numAnchors; ++i) {
		distanceDifferences[i] = distances[i] - distancesOrigin[i];
	};

	for (size_t i = 0; i < numAnchors; ++i) {
		flex[i] = linePos[i] - distanceDifferences[i];
	}
}

// Convert motor coordinates to machine coordinates.
// Assumes lines are tight and anchor location norms are followed
void HangprinterKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	float distances[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	for (size_t i = 0; i < numAnchors; ++i) {
		distances[i] = MotorPosToLinePos(motorPos[i], i) + distancesOrigin[i];
	};
	ForwardTransform(distances, machinePos);

	// Now we have an approximate machinePos
	// Let's correct for line flex
	float flex[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	flexDistances(machinePos, distances, flex);
	float adjustedDistances[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	for (size_t i = 0; i < numAnchors; ++i) {
		adjustedDistances[i] = distances[i] - flex[i];
	}
	ForwardTransform(adjustedDistances, machinePos);
}

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
			if (finalCoords[Z_AXIS] < reprap.GetPlatform().AxisMinimum(Z_AXIS))
			{
				finalCoords[Z_AXIS] = reprap.GetPlatform().AxisMinimum(Z_AXIS);
				limited = true;
			}
			else if (finalCoords[Z_AXIS] > reprap.GetPlatform().AxisMaximum(Z_AXIS))
			{
				finalCoords[Z_AXIS] = reprap.GetPlatform().AxisMaximum(Z_AXIS);
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

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool HangprinterKinematics::QueryTerminateHomingMove(size_t axis) const noexcept
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void HangprinterKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept
{
	// Hangprinter homing is not supported
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

	scratchString.printf("N%d", numAnchors);
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

	scratchString.printf(" I%.1f", (double)minPlannedForce_Newton[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.1f", (double)minPlannedForce_Newton[i]);
	}
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" X%.1f", (double)maxPlannedForce_Newton[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.1f", (double)maxPlannedForce_Newton[i]);
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

	scratchString.printf(" T%.1f", (double)targetForce_Newton);
	ok = f->Write(scratchString.c_str());
	if (!ok) return false;

	scratchString.printf(" C%.4f", (double)torqueConstants[0]);
	for (size_t i = 1; i < numAnchors; ++i)
	{
		scratchString.catf(":%.4f", (double)torqueConstants[i]);
	}
	ok = f->Write(scratchString.c_str());

	return ok;
}

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool HangprinterKinematics::WriteResumeSettings(FileStore *f) const noexcept
{
	return WriteCalibrationParameters(f);
}

#endif


void HangprinterKinematics::ForwardTransform(float const distances[HANGPRINTER_MAX_ANCHORS], float machinePos[3]) const noexcept {
	switch (anchorMode) {
		case HangprinterAnchorMode::LastOnTop:
			if (numAnchors == 4) {
				ForwardTransformTetrahedron(distances, machinePos);
				return;
			} else if (numAnchors == 5) {
				ForwardTransformQuadrilateralPyramid(distances, machinePos);
				return;
			}
			// Intentional fall-through to next case
			// if no forward transform
		case HangprinterAnchorMode::None:
		case HangprinterAnchorMode::AllOnTop:
		default:
			return;
	}
}

/**
 * Hangprinter forward kinematics tetrahedron case (three low anchors, one high)
 * Basic idea is to subtract squared line lengths to get linear equations,
 * and then to solve with variable substitution.
 *
 * If we assume (enforce by rotations) that anchor location norms are followed
 * Ax=0 Dx=0 Dy=0
 * then
 * we get a fairly clean derivation by
 * subtracting d*d from a*a, b*b, and c*c:
 *
 *  a*a - d*d = k1        +  k2*y +  k3*z     <---- a line  (I)
 *  b*b - d*d = k4 + k5*x +  k6*y +  k7*z     <---- a plane (II)
 *  c*c - d*d = k8 + k9*x + k10*y + k11*z     <---- a plane (III)
 *
 * Use (I) to reduce (II) and (III) into lines. Eliminate y, keep z.
 *
 *  (II):  b*b - d*d = k12 + k13*x + k14*z
 *  <=>            x = k0b + k1b*z,           <---- a line  (IV)
 *
 *  (III): c*c - d*d = k15 + k16*x + k17*z
 *  <=>            x = k0c + k1c*z,           <---- a line  (V)
 *
 * where k1, k2, ..., k17, k0b, k0c, k1b, and k1c are known constants.
 *
 * These two straight lines are not parallel, so they will cross in exactly one point.
 * Find z by setting (IV) = (V)
 * Find x by inserting z into (V)
 * Find y by inserting z into (I)
 *
 * Warning: truncation errors will typically be in the order of a few tens of microns.
 */
void HangprinterKinematics::ForwardTransformTetrahedron(float const distances[HANGPRINTER_MAX_ANCHORS], float machinePos[3]) const noexcept
{
	// Force the anchor location norms Ax=0, Dx=0, Dy=0
	// through a series of rotations.
	static constexpr size_t A_AXIS = 0;
	static constexpr size_t B_AXIS = 1;
	static constexpr size_t C_AXIS = 2;
	static constexpr size_t D_AXIS = 3;
	float const x_angle = atanf(anchors[D_AXIS][Y_AXIS]/anchors[D_AXIS][Z_AXIS]);
	float const rxt[3][3] = {{1, 0, 0}, {0, cosf(x_angle), sinf(x_angle)}, {0, -sinf(x_angle), cosf(x_angle)}};
	float anchors_tmp0[4][3] = { 0 };
	for (size_t row{0}; row < 4; ++row) {
		for (size_t col{0}; col < 3; ++col) {
			anchors_tmp0[row][col] = rxt[0][col]*anchors[row][0] + rxt[1][col]*anchors[row][1] + rxt[2][col]*anchors[row][2];
		}
	}
	float const y_angle = atanf(-anchors_tmp0[D_AXIS][X_AXIS]/anchors_tmp0[D_AXIS][Z_AXIS]);
	float const ryt[3][3] = {{cosf(y_angle), 0, -sinf(y_angle)}, {0, 1, 0}, {sinf(y_angle), 0, cosf(y_angle)}};
	float anchors_tmp1[4][3] = { 0 };
	for (size_t row{0}; row < 4; ++row) {
		for (size_t col{0}; col < 3; ++col) {
			anchors_tmp1[row][col] = ryt[0][col]*anchors_tmp0[row][0] + ryt[1][col]*anchors_tmp0[row][1] + ryt[2][col]*anchors_tmp0[row][2];
		}
	}
	float const z_angle = atanf(anchors_tmp1[A_AXIS][X_AXIS]/anchors_tmp1[A_AXIS][Y_AXIS]);
	float const rzt[3][3] = {{cosf(z_angle), sinf(z_angle), 0}, {-sinf(z_angle), cosf(z_angle), 0}, {0, 0, 1}};
	for (size_t row{0}; row < 4; ++row) {
		for (size_t col{0}; col < 3; ++col) {
			anchors_tmp0[row][col] = rzt[0][col]*anchors_tmp1[row][0] + rzt[1][col]*anchors_tmp1[row][1] + rzt[2][col]*anchors_tmp1[row][2];
		}
	}

	const float Asq = fsquare(distancesOrigin[A_AXIS]);
	const float Bsq = fsquare(distancesOrigin[B_AXIS]);
	const float Csq = fsquare(distancesOrigin[C_AXIS]);
	const float Dsq = fsquare(distancesOrigin[D_AXIS]);
	const float aa = fsquare(distances[A_AXIS]);
	const float dd = fsquare(distances[D_AXIS]);
	const float k0b = (-fsquare(distances[B_AXIS]) + Bsq - Dsq + dd) / (2.0 * anchors_tmp0[B_AXIS][X_AXIS]) + (anchors_tmp0[B_AXIS][Y_AXIS] / (2.0 * anchors_tmp0[A_AXIS][Y_AXIS] * anchors_tmp0[B_AXIS][X_AXIS])) * (Dsq - Asq + aa - dd);
	const float k0c = (-fsquare(distances[C_AXIS]) + Csq - Dsq + dd) / (2.0 * anchors_tmp0[C_AXIS][X_AXIS]) + (anchors_tmp0[C_AXIS][Y_AXIS] / (2.0 * anchors_tmp0[A_AXIS][Y_AXIS] * anchors_tmp0[C_AXIS][X_AXIS])) * (Dsq - Asq + aa - dd);
	const float k1b = (anchors_tmp0[B_AXIS][Y_AXIS] * (anchors_tmp0[A_AXIS][Z_AXIS] - anchors_tmp0[D_AXIS][Z_AXIS])) / (anchors_tmp0[A_AXIS][Y_AXIS] * anchors_tmp0[B_AXIS][X_AXIS]) + (anchors_tmp0[D_AXIS][Z_AXIS] - anchors_tmp0[B_AXIS][Z_AXIS]) / anchors_tmp0[B_AXIS][X_AXIS];
	const float k1c = (anchors_tmp0[C_AXIS][Y_AXIS] * (anchors_tmp0[A_AXIS][Z_AXIS] - anchors_tmp0[D_AXIS][Z_AXIS])) / (anchors_tmp0[A_AXIS][Y_AXIS] * anchors_tmp0[C_AXIS][X_AXIS]) + (anchors_tmp0[D_AXIS][Z_AXIS] - anchors_tmp0[C_AXIS][Z_AXIS]) / anchors_tmp0[C_AXIS][X_AXIS];

	float machinePos_tmp0[3];
	machinePos_tmp0[Z_AXIS] = (k0b - k0c) / (k1c - k1b);
	machinePos_tmp0[X_AXIS] = k0c + k1c * machinePos_tmp0[Z_AXIS];
	machinePos_tmp0[Y_AXIS] = (Asq - Dsq - aa + dd) / (2.0 * anchors_tmp0[A_AXIS][Y_AXIS]) + ((anchors_tmp0[D_AXIS][Z_AXIS] - anchors_tmp0[A_AXIS][Z_AXIS]) / anchors_tmp0[A_AXIS][Y_AXIS]) * machinePos_tmp0[Z_AXIS];

	//// Rotate machinePos_tmp back to original coordinate system
	float machinePos_tmp1[3];
	for (size_t row{0}; row < 3; ++row) {
		machinePos_tmp1[row] = rzt[row][0]*machinePos_tmp0[0] + rzt[row][1]*machinePos_tmp0[1] + rzt[row][2]*machinePos_tmp0[2];
	}
	for (size_t row{0}; row < 3; ++row) {
		machinePos_tmp0[row] = ryt[row][0]*machinePos_tmp1[0] + ryt[row][1]*machinePos_tmp1[1] + ryt[row][2]*machinePos_tmp1[2];
	}
	for (size_t row{0}; row < 3; ++row) {
		machinePos[row] = rxt[row][0]*machinePos_tmp0[0] + rxt[row][1]*machinePos_tmp0[1] + rxt[row][2]*machinePos_tmp0[2];
	}
}

static inline bool det(FixedMatrix<float, 3, 4> M){
  return M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) + M(0, 1) * (M(1, 2) * M(2, 0) - M(2, 2) * M(1, 0)) + M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0));
}

static inline bool singular_3x3(FixedMatrix<float, 3, 4> M){
  float const threshold = 1e-1;
  return fabsf(det(M)) < threshold;
}


/* A quadrilateral pyramid has five anchors: four low and one high.
 * Our input variable `distances` contain the distance (L2-norm, euclidian distance) from each corner
 * to some point that is encapsulated by the five anchors.
 * We want to determine the xyz-coordinates of that point.
 *
 * Since this gives us 5 known variables and 3 unknown, this is an overdetermined system,
 * and we're not guaranteed that the 5 known variables define one exact point in space.
 * The `distance` values have been calculated from motors' rotational positions, and they don't
 * know if lines are slack or over tight, and much less which lines are slack and which are not in that case.
 * The `distance` values will generally be slightly off, and define a region in which we might wiggle, instead of a point.
 *
 * We approach this by solving four linear systems and averaging over the result,
 * which corresponds to all lines being ca equally slack.
 *
 * To get the four linear systems, we start with five non-linear ones.
 * |A - p| = l_a
 * |B - p| = l_b
 * |C - p| = l_c
 * |D - p| = l_d
 * |I - p| = l_i,
 *
 * where p is our unknown (x, y, z), A is the xyz of our A-anchor, and I is the top anchor.
 *
 * Square both sides and move |A|² to the right hand side:
 *
 * -2*A*p  + |p|² = l_a² - |A|²
 *
 * Now subtract the last equation to get rid of |p|² (we expect the last equality to always hold, vertical lines are never slack).
 *
 * -2*A*p  - 2*I*p = l_a² - l_i² - (|A|² - |I|²)
 *
 * Divide by -2 and simplify to get four linear equations
 *
 * (A - I)*p  = -(l_a² - l_i² - (|A|² - |I|²))/2 = k_a
 * (B - I)*p  = -(l_b² - l_i² - (|B|² - |I|²))/2 = k_b
 * (C - I)*p  = -(l_c² - l_i² - (|C|² - |I|²))/2 = k_c
 * (D - I)*p  = -(l_d² - l_i² - (|D|² - |I|²))/2 = k_d
 *
 * Say A' = A - I, and we get
 *
 * A'x A'y A'z | p_x   k_a
 * B'x B'y B'z | p_y = k_b
 * C'x C'y C'z | p_z   k_c
 * D'x D'y D'z |       k_d
 *
 * This is a linear but still overdetermined system (4x3). Skip each row in turn to obtain four different 3x3 matrices, and four different solution vectors k.
 * Solve each by GaussJordan elminiation and average over the result.
 * Voila.
 */
void HangprinterKinematics::ForwardTransformQuadrilateralPyramid(float const distances[HANGPRINTER_MAX_ANCHORS], float machinePos[3]) const noexcept {
	float anch_prim[4][3]{{0.0}};
	float distancesOriginSq[5]{0.0};
	float distancesSq[5]{0.0};
	float k[4]{0.0};
	FixedMatrix<float, 3, 4> M[4];
	float machinePos_tmp[3]{0.0};

	for (size_t i{0}; i < 4; ++i) {
		for (size_t j{0}; j < 3; ++j) {
			anch_prim[i][j] = anchors[i][j] - anchors[4][j];
		}
	}

	for (size_t i{0}; i < 5; ++i) {
		distancesOriginSq[i] = fsquare(distancesOrigin[i]);
		distancesSq[i] = fsquare(distances[i]);
	}

	for (size_t i{0}; i < 4; ++i) {
		k[i] = ((distancesOriginSq[i] - distancesOriginSq[4]) - (distancesSq[i] - distancesSq[4])) / 2.0;
	}

	for (size_t matrix_num{0}; matrix_num < 4; matrix_num++){
		for (size_t row{0}; row < 3; ++row) {
			size_t r = (row + matrix_num) % 4;
			for (size_t col{0}; col < 3; ++col) {
				M[matrix_num](row, col) = anch_prim[r][col];
			}
			M[matrix_num](row, 3) = k[r];
		}
	}

	// Solve the four systems
	size_t used{0};
	for (int k = 0; k < 4; ++k) {
		if (not singular_3x3(M[k])) {
			used++;
			M[k].GaussJordan(3, 4);
			for (size_t i{0}; i < 3; ++i) {
				machinePos_tmp[i] += M[k](i, 3);
			}
		}
	}
	if (used != 0) {
		for (size_t i{0}; i < 3; ++i) {
			machinePos[i] = machinePos_tmp[i]/static_cast<float>(used);
		}
	}

}


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
			reply.printf("Unexpected response length: %d", buf->dataLength);
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
			reply.printf("Max CAN addresses we can reference is %d. Can't reference board %d.", numAnchors, driver.boardAddress);
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
			reply.printf("Unexpected response length: %d", buf->dataLength);
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
		if (boardIndex < 0 or boardIndex > 3) {
			reply.catf("Board address not between 40 and 43: %d", driver.boardAddress);
			return GCodeResult::error;
		}
		// This force calculation if very rough, assuming perfect data from ODrive,
		// perfect transmission between motor gear and spool gear,
		// the exact same line buildup on spool as we have at the origin,
		// and no losses from any of the bearings or eyelets in the motion system.
		float motorTorque_Nm = motorCurrent.value * torqueConstants_[boardIndex];
		if (driver.boardAddress == 40 || driver.boardAddress == 41) // Driver direction is not stored on main board!! (will be in the future)
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
		if (driver.boardAddress == 42 || driver.boardAddress == 43) // Driver direction is not stored on main board!! (will be in the future)
		{
			directionCorrectedEncoderValue *= -1.0;
		}
		reply.catf("%.2f, ", (double)(directionCorrectedEncoderValue * 360.0));
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}
#endif // DUAL_CAN

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

	GCodeResult res = GCodeResult::ok;
	constexpr float MIN_TORQUE_N = 0.001;
	if (fabsf(force_Newton) < MIN_TORQUE_N)
	{
		res = SetODrive3PosMode(driver, reply);
		if (res == GCodeResult::ok)
		{
			reply.cat("pos_mode, ");
		}
	}
	else
	{
		size_t const boardIndex = driver.boardAddress - 40;
		if (boardIndex < 0 or boardIndex > 3) {
			reply.catf("Board address not between 40 and 43: %d", driver.boardAddress);
			return GCodeResult::error;
		}

		float const lineTension_N = force_Newton / mechanicalAdvantage_[boardIndex];
		float const spoolTorque_Nm = lineTension_N * spoolRadii_[boardIndex] * 0.001;
		float motorTorque_Nm = spoolTorque_Nm * motorGearTeeth_[boardIndex] / spoolGearTeeth_[boardIndex];
		// Set the right sign
		motorTorque_Nm = std::abs(motorTorque_Nm);
		if (driver.boardAddress == 40 || driver.boardAddress == 41) // Driver direction is not stored on main board!! (will be in the future)
		{
			motorTorque_Nm = -motorTorque_Nm;
		}
		res = SetODrive3TorqueModeInner(driver, motorTorque_Nm, reply);
		if (res == GCodeResult::ok)
		{
			reply.catf("%.6f Nm, ", (double)motorTorque_Nm);
		}
	}
	return res;
}
#endif // DUAL_CAN


float HangprinterKinematics::SpringK(float const springLength) const noexcept {
	return springKPerUnitLength / springLength;
}


void HangprinterKinematics::StaticForces(float const machinePos[3], float F[HANGPRINTER_MAX_ANCHORS]) const noexcept {
	switch (anchorMode) {
		case HangprinterAnchorMode::LastOnTop:
			if (numAnchors == 4) {
				StaticForcesTetrahedron(machinePos, F);
				return;
			} else if (numAnchors == 5) {
				StaticForcesQuadrilateralPyramid(machinePos, F);
				return;
			}
			// Intentional fall-through to next case
			// if no line flex compensation
		case HangprinterAnchorMode::None:
		case HangprinterAnchorMode::AllOnTop:
		default:
			for (size_t i = 0; i < HANGPRINTER_MAX_ANCHORS; ++i){
				F[i] = 0.0;
			}
	}
}

void HangprinterKinematics::StaticForcesQuadrilateralPyramid(float const machinePos[3], float F[HANGPRINTER_MAX_ANCHORS]) const noexcept {
	// A QuadrilateralPyramid has 5 corners, there's one anchor in each.
	// There are many 4's in this function because 4 motors (the lower ones, ABCD)
	// are assumed to have unknown forces.
	// The forces in the top anchor is assumed to be known and constant, except for gravity's
	// effects who are also known.
	if (moverWeight_kg < 0.0001) {
		return;
	}
	// Space for four linear 3x3 systems, each with two solution columns,
	FixedMatrix<float, 3, 5> M[4];

	float norm[5];
	norm[4] = hyp3(anchors[4], machinePos);
	for (int i = 0; i < 4; ++i) {
		norm[i] = hyp3(anchors[i], machinePos);
		for (int j = 0; j < 3; ++j) {
			for (int k = 0; k < 4; ++k) {
				// Fill 3x3 top left corner of system with
				// unit vectors toward each ABCD anchor from mover
				// If A is the column vector pointing towards A-anchor, we're building these
				// four matrices:
				// k=0: [BCD], A-direction skipped
				// k=1: [ACD], B-direction skipped
				// k=2: [ABD], C-direction skipped
				// k=3: [ABC], D-direction skipped
				if ( k != i) {
					if ( i > k ) {
						M[k](j, i - 1) = (anchors[i][j] - machinePos[j]) / norm[i];
					} else {
						M[k](j, i) = (anchors[i][j] - machinePos[j]) / norm[i];
					}
				}
			}
		}
	}
	float const mg = moverWeight_kg * 9.81;

	float top_mg = 0.0F;
	float top_pre = 0.0F;

	if (anchors[4][Z_AXIS] > machinePos[Z_AXIS]) {
		// These force constants will go into the solution column that has to do with gravity
		top_mg = mg / ((anchors[4][Z_AXIS] - machinePos[Z_AXIS]) / norm[4]);
		top_pre = targetForce_Newton;
	}

	// Indices for the two solution columns
	size_t const sol_mg = 3;
	size_t const sol_pt = 4;
	for (int i = 0; i < 3; ++i) {
		float const top_dist = (anchors[4][i] - machinePos[i]) / norm[4];
		for (int k = 0; k < 4; ++k) {
			M[k](i, sol_mg) = -top_mg * top_dist;  // gravity solution column
			M[k](i, sol_pt) = -top_pre * top_dist; // pretension solution column
		}
	}
	for (int k = 0; k < 4; ++k) {
		// Cancel out top anchor's Z-force with gravity.
		M[k](Z_AXIS, sol_mg) += mg; // == 0
	}

	// Solve the four systems
	for (int k = 0; k < 4; ++k) {
		M[k].GaussJordan(3, 5);
	}

	// Weigh/scale the pre-tension solutions so all have equal max force.
	float norm_ABCD[4];
	for(size_t k{0}; k < 4; ++k) {
		norm_ABCD[k] = fastSqrtf(M[k](0, sol_pt) * M[k](0, sol_pt) + M[k](1, sol_pt) * M[k](1, sol_pt) + M[k](2, sol_pt) * M[k](2, sol_pt));
	}

	// Arrays to hold our weighted combinations of the four (pairs of) solutions
	float p[4] = { 0.0F, 0.0F, 0.0F, 0.0F };
	float m[4] = { 0.0F, 0.0F, 0.0F, 0.0F };
	for (size_t i{0}; i < 3; ++i) {
		for (size_t j{0}; j < 4; ++j) {
			float const pt_weight = targetForce_Newton / norm_ABCD[j];
			// The gravity counter actions are scaled to exactly counter act gravity, and top-line forces neccesary to counter act gravity.
			// So the resultant force of all four solutions is the same. Lets add a quarter of each solution to get back that resultant force.
			float const mg_weight = 1.0/4.0;
			// i can mean BCD, ACD, ABD, or ABC, depending on which matrix we're looking into
			// Let's just translate that back into the solutions vectors
			size_t const s = j <= i ? i + 1 : i;
			p[s] += M[j](i, sol_pt)*pt_weight;
			m[s] += M[j](i, sol_mg)*mg_weight;
		}
	}

	// The pre-tension solution can be scaled up or down however we want.
	// Forces in those solution cancel each other out exactly, so any multiple of the solution is also a valid solution.
	//
	// (The gravity solution can't be scaled since it has to exactly counter act top-line forces that must exactly counter act gravity (mg))
	//
	// Use the scaling freedom of the pre-tension solution to assure that we have at least targetForce_Newton in the ABCD lines,
	// and that no line (incl top-line) get more tension than the configured maxPlannedForce in that direction.
	float  preFac = min(max(std::abs((targetForce_Newton - m[3]) / p[3]),
	                             max(std::abs((targetForce_Newton - m[2]) / p[2]),
	                                 max(std::abs((targetForce_Newton - m[1]) / p[1]), std::abs((targetForce_Newton - m[0]) / p[0])))),
	                         min(std::abs((maxPlannedForce_Newton[4] - top_mg) / top_pre),
	                             min(min(std::abs((maxPlannedForce_Newton[0] - m[0]) / p[0]), std::abs((maxPlannedForce_Newton[1] - m[1]) / p[1])),
	                                 min(std::abs((maxPlannedForce_Newton[2] - m[2]) / p[2]), std::abs((maxPlannedForce_Newton[3] - m[3]) / p[3])))));

	float tot[5] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
	tot[0] = m[0] + preFac * p[0];
	tot[1] = m[1] + preFac * p[1];
	tot[2] = m[2] + preFac * p[2];
	tot[3] = m[3] + preFac * p[3];
	tot[4] = top_mg + preFac * top_pre;

	for (size_t i{0}; i < 5; ++i) {
		// Negative, or very large forces can still have slipped through the preFac filter.
		// Truncate away such forces and assign to the output variable.
		// Voila.
		// The min( ... ) shouldn't be needed here. Just better safe than sorry.
		F[i] = min(max(tot[i], minPlannedForce_Newton[i]), maxPlannedForce_Newton[i]);
	}
}


void HangprinterKinematics::StaticForcesTetrahedron(float const machinePos[3], float F[HANGPRINTER_MAX_ANCHORS]) const noexcept {
	static constexpr size_t A_AXIS = 0;
	static constexpr size_t B_AXIS = 1;
	static constexpr size_t C_AXIS = 2;
	static constexpr size_t D_AXIS = 3;
	static constexpr size_t FOUR_ANCH = 4;
	static constexpr size_t CARTESIAN_AXES = 3;

	if (moverWeight_kg > 0.0001) { // mover weight more than one gram
		float norm[FOUR_ANCH]; // Unit vector directions toward each anchor from mover
		FixedMatrix<float, CARTESIAN_AXES, FOUR_ANCH + 1> M;
		for (size_t i = 0; i < FOUR_ANCH - 1; ++i) { // One anchor above mover
			norm[i] = hyp3(anchors[i], machinePos);
			for (size_t j = 0; j < CARTESIAN_AXES; ++j) {
				M(j, i) = (anchors[i][j] - machinePos[j]) / norm[i];
			}
		}

		float const mg = moverWeight_kg * 9.81; // Size of gravity force in Newtons
		float D_mg = 0.0F;
		float D_pre = 0.0F;

		// The D-forces' z-component is always equal to mg + targetForce_Newton.
		// The D-forces' z-component is always equal to mg +
		// targetForce_Newton. This means ABC-motors combined pull
		// downwards targetForce_Newton N. I don't know if that's always
		// solvable. Still, my tests show that we get very reasonable
		// flex compensation...

		// Right hand side of the equation
		// A + B + C + D + (0,0,-mg)' = 0
		// <=> A + B + C = -D + (0,0,mg)'
		//
		// Mx = y,
		//
		// Where M is the matrix
		//
		//     ax bx cx
		// M = ay by cy,
		//     az bz cz
		//
		// and x is the sizes of the forces:
		//
		//     A
		// x = B,
		//     C
		//
		// and y is
		//
		//     -D*dx
		// y = -D*dy     .
		//     -D*dz + mg

		float const normD = hyp3(anchors[D_AXIS], machinePos);
		if (anchors[D_AXIS][Z_AXIS] > machinePos[Z_AXIS]) { // D anchor above machine
			D_mg = mg / ((anchors[D_AXIS][2] - machinePos[2]) / normD);
			D_pre = targetForce_Newton;
		}

		for (size_t i = 0; i < CARTESIAN_AXES; ++i) {
			float const dist = (anchors[D_AXIS][i] - machinePos[i]) / normD;
			M(i, FOUR_ANCH - 1) = -D_mg * dist;
			M(i, FOUR_ANCH) = -D_pre * dist;
		}
		M(Z_AXIS, D_AXIS) += mg;

		// Solve!
		const bool ok = M.GaussJordan(CARTESIAN_AXES, 5);

		if (ok) {
			// Size of the undetermined forces
			float const A_mg = M(0, 3);
			float const B_mg = M(1, 3);
			float const C_mg = M(2, 3);
			float const A_pre = M(0, 4);
			float const B_pre = M(1, 4);
			float const C_pre = M(2, 4);

			// Assure at least targetForce in the ABC lines (first argument to outer min()),
			// and that no line get more than max planned force (second argument to outer min()).
			float const preFac = min(max(std::abs((targetForce_Newton - C_mg) / C_pre),
			                             max(std::abs((targetForce_Newton - B_mg) / B_pre), std::abs((targetForce_Newton - A_mg) / A_pre))),
			                         min(min(std::abs((maxPlannedForce_Newton[A_AXIS] - A_mg) / A_pre), std::abs((maxPlannedForce_Newton[B_AXIS] - B_mg) / B_pre)),
			                             min(std::abs((maxPlannedForce_Newton[C_AXIS] - C_mg) / C_pre), std::abs((maxPlannedForce_Newton[D_AXIS] - D_mg) / D_pre))));

			float totalForces[FOUR_ANCH] = {
				A_mg + preFac * A_pre,
				B_mg + preFac * B_pre,
				C_mg + preFac * C_pre,
				D_mg + preFac * D_pre
			};

			for (size_t i = 0; i < FOUR_ANCH; ++i) {
				F[i] = min(max(totalForces[i], minPlannedForce_Newton[i]), maxPlannedForce_Newton[i]);
			}
		}
	}
}

#endif // SUPPORT_HANGPRINTER

// End
