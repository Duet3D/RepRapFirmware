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

#include <General/Portability.h>

// Default anchor coordinates
// These are only placeholders. Each machine must have these values calibrated in order to work correctly.
constexpr float DefaultAnchors[4][3] = {{    0.0, -2000.0, -100.0},
                                        { 2000.0,  1000.0, -100.0},
                                        {-2000.0,  1000.0, -100.0},
                                        {    0.0,     0.0, 3000.0}};
constexpr float DefaultPrintRadius = 1500.0;

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(HangprinterKinematics, __VA_ARGS__)

constexpr ObjectModelArrayDescriptor HangprinterKinematics::anchorCoordinatesArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 3; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const HangprinterKinematics *)self)->anchors[context.GetIndex(1)][context.GetLastIndex()], 1); }
};

constexpr ObjectModelArrayDescriptor HangprinterKinematics::anchorsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return HANGPRINTER_AXES; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&anchorCoordinatesArrayDescriptor); }
};

constexpr ObjectModelTableEntry HangprinterKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "anchors",		OBJECT_MODEL_FUNC_NOSELF(&anchorsArrayDescriptor), 	ObjectModelEntryFlags::none },
	{ "name",			OBJECT_MODEL_FUNC(self->GetName(true)), 			ObjectModelEntryFlags::none },
	{ "printRadius",	OBJECT_MODEL_FUNC(self->printRadius, 1), 			ObjectModelEntryFlags::none },
};

constexpr uint8_t HangprinterKinematics::objectModelTableDescriptor[] = { 1, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(HangprinterKinematics, Kinematics)

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
	constexpr float DefaultSpoolRadii[4] = { 75.0, 75.0, 75.0, 75.0}; // HP4 default
	/* If axis runs lines back through pulley system, set mechanical advantage accordingly with M669 */
	constexpr uint32_t DefaultMechanicalAdvantage[4] = { 2, 2, 2, 4}; // HP4 default
	constexpr uint32_t DefaultLinesPerSpool[4] = { 1, 1, 1, 1}; // HP4 default
	constexpr uint32_t DefaultMotorGearTeeth[4] = {  20,  20,  20,  20}; // HP4 default
	constexpr uint32_t DefaultSpoolGearTeeth[4] = { 255, 255, 255, 255}; // HP4 default
	constexpr uint32_t DefaultFullStepsPerMotorRev[4] = { 25, 25, 25, 25};
	ARRAY_INIT(anchors, DefaultAnchors);
	printRadius = DefaultPrintRadius;
	spoolBuildupFactor = DefaultSpoolBuildupFactor;
	ARRAY_INIT(spoolRadii, DefaultSpoolRadii);
	ARRAY_INIT(mechanicalAdvantage, DefaultMechanicalAdvantage);
	ARRAY_INIT(linesPerSpool, DefaultLinesPerSpool);
	ARRAY_INIT(motorGearTeeth, DefaultMotorGearTeeth);
	ARRAY_INIT(spoolGearTeeth, DefaultSpoolGearTeeth);
	ARRAY_INIT(fullStepsPerMotorRev, DefaultFullStepsPerMotorRev);
	Recalc();
}

// Recalculate the derived parameters
void HangprinterKinematics::Recalc() noexcept
{
	printRadiusSquared = fsquare(printRadius);

	// This is the difference between a "line length" and a "line position"
	// "line length" == ("line position" + "line length in origin")
	lineLengthsOrigin[A_AXIS] = fastSqrtf(fsquare(anchors[A_AXIS][0]) + fsquare(anchors[A_AXIS][1]) + fsquare(anchors[A_AXIS][2]));
	lineLengthsOrigin[B_AXIS] = fastSqrtf(fsquare(anchors[B_AXIS][0]) + fsquare(anchors[B_AXIS][1]) + fsquare(anchors[B_AXIS][2]));
	lineLengthsOrigin[C_AXIS] = fastSqrtf(fsquare(anchors[C_AXIS][0]) + fsquare(anchors[C_AXIS][1]) + fsquare(anchors[C_AXIS][2]));
	lineLengthsOrigin[D_AXIS] = fastSqrtf(fsquare(anchors[D_AXIS][0]) + fsquare(anchors[D_AXIS][1]) + fsquare(anchors[D_AXIS][2]));


	// Line buildup compensation
	float stepsPerUnitTimesRTmp[HANGPRINTER_AXES] = { 0.0 };
	Platform& platform = reprap.GetPlatform(); // No const because we want to set drive steper per unit
	for (size_t i = 0; i < HANGPRINTER_AXES; i++)
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
		if (gb.TryGetFloatArray('A', 3, anchors[A_AXIS], reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetFloatArray('B', 3, anchors[B_AXIS], reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetFloatArray('C', 3, anchors[C_AXIS], reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetFloatArray('D', 3, anchors[D_AXIS], reply, seen))
		{
			error = true;
			return true;
		}

		if (gb.Seen('P'))
		{
			printRadius = gb.GetFValue();
			seen = true;
		}

		if (seen)
		{
			Recalc();
		}
		else if (!seenNonGeometry && !gb.Seen('K'))
		{
			Kinematics::Configure(mCode, gb, reply, error);
			reply.lcatf(
				"A:%.2f, %.2f, %.2f\n"
				"B:%.2f, %.2f, %.2f\n"
				"C:%.2f, %.2f, %.2f\n"
				"D:%.2f, %.2f, %.2f\n"
				"P:Print radius: %.1f",
				(double)anchors[A_AXIS][X_AXIS], (double)anchors[A_AXIS][Y_AXIS], (double)anchors[A_AXIS][Z_AXIS],
				(double)anchors[B_AXIS][X_AXIS], (double)anchors[B_AXIS][Y_AXIS], (double)anchors[B_AXIS][Z_AXIS],
				(double)anchors[C_AXIS][X_AXIS], (double)anchors[C_AXIS][Y_AXIS], (double)anchors[C_AXIS][Z_AXIS],
				(double)anchors[D_AXIS][X_AXIS], (double)anchors[D_AXIS][Y_AXIS], (double)anchors[D_AXIS][Z_AXIS],
				(double)printRadius
				);
		}
	}
	else if (mCode == 666)
	{
		gb.TryGetFValue('Q', spoolBuildupFactor, seen);
		if (gb.TryGetFloatArray('R', HANGPRINTER_AXES, spoolRadii, reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetUIArray('U', HANGPRINTER_AXES, mechanicalAdvantage, reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetUIArray('O', HANGPRINTER_AXES, linesPerSpool, reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetUIArray('L', HANGPRINTER_AXES, motorGearTeeth, reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetUIArray('H', HANGPRINTER_AXES, spoolGearTeeth, reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetUIArray('J', HANGPRINTER_AXES, fullStepsPerMotorRev, reply, seen))
		{
			error = true;
			return true;
		}
		if (seen)
		{
			Recalc();
		}
		else
		{
			reply.printf(
				"Q:Buildup fac %.4f\n"
				"R:Spool r %.2f, %.2f, %.2f, %.2f\n"
				"U:Mech Adv %d, %d, %d, %d\n"
				"O:Lines/spool %d, %d, %d, %d\n"
				"L:Motor gear teeth %d, %d, %d, %d\n"
				"H:Spool gear teeth %d, %d, %d, %d\n"
				"J:Full steps/rev %d, %d, %d, %d",
				(double)spoolBuildupFactor,
				(double)spoolRadii[A_AXIS], (double)spoolRadii[B_AXIS], (double)spoolRadii[C_AXIS], (double)spoolRadii[D_AXIS],
				(int)mechanicalAdvantage[A_AXIS], (int)mechanicalAdvantage[B_AXIS], (int)mechanicalAdvantage[C_AXIS], (int)mechanicalAdvantage[D_AXIS],
				(int)linesPerSpool[A_AXIS], (int)linesPerSpool[B_AXIS], (int)linesPerSpool[C_AXIS], (int)linesPerSpool[D_AXIS],
				(int)motorGearTeeth[A_AXIS], (int)motorGearTeeth[B_AXIS], (int)motorGearTeeth[C_AXIS], (int)motorGearTeeth[D_AXIS],
				(int)spoolGearTeeth[A_AXIS], (int)spoolGearTeeth[B_AXIS], (int)spoolGearTeeth[C_AXIS], (int)spoolGearTeeth[D_AXIS],
				(int)fullStepsPerMotorRev[A_AXIS], (int)fullStepsPerMotorRev[B_AXIS], (int)fullStepsPerMotorRev[C_AXIS], (int)fullStepsPerMotorRev[D_AXIS]
				);
		}
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
	return seen;
}

// Calculate the square of the line length from a spool from a Cartesian coordinate
inline float HangprinterKinematics::LineLengthSquared(const float machinePos[3], const float anchors[3]) const noexcept
{
	return fsquare(anchors[Z_AXIS] - machinePos[Z_AXIS]) + fsquare(anchors[Y_AXIS] - machinePos[Y_AXIS]) + fsquare(anchors[X_AXIS] - machinePos[X_AXIS]);
}

// Convert Cartesian coordinates to motor coordinates, returning true if successful
bool HangprinterKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
													size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	float squaredLineLengths[HANGPRINTER_AXES];
	squaredLineLengths[A_AXIS] = LineLengthSquared(machinePos, anchors[A_AXIS]);
	squaredLineLengths[B_AXIS] = LineLengthSquared(machinePos, anchors[B_AXIS]);
	squaredLineLengths[C_AXIS] = LineLengthSquared(machinePos, anchors[C_AXIS]);
	squaredLineLengths[D_AXIS] = LineLengthSquared(machinePos, anchors[D_AXIS]);

	float linePos[HANGPRINTER_AXES];
	for (size_t i = 0; i < HANGPRINTER_AXES; ++i)
	{
		linePos[i] = fastSqrtf(squaredLineLengths[i]) - lineLengthsOrigin[i];
	}

	motorPos[A_AXIS] = lrintf(k0[A_AXIS] * (fastSqrtf(spoolRadiiSq[A_AXIS] + linePos[A_AXIS] * k2[A_AXIS]) - spoolRadii[A_AXIS]));
	motorPos[B_AXIS] = lrintf(k0[B_AXIS] * (fastSqrtf(spoolRadiiSq[B_AXIS] + linePos[B_AXIS] * k2[B_AXIS]) - spoolRadii[B_AXIS]));
	motorPos[C_AXIS] = lrintf(k0[C_AXIS] * (fastSqrtf(spoolRadiiSq[C_AXIS] + linePos[C_AXIS] * k2[C_AXIS]) - spoolRadii[C_AXIS]));
	motorPos[D_AXIS] = lrintf(k0[D_AXIS] * (fastSqrtf(spoolRadiiSq[D_AXIS] + linePos[D_AXIS] * k2[D_AXIS]) - spoolRadii[D_AXIS]));

	return true;
}


inline float HangprinterKinematics::MotorPosToLinePos(const int32_t motorPos, size_t axis) const noexcept
{
	return (fsquare(motorPos / k0[axis] + spoolRadii[axis]) - spoolRadiiSq[axis]) / k2[axis];
}

// Convert motor coordinates to machine coordinates.
// Assumes lines are tight and anchor location norms are followed
void HangprinterKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	ForwardTransform(
		MotorPosToLinePos(motorPos[A_AXIS], A_AXIS) + lineLengthsOrigin[A_AXIS],
		MotorPosToLinePos(motorPos[B_AXIS], B_AXIS) + lineLengthsOrigin[B_AXIS],
		MotorPosToLinePos(motorPos[C_AXIS], C_AXIS) + lineLengthsOrigin[C_AXIS],
		MotorPosToLinePos(motorPos[D_AXIS], D_AXIS) + lineLengthsOrigin[D_AXIS],
		machinePos);
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

static bool isInsideTetrahedron(float const point[3], float const tetrahedron[4][3]){
	return isSameSide(tetrahedron[0], tetrahedron[1], tetrahedron[2], tetrahedron[3], point) &&
	       isSameSide(tetrahedron[2], tetrahedron[1], tetrahedron[3], tetrahedron[0], point) &&
	       isSameSide(tetrahedron[2], tetrahedron[3], tetrahedron[0], tetrahedron[1], point) &&
	       isSameSide(tetrahedron[0], tetrahedron[3], tetrahedron[1], tetrahedron[2], point);
}

bool HangprinterKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept /*override*/
{
	float const coords[3] = {axesCoords[X_AXIS], axesCoords[Y_AXIS], axesCoords[Z_AXIS]};
	return isInsideTetrahedron(coords, anchors);
}

// Limit the Cartesian position that the user wants to move to returning true if we adjusted the position
LimitPositionResult HangprinterKinematics::LimitPosition(float finalCoords[], const float * null initialCoords,
															size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
	bool limited = false;
	if ((axesToLimit & XyzAxes) == XyzAxes)
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
		g92Axes.SetBit(D_AXIS);
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
	bool ok = f->Write("; Hangprinter parameters\n");
	if (ok)
	{
		String<100> scratchString;
		scratchString.printf("M669 K6 A%.3f:%.3f:%.3f B%.3f:%.3f:%.3f",
							(double)anchors[A_AXIS][X_AXIS], (double)anchors[A_AXIS][Y_AXIS], (double)anchors[A_AXIS][Z_AXIS],
							(double)anchors[B_AXIS][X_AXIS], (double)anchors[B_AXIS][Y_AXIS], (double)anchors[B_AXIS][Z_AXIS]);
		ok = f->Write(scratchString.c_str());
		if (ok)
		{
			scratchString.printf(" C%.3f:%.3f:%.3f D%.3f:%.3f:%.3f P%.1f\n",
								(double)anchors[C_AXIS][X_AXIS], (double)anchors[C_AXIS][Y_AXIS], (double)anchors[C_AXIS][Z_AXIS],
								(double)anchors[D_AXIS][X_AXIS], (double)anchors[D_AXIS][Y_AXIS], (double)anchors[D_AXIS][Z_AXIS],
								(double)printRadius);
			ok = f->Write(scratchString.c_str());
			if (ok)
			{
				scratchString.printf("M666 Q%.6f R%.3f:%.3f:%.3f:%.3f U%d:%d:%d:%d",
									(double)spoolBuildupFactor, (double)spoolRadii[A_AXIS],
									(double)spoolRadii[B_AXIS], (double)spoolRadii[C_AXIS], (double)spoolRadii[D_AXIS],
									(int)mechanicalAdvantage[A_AXIS], (int)mechanicalAdvantage[B_AXIS],
									(int)mechanicalAdvantage[C_AXIS], (int)mechanicalAdvantage[D_AXIS]
						);
				ok = f->Write(scratchString.c_str());
				if (ok)
				{
					scratchString.printf(" O%d:%d:%d:%d L%d:%d:%d:%d H%d:%d:%d:%d J%d:%d:%d:%d\n",
										(int)linesPerSpool[A_AXIS], (int)linesPerSpool[B_AXIS],
										(int)linesPerSpool[C_AXIS], (int)linesPerSpool[D_AXIS],
										(int)motorGearTeeth[A_AXIS], (int)motorGearTeeth[B_AXIS],
										(int)motorGearTeeth[C_AXIS], (int)motorGearTeeth[D_AXIS],
										(int)spoolGearTeeth[A_AXIS], (int)spoolGearTeeth[B_AXIS],
										(int)spoolGearTeeth[C_AXIS], (int)spoolGearTeeth[D_AXIS],
										(int)fullStepsPerMotorRev[A_AXIS], (int)fullStepsPerMotorRev[B_AXIS],
										(int)fullStepsPerMotorRev[C_AXIS], (int)fullStepsPerMotorRev[D_AXIS]
							);
					ok = f->Write(scratchString.c_str());
				}
			}
		}
	}
	return ok;
}

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool HangprinterKinematics::WriteResumeSettings(FileStore *f) const noexcept
{
	return WriteCalibrationParameters(f);
}

#endif

/**
 * Hangprinter forward kinematics
 * Basic idea is to subtract squared line lengths to get linear equations,
 * and then to solve with variable substitution.
 *
 * If we assume anchor location norms are followed
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
void HangprinterKinematics::ForwardTransform(float const a, float const b, float const c, float const d, float machinePos[3]) const noexcept
{
	// Force the anchor location norms Ax=0, Dx=0, Dy=0
	// through a series of rotations.
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

	const float Asq = fsquare(lineLengthsOrigin[A_AXIS]);
	const float Bsq = fsquare(lineLengthsOrigin[B_AXIS]);
	const float Csq = fsquare(lineLengthsOrigin[C_AXIS]);
	const float Dsq = fsquare(lineLengthsOrigin[D_AXIS]);
	const float aa = fsquare(a);
	const float dd = fsquare(d);
	const float k0b = (-fsquare(b) + Bsq - Dsq + dd) / (2.0 * anchors_tmp0[B_AXIS][X_AXIS]) + (anchors_tmp0[B_AXIS][Y_AXIS] / (2.0 * anchors_tmp0[A_AXIS][Y_AXIS] * anchors_tmp0[B_AXIS][X_AXIS])) * (Dsq - Asq + aa - dd);
	const float k0c = (-fsquare(c) + Csq - Dsq + dd) / (2.0 * anchors_tmp0[C_AXIS][X_AXIS]) + (anchors_tmp0[C_AXIS][Y_AXIS] / (2.0 * anchors_tmp0[A_AXIS][Y_AXIS] * anchors_tmp0[C_AXIS][X_AXIS])) * (Dsq - Asq + aa - dd);
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

// Print all the parameters for debugging
void HangprinterKinematics::PrintParameters(const StringRef& reply) const noexcept
{
	reply.printf("Anchor coordinates (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)\n",
					(double)anchors[A_AXIS][X_AXIS], (double)anchors[A_AXIS][Y_AXIS], (double)anchors[A_AXIS][Z_AXIS],
					(double)anchors[B_AXIS][X_AXIS], (double)anchors[B_AXIS][Y_AXIS], (double)anchors[B_AXIS][Z_AXIS],
					(double)anchors[C_AXIS][X_AXIS], (double)anchors[C_AXIS][Y_AXIS], (double)anchors[C_AXIS][Z_AXIS]);
}

#if DUAL_CAN
HangprinterKinematics::ODriveAnswer HangprinterKinematics::GetODrive3EncoderEstimate(DriverId const driver, bool const makeReference, const StringRef& reply, bool const subtractReference) THROWS(GCodeException)
{
	const uint8_t cmd = CANSimple::MSG_GET_ENCODER_ESTIMATES;
	static CanAddress seenDrives[HANGPRINTER_AXES] = { 0, 0, 0, 0 };
	static float referencePositions[HANGPRINTER_AXES] = { 0.0, 0.0, 0.0, 0.0 };
	static size_t numSeenDrives = 0;
	size_t thisDriveIdx = 0;

	while (thisDriveIdx < numSeenDrives && seenDrives[thisDriveIdx] != driver.boardAddress)
	{
		thisDriveIdx++;
	}
	bool const newOne = (thisDriveIdx == numSeenDrives);
	if (newOne)
	{
		if (numSeenDrives < HANGPRINTER_AXES)
		{
			seenDrives[thisDriveIdx] = driver.boardAddress;
			numSeenDrives++;
		}
		else // we don't have space for a new one
		{
			reply.printf("Max CAN addresses we can reference is %d. Can't reference board %d.", HANGPRINTER_AXES, driver.boardAddress);
			numSeenDrives = HANGPRINTER_AXES;
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
			encoderEstimate = LoadLEFloat(buf->msg.raw);
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
GCodeResult HangprinterKinematics::SetODrive3TorqueModeInner(DriverId const driver, float const torque, const StringRef& reply) noexcept
{
	// Get a buffer
	CanMessageBuffer * buf = CanInterface::ODrive::PrepareSimpleMessage(driver, reply);
	if (buf == nullptr)
	{
		return GCodeResult::error;
	}

	// Set the right target torque
	buf->id = CanInterface::ODrive::ArbitrationId(driver, CANSimple::MSG_SET_INPUT_TORQUE);
	buf->dataLength = 4;
	buf->remote = false;
	memcpy(buf->msg.raw, &torque, sizeof(torque));
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
		CanMessageBuffer * buf = CanInterface::ODrive::PrepareSimpleMessage(driver, reply);
		if (buf == nullptr)
		{
			return GCodeResult::error;
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
GCodeResult HangprinterKinematics::SetODrive3TorqueMode(DriverId const driver, float torque, const StringRef& reply) noexcept
{
	GCodeResult res = GCodeResult::ok;
	constexpr double MIN_TORQUE_NM = 0.0001;
	if (fabs(torque) < MIN_TORQUE_NM)
	{
		res = SetODrive3PosMode(driver, reply);
		if (res == GCodeResult::ok)
		{
			reply.cat("pos_mode, ");
		}
	}
	else
	{
		// Set the right sign
		torque = std::abs(torque);
		if (driver.boardAddress == 40 || driver.boardAddress == 41) // Driver direction is not stored on main board!! (will be in the future)
		{
			torque = -torque;
		}
		res = SetODrive3TorqueModeInner(driver, torque, reply);
		if (res == GCodeResult::ok)
		{
			reply.catf("%.6f Nm, ", (double)torque);
		}
	}
	return res;
}
#endif // DUAL_CAN

#endif // SUPPORT_HANGPRINTER

// End
