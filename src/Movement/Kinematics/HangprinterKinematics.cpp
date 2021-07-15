/*
 * HangprinterKinematics.cpp
 *
 *  Created on: 24 Nov 2017
 *      Author: David
 */

#include "HangprinterKinematics.h"
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

DEFINE_GET_OBJECT_MODEL_TABLE(HangprinterKinematics)

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
	doneAutoCalibration = false;
	Recalc();
}

// Recalculate the derived parameters
void HangprinterKinematics::Recalc() noexcept
{
	printRadiusSquared = fsquare(printRadius);
	Da2 = fsquare(anchors[A_AXIS][0]) + fsquare(anchors[A_AXIS][1]) + fsquare(anchors[A_AXIS][2]);
	Db2 = fsquare(anchors[B_AXIS][0]) + fsquare(anchors[B_AXIS][1]) + fsquare(anchors[B_AXIS][2]);
	Dc2 = fsquare(anchors[C_AXIS][0]) + fsquare(anchors[C_AXIS][1]) + fsquare(anchors[C_AXIS][2]);
	Xab = anchors[A_AXIS][0] - anchors[B_AXIS][0]; // maybe zero
	Xbc = anchors[B_AXIS][0] - anchors[C_AXIS][0];
	Xca = anchors[C_AXIS][0] - anchors[A_AXIS][0];
	Yab = anchors[A_AXIS][1] - anchors[B_AXIS][1];
	Ybc = anchors[B_AXIS][1] - anchors[C_AXIS][1];
	Yca = anchors[C_AXIS][1] - anchors[A_AXIS][1];
	Zab = anchors[A_AXIS][2] - anchors[B_AXIS][2];
	Zbc = anchors[B_AXIS][2] - anchors[C_AXIS][2];
	Zca = anchors[C_AXIS][2] - anchors[A_AXIS][2];
	P = (  anchors[B_AXIS][0] * Yca
		 - anchors[A_AXIS][0] * anchors[C_AXIS][1]
		 + anchors[A_AXIS][1] * anchors[C_AXIS][0]
		 - anchors[B_AXIS][1] * Xca
		) * 2;
	P2 = fsquare(P);
	Q = (  anchors[B_AXIS][Y_AXIS] * Zca
		 - anchors[A_AXIS][Y_AXIS] * anchors[C_AXIS][Z_AXIS]
		 + anchors[A_AXIS][Z_AXIS] * anchors[C_AXIS][Y_AXIS]
		 - anchors[B_AXIS][Z_AXIS] * Yca
		) * 2;
	R = - (  anchors[B_AXIS][0] * Zca
		 + anchors[A_AXIS][0] * anchors[C_AXIS][2]
		 + anchors[A_AXIS][2] * anchors[C_AXIS][0]
		 - anchors[B_AXIS][2] * Xca
		) * 2;
	U = (anchors[A_AXIS][2] * P2) + (anchors[A_AXIS][0] * Q * P) + (anchors[A_AXIS][1] * R * P);
	A = (P2 + fsquare(Q) + fsquare(R)) * 2;

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

	debugPrintf("Recalced params\nDa2: %.2f, Db2: %.2f, Dc2: %.2f, Xab: %.2f, Xbc: %.2f, Xca: %.2f, Yab: %.2f, Ybc: %.2f, Yca: %.2f, Zab: %.2f, Zbc: %.2f, Zca: %.2f, P: %.2f, P2: %.2f, Q: %.2f, R: %.2f, U: %.2f, A: %.2f\n", (double)Da2, (double)Db2, (double)Dc2, (double)Xab, (double)Xbc, (double)Xca, (double)Yab, (double)Ybc, (double)Yca, (double)Zab, (double)Zbc, (double)Zca, (double)P, (double)P2, (double)Q, (double)R, (double)U, (double)A);
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
			reply.printf("Hangprinter\n"
				"A:%.2f, %.2f, %.2f\n"
				"B:%.2f, %.2f, %.2f\n"
				"C:%.2f, %.2f, %.2f\n"
				"D:%.2f, %.2f, %.2f\n"
				"P:Print radius: %.1f\n"
				"S:Segments/s: %d\n"
				"T:Min segment length: %.2f\n",
				(double)anchors[A_AXIS][X_AXIS], (double)anchors[A_AXIS][Y_AXIS], (double)anchors[A_AXIS][Z_AXIS],
				(double)anchors[B_AXIS][X_AXIS], (double)anchors[B_AXIS][Y_AXIS], (double)anchors[B_AXIS][Z_AXIS],
				(double)anchors[C_AXIS][X_AXIS], (double)anchors[C_AXIS][Y_AXIS], (double)anchors[C_AXIS][Z_AXIS],
				(double)anchors[D_AXIS][X_AXIS], (double)anchors[D_AXIS][Y_AXIS], (double)anchors[D_AXIS][Z_AXIS],
				(double)printRadius,
				(int)GetSegmentsPerSecond(), (double)GetMinSegmentLength()
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

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void HangprinterKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	InverseTransform(
		MotorPosToLinePos(motorPos[A_AXIS], A_AXIS) + lineLengthsOrigin[A_AXIS],
		MotorPosToLinePos(motorPos[B_AXIS], B_AXIS) + lineLengthsOrigin[B_AXIS],
		MotorPosToLinePos(motorPos[C_AXIS], C_AXIS) + lineLengthsOrigin[C_AXIS],
		machinePos);
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

#if HAS_MASS_STORAGE

// Write the parameters that are set by auto calibration to a file, returning true if success
bool HangprinterKinematics::WriteCalibrationParameters(FileStore *f) const noexcept
{
	bool ok = f->Write("; Hangprinter parameters\n");
	if (ok)
	{
		String<100> scratchString;
		scratchString.printf("M669 K6 A%.3f:%.3f:%.3f B%.3f:%.3f:%.3f C%.3f:%.3f:%.3f D%.3f:%.3f:%.3f P%.1f\n",
							(double)anchors[A_AXIS][X_AXIS], (double)anchors[A_AXIS][Y_AXIS], (double)anchors[A_AXIS][Z_AXIS],
							(double)anchors[B_AXIS][X_AXIS], (double)anchors[B_AXIS][Y_AXIS], (double)anchors[B_AXIS][Z_AXIS],
							(double)anchors[C_AXIS][X_AXIS], (double)anchors[C_AXIS][Y_AXIS], (double)anchors[C_AXIS][Z_AXIS],
							(double)anchors[D_AXIS][X_AXIS], (double)anchors[D_AXIS][Y_AXIS], (double)anchors[D_AXIS][Z_AXIS],
							(double)printRadius);
		ok = f->Write(scratchString.c_str());
		if (ok) {
			scratchString.printf("M666 Q%.6f R%.3f:%.3f:%.3f:%.3f U%d:%d:%d:%d ",
								(double)spoolBuildupFactor, (double)spoolRadii[A_AXIS],
								(double)spoolRadii[B_AXIS], (double)spoolRadii[C_AXIS], (double)spoolRadii[D_AXIS],
								(int)mechanicalAdvantage[A_AXIS], (int)mechanicalAdvantage[B_AXIS],
								(int)mechanicalAdvantage[C_AXIS], (int)mechanicalAdvantage[D_AXIS]
					);
			ok = f->Write(scratchString.c_str());
			if (ok) {
				scratchString.printf("O%d:%d:%d:%d L%d:%d:%d:%d H%d:%d:%d:%d J%d:%d:%d:%d\n",
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
	return ok;
}

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool HangprinterKinematics::WriteResumeSettings(FileStore *f) const noexcept
{
	return !doneAutoCalibration || WriteCalibrationParameters(f);
}

#endif

// Calculate the Cartesian coordinates from the motor coordinates
void HangprinterKinematics::InverseTransform(float La, float Lb, float Lc, float machinePos[3]) const noexcept
{
	// Calculate PQRST such that x = (Qz + S)/P, y = (Rz + T)/P
	const float S = - Yab * (fsquare(Lc) - Dc2)
					- Yca * (fsquare(Lb) - Db2)
					- Ybc * (fsquare(La) - Da2);
	const float T = - Xab * (fsquare(Lc) - Dc2)
					+ Xca * (fsquare(Lb) - Db2)
					+ Xbc * (fsquare(La) - Da2);

	// Calculate quadratic equation coefficients
	const float halfB = (S * Q) - (R * T) - U;
	const float C = fsquare(S) + fsquare(T) + (anchors[A_AXIS][Y_AXIS] * T - anchors[A_AXIS][X_AXIS] * S) * P * 2 + (Da2 - fsquare(La)) * P2;
	debugPrintf("S: %.2f, T: %.2f, halfB: %.2f, C: %.2f, P: %.2f, A: %.2f\n", (double)S, (double)T, (double)halfB, (double)C, (double)P, (double) A);

	// Solve the quadratic equation for z
	machinePos[2] = (- halfB - fastSqrtf(fabsf(fsquare(halfB) - A * C)))/A;

	// Substitute back for X and Y
	machinePos[0] = (Q * machinePos[2] + S)/P;
	machinePos[1] = (R * machinePos[2] + T)/P;

	debugPrintf("Motor %.2f,%.2f,%.2f to Cartesian %.2f,%.2f,%.2f\n", (double)La, (double)Lb, (double)Lc, (double)machinePos[0], (double)machinePos[1], (double)machinePos[2]);
}

// Auto calibrate from a set of probe points returning true if it failed
// We can't calibrate all the XY parameters because translations and rotations of the anchors don't alter the height. Therefore we calibrate the following:
// 0, 1, 2 Spool zero length motor positions (this is like endstop calibration on a delta)
// 3       Y coordinate of the B anchor
// 4, 5    X and Y coordinates of the C anchor
// 6, 7, 8 Heights of the A, B, C anchors
// We don't touch the XY coordinates of the A anchor or the X coordinate of the B anchor.
bool HangprinterKinematics::DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept
{
	const size_t NumHangprinterFactors = 9;		// maximum number of machine factors we can adjust

	if (numFactors != 3 && numFactors != 6 && numFactors != NumHangprinterFactors)
	{
		reply.printf("Hangprinter calibration with %d factors requested but only 3, 6, and 9 supported", numFactors);
		return true;
	}

	if (reprap.Debug(moduleMove))
	{
		String<StringLength256> scratchString;
		PrintParameters(scratchString.GetRef());
		debugPrintf("%s\n", scratchString.c_str());
	}

	// The following is for printing out the calculation time, see later
	//uint32_t startTime = reprap.GetPlatform()->GetInterruptClocks();

	// Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
	FixedMatrix<floatc_t, MaxCalibrationPoints, 3> probeMotorPositions;
	floatc_t corrections[MaxCalibrationPoints];
	Deviation initialDeviation;
	const size_t numPoints = probePoints.NumberOfProbePoints();

	{
		floatc_t initialSum = 0.0, initialSumOfSquares = 0.0;
		for (size_t i = 0; i < numPoints; ++i)
		{
			corrections[i] = 0.0;
			float machinePos[3];
			const floatc_t zp = reprap.GetMove().GetProbeCoordinates(i, machinePos[X_AXIS], machinePos[Y_AXIS], probePoints.PointWasCorrected(i));
			machinePos[Z_AXIS] = 0.0;

			probeMotorPositions(i, A_AXIS) = fastSqrtf(LineLengthSquared(machinePos, anchors[A_AXIS]));
			probeMotorPositions(i, B_AXIS) = fastSqrtf(LineLengthSquared(machinePos, anchors[B_AXIS]));
			probeMotorPositions(i, C_AXIS) = fastSqrtf(LineLengthSquared(machinePos, anchors[C_AXIS]));
			initialSumOfSquares += fcsquare(zp);
		}
		initialDeviation.Set(initialSumOfSquares, initialSum, numPoints);
	}

	// Do 1 or more Newton-Raphson iterations
	Deviation finalDeviation;
	unsigned int iteration = 0;
	for (;;)
	{
		// Build a Nx9 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
		FixedMatrix<floatc_t, MaxCalibrationPoints, NumHangprinterFactors> derivativeMatrix;
		for (size_t i = 0; i < numPoints; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				const size_t adjustedJ = (numFactors == 8 && j >= 6) ? j + 1 : j;		// skip diagonal rod length if doing 8-factor calibration
				derivativeMatrix(i, j) =
					ComputeDerivative(adjustedJ, probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS));
			}
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Derivative matrix", derivativeMatrix, numPoints, numFactors);
		}

		// Now build the normal equations for least squares fitting
		FixedMatrix<floatc_t, NumHangprinterFactors, NumHangprinterFactors + 1> normalMatrix;
		for (size_t i = 0; i < numFactors; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				floatc_t temp = derivativeMatrix(0, i) * derivativeMatrix(0, j);
				for (size_t k = 1; k < numPoints; ++k)
				{
					temp += derivativeMatrix(k, i) * derivativeMatrix(k, j);
				}
				normalMatrix(i, j) = temp;
			}
			floatc_t temp = derivativeMatrix(0, i) * -((floatc_t)probePoints.GetZHeight(0) + corrections[0]);
			for (size_t k = 1; k < numPoints; ++k)
			{
				temp += derivativeMatrix(k, i) * -((floatc_t)probePoints.GetZHeight(k) + corrections[k]);
			}
			normalMatrix(i, numFactors) = temp;
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Normal matrix", normalMatrix, numFactors, numFactors + 1);
		}

		if (!normalMatrix.GaussJordan(numFactors, numFactors + 1))
		{
			reply.copy("Unable to calculate calibration parameters. Please choose different probe points.");
			return true;
		}

		floatc_t solution[NumHangprinterFactors];
		for (size_t i = 0; i < numFactors; ++i)
		{
			solution[i] = normalMatrix(i, numFactors);
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Solved matrix", normalMatrix, numFactors, numFactors + 1);
			PrintVector("Solution", solution, numFactors);

			// Calculate and display the residuals
			// Save a little stack by not allocating a residuals vector, because stack for it doesn't only get reserved when debug is enabled.
			debugPrintf("Residuals:");
			for (size_t i = 0; i < numPoints; ++i)
			{
				floatc_t residual = probePoints.GetZHeight(i);
				for (size_t j = 0; j < numFactors; ++j)
				{
					residual += solution[j] * derivativeMatrix(i, j);
				}
				debugPrintf(" %7.4f", (double)residual);
			}

			debugPrintf("\n");
		}

		Adjust(numFactors, solution);								// adjust the delta parameters

		float heightAdjust[3];
		for (size_t drive = 0; drive < 3; ++drive)
		{
			heightAdjust[drive] = (float)solution[drive];			// convert double to float
		}
		reprap.GetMove().AdjustMotorPositions(heightAdjust, 3);		// adjust the motor positions

		// Calculate the expected probe heights using the new parameters
		{
			floatc_t expectedResiduals[MaxCalibrationPoints];
			floatc_t finalSum = 0.0, finalSumOfSquares = 0.0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				for (size_t axis = 0; axis < 3; ++axis)
				{
					probeMotorPositions(i, axis) += solution[axis];
				}
				float newPosition[3];
				InverseTransform(probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS), newPosition);
				corrections[i] = newPosition[Z_AXIS];
				expectedResiduals[i] = probePoints.GetZHeight(i) + newPosition[Z_AXIS];
				finalSum += expectedResiduals[i];
				finalSumOfSquares += fcsquare(expectedResiduals[i]);
			}

			finalDeviation.Set(finalSumOfSquares, finalSum, numPoints);

			if (reprap.Debug(moduleMove))
			{
				PrintVector("Expected probe error", expectedResiduals, numPoints);
			}
		}

		// Decide whether to do another iteration. Two is slightly better than one, but three doesn't improve things.
		// Alternatively, we could stop when the expected RMS error is only slightly worse than the RMS of the residuals.
		++iteration;
		if (iteration == 2)
		{
			break;
		}
	}

	if (reprap.Debug(moduleMove))
	{
		String<StringLength256> scratchString;
		PrintParameters(scratchString.GetRef());
		debugPrintf("%s\n", scratchString.c_str());
	}

	reprap.GetMove().SetInitialCalibrationDeviation(initialDeviation);
	reprap.GetMove().SetLatestCalibrationDeviation(finalDeviation, numFactors);

	reply.printf("Calibrated %d factors using %d points, (mean, deviation) before (%.3f, %.3f) after (%.3f, %.3f)",
			numFactors, numPoints,
			(double)initialDeviation.GetMean(), (double)initialDeviation.GetDeviationFromMean(),
			(double)finalDeviation.GetMean(), (double)finalDeviation.GetDeviationFromMean());

	// We don't want to call MessageF(LogMessage, "%s\n", reply.c_str()) here because that will allocate a buffer within MessageF, which adds to our stack usage.
	// Better to allocate the buffer here so that it uses the same stack space as the arrays that we have finished with
	{
		String<StringLength256> scratchString;
		scratchString.printf("%s\n", reply.c_str());
		reprap.GetPlatform().Message(LogWarn, scratchString.c_str());
	}

    doneAutoCalibration = true;
    return false;
}

// Compute the derivative of height with respect to a parameter at the specified motor endpoints.
// 'deriv' indicates the parameter as follows:
// 0, 1, 2 = A, B, C line length adjustments
// 3 = B anchor Y coordinate
// 4, 5 = C anchor X and Y coordinates
// 6, 7, 8 = A, B and C anchor Z coordinates
floatc_t HangprinterKinematics::ComputeDerivative(unsigned int deriv, float La, float Lb, float Lc) const noexcept
{
	const float perturb = 0.2;			// perturbation amount in mm
	HangprinterKinematics hiParams(*this), loParams(*this);
	switch(deriv)
	{
	case 0:
	case 1:
	case 2:
		// Line length corrections
		break;

	case 3:
		hiParams.anchors[B_AXIS][Y_AXIS] += perturb;
		loParams.anchors[B_AXIS][Y_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 4:
		hiParams.anchors[C_AXIS][X_AXIS] += perturb;
		loParams.anchors[C_AXIS][X_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 5:
		hiParams.anchors[C_AXIS][Y_AXIS] += perturb;
		loParams.anchors[C_AXIS][Y_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 6:
		hiParams.anchors[A_AXIS][Z_AXIS] += perturb;
		loParams.anchors[A_AXIS][Z_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 7:
		hiParams.anchors[B_AXIS][Z_AXIS] += perturb;
		loParams.anchors[B_AXIS][Z_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 8:
		hiParams.anchors[B_AXIS][Z_AXIS] += perturb;
		loParams.anchors[B_AXIS][Z_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;
	}

	float newPos[3];
	hiParams.InverseTransform((deriv == 0) ? La + perturb : La, (deriv == 1) ? Lb + perturb : Lb, (deriv == 2) ? Lc + perturb : Lc, newPos);

	const float zHi = newPos[Z_AXIS];
	loParams.InverseTransform((deriv == 0) ? La - perturb : La, (deriv == 1) ? Lb - perturb : Lb, (deriv == 2) ? Lc - perturb : Lc, newPos);
	const float zLo = newPos[Z_AXIS];
	return ((floatc_t)zHi - (floatc_t)zLo)/(floatc_t)(2 * perturb);
}

// Perform 3, 6 or 9-factor adjustment.
// The input vector contains the following parameters in this order:
// 0, 1, 2 = A, B, C line length adjustments
// 3 = B anchor Y coordinate
// 4, 5 = C anchor X and Y coordinates
// 6, 7, 8 = A, B and C anchor Z coordinates
void HangprinterKinematics::Adjust(size_t numFactors, const floatc_t v[]) noexcept
{
	if (numFactors >= 4)
	{
		anchors[B_AXIS][Y_AXIS] += (float)v[3];
	}
	if (numFactors >= 5)
	{
		anchors[C_AXIS][X_AXIS] += (float)v[4];
	}
	if (numFactors >= 6)
	{
		anchors[C_AXIS][Y_AXIS] += (float)v[5];
	}
	if (numFactors >= 7)
	{
		anchors[A_AXIS][Z_AXIS] += (float)v[6];
	}
	if (numFactors >= 8)
	{
		anchors[B_AXIS][Z_AXIS] += (float)v[7];
	}
	if (numFactors >= 9)
	{
		anchors[C_AXIS][Z_AXIS] += (float)v[8];
	}

	Recalc();
}

// Print all the parameters for debugging
void HangprinterKinematics::PrintParameters(const StringRef& reply) const noexcept
{
	reply.printf("Anchor coordinates (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)\n",
					(double)anchors[A_AXIS][X_AXIS], (double)anchors[A_AXIS][Y_AXIS], (double)anchors[A_AXIS][Z_AXIS],
					(double)anchors[B_AXIS][X_AXIS], (double)anchors[B_AXIS][Y_AXIS], (double)anchors[B_AXIS][Z_AXIS],
					(double)anchors[C_AXIS][X_AXIS], (double)anchors[C_AXIS][Y_AXIS], (double)anchors[C_AXIS][Z_AXIS]);
}

std::optional<float> HangprinterKinematics::GetODrive3EncoderEstimate(DriverId const driver, bool const makeReference, const StringRef& reply, bool const subtractReference)
{
	const uint8_t cmd = CANSimple::MSG_GET_ENCODER_ESTIMATES;
	static CanAddress seenDrives[HANGPRINTER_AXES] = { 0, 0, 0, 0 };
	static float referencePositions[HANGPRINTER_AXES] = { 0.0, 0.0, 0.0, 0.0 };
	static size_t numSeenDrives = 0;
	size_t thisDriveIdx = 0;

	while (thisDriveIdx < numSeenDrives and seenDrives[thisDriveIdx] != driver.boardAddress)
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

	CanMessageBuffer * buf = CanInterface::ODrive::PrepareSimpleMessage(driver, cmd, reply);
	if (buf == nullptr)
	{
		return {};
	}

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

	if (newOne and not ok)
	{
		seenDrives[thisDriveIdx] = 0;
		numSeenDrives--;
	}

	CanMessageBuffer::Free(buf);
	if (ok)
	{
		return encoderEstimate;
	}
	return {};
}

GCodeResult HangprinterKinematics::ReadODrive3Encoder(DriverId const driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	std::optional<float> const estimate = GetODrive3EncoderEstimate(driver, gb.Seen('S'), reply, true);
	if (estimate.has_value())
	{
		float directionCorrectedEncoderValue = estimate.value();
		if (driver.boardAddress == 40 or driver.boardAddress == 41) // Driver direction is not stored on main board!! (will be in the future)
		{
			directionCorrectedEncoderValue *= -1.0;
		}
		reply.catf("%.2f, ", (double)(directionCorrectedEncoderValue * 360.0));
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}

GCodeResult HangprinterKinematics::SetODrive3TorqueModeInner(DriverId const driver, float const torque, const StringRef& reply)
{
	// Set the right target torque
	CanMessageBuffer * buf = CanInterface::ODrive::PrepareSimpleMessage(driver, CANSimple::MSG_SET_INPUT_TORQUE, reply);
	if (buf == nullptr)
	{
		return GCodeResult::error;
	}
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

GCodeResult HangprinterKinematics::SetODrive3PosMode(DriverId const driver, const StringRef& reply)
{
	std::optional<float> const estimate = GetODrive3EncoderEstimate(driver, false, reply, false);
	if (estimate.has_value())
	{
		float const desiredPos = estimate.value();
		CanMessageBuffer * buf = CanInterface::ODrive::PrepareSimpleMessage(driver, CANSimple::MSG_SET_INPUT_POS, reply);
		if (buf == nullptr)
		{
			return GCodeResult::error;
		}
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

GCodeResult HangprinterKinematics::SetODrive3TorqueMode(DriverId const driver, float torque, const StringRef& reply)
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
		if (driver.boardAddress == 42 or driver.boardAddress == 43) // Driver direction is not stored on main board!! (will be in the future)
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

// End
