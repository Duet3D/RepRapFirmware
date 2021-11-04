/*
 * PolarKinematics.cpp
 *
 *  Created on: 13 Oct 2017
 *      Author: David
 */

#include "PolarKinematics.h"

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Storage/MassStorage.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/DDA.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(PolarKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry PolarKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
};

constexpr uint8_t PolarKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE(PolarKinematics)

#endif

// Constructor
PolarKinematics::PolarKinematics() noexcept
	: Kinematics(KinematicsType::polar, SegmentationType(true, false, false)),
	  minRadius(0.0), maxRadius(DefaultMaxRadius), homedRadius(0.0),
	  maxTurntableSpeed(DefaultMaxTurntableSpeed), maxTurntableAcceleration(DefaultMaxTurntableAcceleration)
{
	Recalc();
}

// Return the name of the current kinematics.
// If 'forStatusReport' is true then the string must be the one for that kinematics expected by DuetWebControl and PanelDue.
// Otherwise it should be in a format suitable for printing.
// For any new kinematics, the same string can be returned regardless of the parameter.
const char *PolarKinematics::GetName(bool forStatusReport) const noexcept
{
	return "Polar";
}

// Set or report the parameters from a M665, M666 or M669 command
// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 665, 666, 667 or 669)
// then search for parameters used to configure the current kinematics. If any are found, perform appropriate actions,
// and return true if the changes affect the geometry.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
// If 'mCode' does not apply to this kinematics, call the base class version of this function, which will print a suitable error message.
bool PolarKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
	if (mCode == 669)
	{
		bool seen = false;
		if (gb.Seen('R'))
		{
			seen = true;
			float radiusLimits[2];
			size_t numRadiusLimits = 2;
			gb.GetFloatArray(radiusLimits, numRadiusLimits, false);
			if (numRadiusLimits == 2)
			{
				minRadius = radiusLimits[0];
				maxRadius = radiusLimits[1];
			}
			else
			{
				minRadius = 0.0;
				maxRadius = radiusLimits[0];
			}
			homedRadius = minRadius;		// set up default
		}

		gb.TryGetFValue('H', homedRadius, seen);

		bool seenNonGeometry = TryConfigureSegmentation(gb);
		gb.TryGetFValue('A', maxTurntableAcceleration, seenNonGeometry);
		gb.TryGetFValue('F', maxTurntableSpeed, seenNonGeometry);

		if (seen)
		{
			Recalc();
		}
		else if (!seenNonGeometry && !gb.Seen('K'))
		{
			Kinematics::Configure(mCode, gb, reply, error);
			reply.catf(", radius %.1f to %.1fmm, homed radius %.1fmm",
							(double)minRadius, (double)maxRadius, (double)homedRadius);
		}
		return seen;
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

// Convert Cartesian coordinates to motor positions measured in steps from reference position
// 'machinePos' is a set of axis and extruder positions to convert
// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
// 'numAxes' is the number of machine axes to convert, which will always be at least 3
// 'motorPos' is the output vector of motor positions
// Return true if successful, false if we were unable to convert
bool PolarKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	motorPos[0] = lrintf(fastSqrtf(fsquare(machinePos[0]) + fsquare(machinePos[1])) * stepsPerMm[0]);
	motorPos[1] = (motorPos[0] == 0.0) ? 0 : lrintf(atan2f(machinePos[1], machinePos[0]) * RadiansToDegrees * stepsPerMm[1]);

	// Transform remaining axes linearly
	for (size_t axis = Z_AXIS; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return true;
}

// Convert motor positions (measured in steps from reference position) to Cartesian coordinates
// 'motorPos' is the input vector of motor positions
// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
// 'numDrives' is the number of machine drives to convert, which will always be at least 3
// 'machinePos' is the output set of converted axis and extruder positions
void PolarKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	const float angle = (motorPos[1] * DegreesToRadians)/stepsPerMm[1];
	const float radius = (float)motorPos[0]/stepsPerMm[0];
	machinePos[0] = radius * cosf(angle);
	machinePos[1] = radius * sinf(angle);

	// Convert any additional axes linearly
	for (size_t drive = Z_AXIS; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified XY position is reachable by the print head reference point.
bool PolarKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept
{
	if (axes.IsBitSet(X_AXIS) && axes.IsBitSet(Y_AXIS))
	{
		const float r2 = fsquare(axesCoords[X_AXIS]) + fsquare(axesCoords[Y_AXIS]);
		if(r2 < minRadiusSquared || r2 > maxRadiusSquared)
		{
			return false;
		}
	}
	axes.ClearBit(X_AXIS);
	axes.ClearBit(Y_AXIS);
	return Kinematics::IsReachable(axesCoords, axes);
}

// Limit the Cartesian position that the user wants to move to, returning true if any coordinates were changed
LimitPositionResult PolarKinematics::LimitPosition(float finalCoords[], const float * null initialCoords,
													size_t numAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
	const bool m208Limited = (applyM208Limits)
								? Kinematics::LimitPositionFromAxis(finalCoords, Z_AXIS, numAxes, axesToLimit)	// call base class function to limit Z and higher axes
								: false;
	const float r2 = fsquare(finalCoords[X_AXIS]) + fsquare(finalCoords[Y_AXIS]);
	bool radiusLimited;
	if (r2 < minRadiusSquared)
	{
		radiusLimited = true;
		const float r = fastSqrtf(r2);
		if (r < 0.01)
		{
			finalCoords[X_AXIS] = minRadius;
			finalCoords[Y_AXIS] = 0.0;
		}
		else
		{
			finalCoords[X_AXIS] *= minRadius/r;
			finalCoords[Y_AXIS] *= minRadius/r;
		}
	}
	else if (r2 > maxRadiusSquared)
	{
		radiusLimited = true;
		const float r = fastSqrtf(r2);
		finalCoords[X_AXIS] *= maxRadius/r;
		finalCoords[Y_AXIS] *= maxRadius/r;
	}
	else
	{
		radiusLimited = false;
	}

	return (m208Limited || radiusLimited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void PolarKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap PolarKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	// If both X and Y have been specified then we know the positions the radius motor and the turntable, otherwise we don't
	if ((g92Axes & XyAxes) != XyAxes)
	{
		g92Axes &= ~XyAxes;
	}
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap PolarKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
	if (axesMoving.Intersects(XyzAxes))
	{
		axesMoving |= XyzAxes;
	}
	return axesMoving;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called. Optionally, update 'alreadyHomed' to indicate
// that some additional axes should be considered not homed.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap PolarKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept
{
	// Ask the base class which homing file we should call first
	const AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
	if (ret.IsNonEmpty())
	{
		// Change the returned name if it is X or Y
		if (StringEqualsIgnoreCase(filename.c_str(), "homex.g"))
		{
			filename.copy(HomeRadiusFileName);
		}
		else if (StringEqualsIgnoreCase(filename.c_str(), "homey.g"))
		{
			filename.copy(HomeBedFileName);
		}
	}

	return ret;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool PolarKinematics::QueryTerminateHomingMove(size_t axis) const noexcept
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void PolarKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept
{
	switch(axis)
	{
	case X_AXIS:	// radius
		dda.SetDriveCoordinate(lrintf(homedRadius * stepsPerMm[axis]), axis);
		break;

	case Y_AXIS:	// bed
		dda.SetDriveCoordinate(0, axis);
		break;

	default:
		const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
		dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
		break;
	}
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void PolarKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept
{
	int32_t turntableMovement = labs(dda.DriveCoordinates()[1] - dda.GetPrevious()->DriveCoordinates()[1]);
	if (turntableMovement != 0)
	{
		const float stepsPerDegree = reprap.GetPlatform().DriveStepsPerUnit(1);
		if (continuousRotationShortcut)
		{
			const int32_t stepsPerRotation = lrintf(360.0 * stepsPerDegree);
			if (turntableMovement > stepsPerRotation/2)
			{
				turntableMovement -= stepsPerRotation;
			}
			else if (turntableMovement < -stepsPerRotation/2)
			{
				turntableMovement += stepsPerRotation;
			}
		}
		if (turntableMovement != 0)
		{
			const float stepRatio = dda.GetTotalDistance() * stepsPerDegree/abs(turntableMovement);
			dda.LimitSpeedAndAcceleration(stepRatio * maxTurntableSpeed, stepRatio * maxTurntableAcceleration);
		}
	}
}

// Return true if the specified axis is a continuous rotation axis
bool PolarKinematics::IsContinuousRotationAxis(size_t axis) const noexcept
{
	return axis == Y_AXIS || Kinematics::IsContinuousRotationAxis(axis);
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap PolarKinematics::GetLinearAxes() const noexcept
{
	return AxesBitmap::MakeFromBits(Z_AXIS);
}

// Update the derived parameters after the master parameters have been changed
void PolarKinematics::Recalc()
{
	minRadiusSquared = (minRadius <= 0.0) ? 0.0 : fsquare(minRadius);
	maxRadiusSquared = fsquare(maxRadius);
}

// End
