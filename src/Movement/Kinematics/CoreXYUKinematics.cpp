/*
 * CoreXYUKinematics.cpp
 *
 *  Created on: 4 Jun 2017
 *      Author: Lars
 */

#include "CoreXYUKinematics.h"
#include "GCodes/GCodes.h"

const size_t CoreXYU_AXES = 5;
const size_t U_AXIS = 3;			// X2
const size_t V_AXIS = 4;			// Y2

CoreXYUKinematics::CoreXYUKinematics() : CoreBaseKinematics(KinematicsType::coreXYU)
{
}

// Return the name of the current kinematics
const char *CoreXYUKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "coreXYU" : "CoreXYU";
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
// This function is used for CoreXY and CoreXZ kinematics, but it overridden for CoreXYU kinematics
bool CoreXYUKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) /*override*/
{
	if (mCode == 667)
	{
		bool seen = false;
		for (size_t axis = 0; axis < CoreXYU_AXES; ++axis)
		{
			if (gb.Seen(GCodes::axisLetters[axis]))
			{
				axisFactors[axis] = gb.GetFValue();
				seen = true;
			}
			else
			{
				axisFactors[axis] = 1.0;
			}
		}
		if (!seen && !gb.Seen('S'))
		{
			reply.printf("Printer mode is %s with axis factors", GetName(false));
			for (size_t axis = 0; axis < CoreXYU_AXES; ++axis)
			{
				reply.catf(" %c:%f", GCodes::axisLetters[axis], axisFactors[axis]);
			}
		}
		return seen;
	}
	else
	{
		return CoreBaseKinematics::Configure(mCode, gb, reply, error);
	}
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreXYUKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
  // Convert the axes
	machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) - (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[X_AXIS] * stepsPerMm[X_AXIS] * stepsPerMm[Y_AXIS]);
	machinePos[Y_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) + (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[Y_AXIS] * stepsPerMm[X_AXIS] * stepsPerMm[Y_AXIS]);
	machinePos[U_AXIS] = ((motorPos[U_AXIS] * stepsPerMm[V_AXIS]) - (motorPos[V_AXIS] * stepsPerMm[U_AXIS]))
								/(2 * axisFactors[V_AXIS] * stepsPerMm[U_AXIS] * stepsPerMm[V_AXIS]);
	machinePos[V_AXIS] = ((motorPos[U_AXIS] * stepsPerMm[V_AXIS]) + (motorPos[V_AXIS] * stepsPerMm[U_AXIS]))
								/(2 * axisFactors[V_AXIS] * stepsPerMm[U_AXIS] * stepsPerMm[V_AXIS]);

	machinePos[Z_AXIS] = motorPos[Z_AXIS]/stepsPerMm[Z_AXIS];

	// Convert any additional axes
	for (size_t drive = CoreXYU_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified endstop axis uses shared motors.
// Used to determine whether to abort the whole move or just one motor when an endstop switch is triggered.
bool CoreXYUKinematics::DriveIsShared(size_t drive) const
{
	return drive == X_AXIS || drive == Y_AXIS
			|| drive == U_AXIS || drive == V_AXIS;			// X, Y and U has endstops. V don't have endstop switches, but include them here just in case
}

// Calculate the movement fraction for a single axis motor
float CoreXYUKinematics::MotorFactor(size_t drive, const float directionVector[]) const
{
	switch(drive)
	{
	case X_AXIS:
		return (directionVector[X_AXIS] * axisFactors[X_AXIS]) + (directionVector[Y_AXIS] * axisFactors[Y_AXIS]);
	case Y_AXIS:
		return (directionVector[Y_AXIS] * axisFactors[Y_AXIS]) - (directionVector[X_AXIS] * axisFactors[X_AXIS]);
	case U_AXIS: // X2, Use Y and U to calculate
		return (directionVector[U_AXIS] * axisFactors[U_AXIS]) + (directionVector[Y_AXIS] * axisFactors[Y_AXIS]);
	case V_AXIS: // Y2, Use Y and U to calculate
		return (directionVector[Y_AXIS] * axisFactors[Y_AXIS]) - (directionVector[U_AXIS] * axisFactors[U_AXIS]);
	default:
		return directionVector[drive];
	}
}

// End
