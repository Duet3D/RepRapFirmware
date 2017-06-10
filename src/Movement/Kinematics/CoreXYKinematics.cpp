/*
 * CoreXYKinematics.cpp
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#include "CoreXYKinematics.h"

CoreXYKinematics::CoreXYKinematics() : CoreBaseKinematics(KinematicsType::coreXY)
{
}

// Return the name of the current kinematics
const char *CoreXYKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "coreXY" : "CoreXY";
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreXYKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	// Convert the axes
	machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) - (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[X_AXIS] * stepsPerMm[X_AXIS] * stepsPerMm[Y_AXIS]);
	machinePos[Y_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) + (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[Y_AXIS] * stepsPerMm[X_AXIS] * stepsPerMm[Y_AXIS]);
	machinePos[Z_AXIS] = motorPos[Z_AXIS]/stepsPerMm[Z_AXIS];

	// Convert any additional axes
	for (size_t drive = XYZ_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Calculate the movement fraction for a single axis motor
float CoreXYKinematics::MotorFactor(size_t drive, const float directionVector[]) const
{
	switch(drive)
	{
	case X_AXIS:
		return (directionVector[X_AXIS] * axisFactors[X_AXIS]) + (directionVector[Y_AXIS] * axisFactors[Y_AXIS]);

	case Y_AXIS:
		return (directionVector[Y_AXIS] * axisFactors[Y_AXIS]) - (directionVector[X_AXIS] * axisFactors[X_AXIS]);

	default:
		return directionVector[drive];
	}
}

// End
