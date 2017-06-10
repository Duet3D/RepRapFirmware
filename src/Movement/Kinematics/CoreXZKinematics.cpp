/*
 * CoreXZKinematics.cpp
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#include "CoreXZKinematics.h"

CoreXZKinematics::CoreXZKinematics() : CoreBaseKinematics(KinematicsType::coreXZ)
{
}

// Return the name of the current kinematics
const char *CoreXZKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "coreXZ" : "CoreXZ";
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreXZKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Z_AXIS]) - (motorPos[Z_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[X_AXIS] * stepsPerMm[X_AXIS] * stepsPerMm[Z_AXIS]);
	machinePos[Y_AXIS] = motorPos[Y_AXIS]/stepsPerMm[Y_AXIS];
	machinePos[Z_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Z_AXIS]) + (motorPos[Z_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[Z_AXIS] * stepsPerMm[X_AXIS] * stepsPerMm[Z_AXIS]);

	// Convert any additional axes linearly
	for (size_t drive = XYZ_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Calculate the movement fraction for a single axis motor of a Cartesian-like printer
float CoreXZKinematics::MotorFactor(size_t drive, const float directionVector[]) const
{
	switch(drive)
	{
	case X_AXIS:
		return (directionVector[X_AXIS] * axisFactors[X_AXIS]) + (directionVector[Z_AXIS] * axisFactors[Z_AXIS]);

	case Z_AXIS:
		return (directionVector[Z_AXIS] * axisFactors[Z_AXIS]) - (directionVector[X_AXIS] * axisFactors[X_AXIS]);

	default:
		return directionVector[drive];
	}
}

// End
