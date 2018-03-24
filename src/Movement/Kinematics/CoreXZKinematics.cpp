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

// Convert Cartesian coordinates to motor coordinates
bool CoreXZKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	motorPos[X_AXIS] = lrintf(((machinePos[X_AXIS] * axisFactors[X_AXIS]) + (machinePos[Z_AXIS] * axisFactors[Z_AXIS])) * stepsPerMm[X_AXIS]);
	motorPos[Y_AXIS] = lrintf(machinePos[Y_AXIS] * stepsPerMm[Y_AXIS]);
	motorPos[Z_AXIS] = lrintf(((machinePos[X_AXIS] * axisFactors[X_AXIS]) - (machinePos[Z_AXIS] * axisFactors[Z_AXIS])) * stepsPerMm[Z_AXIS]);

	for (size_t axis = XYZ_AXES; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreXZKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	// Convert the main axes
	const float xzStepsMmm = stepsPerMm[X_AXIS] * stepsPerMm[Z_AXIS];
	machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Z_AXIS]) + (motorPos[Z_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[X_AXIS] * xzStepsMmm);
	machinePos[Y_AXIS] = motorPos[Y_AXIS]/stepsPerMm[Y_AXIS];
	machinePos[Z_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Z_AXIS]) - (motorPos[Z_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[Z_AXIS] * xzStepsMmm);

	// Convert any additional axes linearly
	for (size_t drive = XYZ_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified endstop axis uses shared motors.
// Used to determine whether to abort the whole move or just one motor when an endstop switch is triggered.
bool CoreXZKinematics::DriveIsShared(size_t drive) const
{
	return drive == X_AXIS || drive == Z_AXIS;
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void CoreXZKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const
{
	// Ideally we would do something here, but for now we don't.
	// This could mean that a simultaneous X and Z movement with Z at maximum speed up and X at 3x that speed would be under-powered,
	// but the workaround in that case would be just to lower the maximum Z speed a little, which won't affect printing speed significantly.
}

// End
