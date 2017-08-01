/*
 * CartesianKinematics.cpp
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#include "CartesianKinematics.h"

CartesianKinematics::CartesianKinematics() : ZLeadscrewKinematics(KinematicsType::cartesian)
{
}

// Return the name of the current kinematics
const char *CartesianKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "cartesian" : "Cartesian";
}

// Convert Cartesian coordinates to motor coordinates
bool CartesianKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[]) const
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CartesianKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	// Convert all axes
	for (size_t drive = 0; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// End
