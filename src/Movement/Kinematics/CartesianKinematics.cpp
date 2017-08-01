/*
 * CartesianKinematics.cpp
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#include "CartesianKinematics.h"
#include "Movement/DDA.h"

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

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() or dda.SetPositions().
// Return true if the entire move should be stopped, false if only the motor concerned should be stopped.
bool CartesianKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
	dda.SetDriveCoordinate(hitPoint * stepsPerMm[axis], axis);
	return false;
}

// End
