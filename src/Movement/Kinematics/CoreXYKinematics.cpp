/*
 * CoreXYKinematics.cpp
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#include "CoreXYKinematics.h"
#include "Movement/DDA.h"

CoreXYKinematics::CoreXYKinematics() : CoreBaseKinematics(KinematicsType::coreXY)
{
}

// Return the name of the current kinematics
const char *CoreXYKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "coreXY" : "CoreXY";
}

// Convert Cartesian coordinates to motor coordinates returning true if successful
bool CoreXYKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	motorPos[X_AXIS] = lrintf(((machinePos[X_AXIS] * axisFactors[X_AXIS]) + (machinePos[Y_AXIS] * axisFactors[Y_AXIS])) * stepsPerMm[X_AXIS]);
	motorPos[Y_AXIS] = lrintf(((machinePos[X_AXIS] * axisFactors[X_AXIS]) - (machinePos[Y_AXIS] * axisFactors[Y_AXIS])) * stepsPerMm[Y_AXIS]);

	for (size_t axis = Z_AXIS; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreXYKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	// Convert the main axes
	const float xyStepsMm = stepsPerMm[X_AXIS] * stepsPerMm[Y_AXIS];
	machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) + (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[X_AXIS] * xyStepsMm);
	machinePos[Y_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) - (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[Y_AXIS] * xyStepsMm);
	machinePos[Z_AXIS] = motorPos[Z_AXIS]/stepsPerMm[Z_AXIS];

	// Convert any additional axes
	for (size_t drive = XYZ_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified endstop axis uses shared motors.
// Used to determine whether to abort the whole move or just one motor when an endstop switch is triggered.
bool CoreXYKinematics::DriveIsShared(size_t drive) const
{
	return drive == X_AXIS || drive == Y_AXIS;
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds along individual Cartesian axes have already been limited before this is called.
void CoreXYKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const
{
	const float vecX = normalisedDirectionVector[0];
	const float vecY = normalisedDirectionVector[1];
	const float vecMax = max<float>(fabs(vecX + vecY), fabs(vecX - vecY));		// pick the case for the motor that is working hardest
	if (vecMax > 0.01)															// avoid division by zero or near-zero
	{
		const Platform& platform = reprap.GetPlatform();
		const float aX = platform.Acceleration(0);
		const float aY = platform.Acceleration(1);
		const float vX = platform.MaxFeedrate(0);
		const float vY = platform.MaxFeedrate(1);
		const float aMax = (fabs(vecX) + fabs(vecY)) * aX * aY/(vecMax * (fabs(vecX) * aY + fabs(vecY) * aX));
		const float vMax = (fabs(vecX) + fabs(vecY)) * vX * vY/(vecMax * (fabs(vecX) * vY + fabs(vecY) * vX));
		dda.LimitSpeedAndAcceleration(vMax, aMax);
	}
}

// End
