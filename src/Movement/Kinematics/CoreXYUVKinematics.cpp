/*
 * CoreXYUVKinematics.cpp
 *
 *  Created on: 23 Nov 2017
 *      Author: David
 */

#include "CoreXYUVKinematics.h"

#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/DDA.h"

CoreXYUVKinematics::CoreXYUVKinematics() : CoreBaseKinematics(KinematicsType::coreXYUV)
{
}

// Return the name of the current kinematics
const char *CoreXYUVKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "coreXYUV" : "CoreXYUV";
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool CoreXYUVKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) /*override*/
{
	if (mCode == 669)
	{
		bool seen = false;
		for (size_t axis = 0; axis < CoreXYUV_AXES; ++axis)
		{
			if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
			{
				axisFactors[axis] = gb.GetFValue();
				seen = true;
			}
		}
		if (!seen && !gb.Seen('K'))
		{
			reply.printf("Kinematics is %s with axis factors", GetName(false));
			for (size_t axis = 0; axis < CoreXYUV_AXES; ++axis)
			{
				reply.catf(" %c:%.3f", reprap.GetGCodes().GetAxisLetters()[axis], (double)axisFactors[axis]);
			}
		}
		return seen;
	}

	return CoreBaseKinematics::Configure(mCode, gb, reply, error);
}

// Convert Cartesian coordinates to motor coordinates
bool CoreXYUVKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	motorPos[X_AXIS] = lrintf(((machinePos[X_AXIS] * axisFactors[X_AXIS]) + (machinePos[Y_AXIS] * axisFactors[Y_AXIS])) * stepsPerMm[X_AXIS]);
	motorPos[Y_AXIS] = lrintf(((machinePos[X_AXIS] * axisFactors[X_AXIS]) - (machinePos[Y_AXIS] * axisFactors[Y_AXIS])) * stepsPerMm[Y_AXIS]);
	motorPos[Z_AXIS] = lrintf(machinePos[Z_AXIS] * stepsPerMm[Z_AXIS]);
	motorPos[U_AXIS] = lrintf(((machinePos[U_AXIS] * axisFactors[U_AXIS]) + (machinePos[V_AXIS] * axisFactors[V_AXIS])) * stepsPerMm[U_AXIS]);
	motorPos[V_AXIS] = lrintf(((machinePos[U_AXIS] * axisFactors[U_AXIS]) - (machinePos[V_AXIS] * axisFactors[V_AXIS])) * stepsPerMm[V_AXIS]);

	for (size_t axis = CoreXYUV_AXES; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreXYUVKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	// Convert the main axes
	const float xyStepsMm = stepsPerMm[X_AXIS] * stepsPerMm[Y_AXIS];
	const float uvStepsMm = stepsPerMm[U_AXIS] * stepsPerMm[V_AXIS];
	machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) + (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[X_AXIS] * xyStepsMm);
	machinePos[Y_AXIS] = ((motorPos[X_AXIS] * stepsPerMm[Y_AXIS]) - (motorPos[Y_AXIS] * stepsPerMm[X_AXIS]))
								/(2 * axisFactors[Y_AXIS] * xyStepsMm);
	machinePos[U_AXIS] = ((motorPos[U_AXIS] * stepsPerMm[V_AXIS]) + (motorPos[V_AXIS] * stepsPerMm[U_AXIS]))
								/(2 * axisFactors[V_AXIS] * uvStepsMm);
	machinePos[V_AXIS] = ((motorPos[U_AXIS] * stepsPerMm[V_AXIS]) - (motorPos[V_AXIS] * stepsPerMm[U_AXIS]))
								/(2 * axisFactors[V_AXIS] * uvStepsMm);

	machinePos[Z_AXIS] = motorPos[Z_AXIS]/stepsPerMm[Z_AXIS];

	// Convert any additional axes
	for (size_t drive = CoreXYUV_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified endstop axis uses shared motors.
// Used to determine whether to abort the whole move or just one motor when an endstop switch is triggered.
bool CoreXYUVKinematics::DriveIsShared(size_t drive) const
{
	return drive == X_AXIS || drive == Y_AXIS || drive == U_AXIS || drive == V_AXIS;
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds along individual Cartesian axes have already been limited before this is called.
void CoreXYUVKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const
{
	const float vecX = normalisedDirectionVector[0];
	const float vecY = normalisedDirectionVector[1];

	// Limit the XY motor accelerations
	const float vecMaxXY = max<float>(fabs(vecX + vecY), fabs(vecX - vecY));		// pick the case for the motor that is working hardest
	if (vecMaxXY > 0.01)															// avoid division by zero or near-zero
	{
		const Platform& platform = reprap.GetPlatform();
		const float aX = platform.Acceleration(0);
		const float aY = platform.Acceleration(1);
		const float vX = platform.MaxFeedrate(0);
		const float vY = platform.MaxFeedrate(1);
		const float aMax = (fabs(vecX) + fabs(vecY)) * aX * aY/(vecMaxXY * (fabs(vecX) * aY + fabs(vecY) * aX));
		const float vMax = (fabs(vecX) + fabs(vecY)) * vX * vY/(vecMaxXY * (fabs(vecX) * vY + fabs(vecY) * vX));
		dda.LimitSpeedAndAcceleration(vMax, aMax);
	}

	// Limit the UV motor accelerations
	const float vecU = normalisedDirectionVector[3];
	const float vecV = normalisedDirectionVector[4];
	const float vecMaxUV = max<float>(fabs(vecU + vecV), fabs(vecU - vecV));		// pick the case for the motor that is working hardest
	if (vecMaxUV > 0.01)															// avoid division by zero or near-zero
	{
		const Platform& platform = reprap.GetPlatform();
		const float aU = platform.Acceleration(3);
		const float aV = platform.Acceleration(4);
		const float vU = platform.MaxFeedrate(3);
		const float vV = platform.MaxFeedrate(4);
		const float aMax = (fabs(vecU) + fabs(vecV)) * aU * aV/(vecMaxUV * (fabs(vecU) * aV + fabs(vecV) * aU));
		const float vMax = (fabs(vecU) + fabs(vecV)) * vU * vV/(vecMaxUV * (fabs(vecU) * vV + fabs(vecV) * vU));
		dda.LimitSpeedAndAcceleration(vMax, aMax);
	}
}

// End
