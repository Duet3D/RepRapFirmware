/*
 * CoreBaseKinematics.cpp
 *
 *  Created on: 7 May 2017
 *      Author: David
 */

#include "CoreBaseKinematics.h"
#include "GCodes/GCodes.h"

CoreBaseKinematics::CoreBaseKinematics(KinematicsType t) : Kinematics(t)
{
	for (size_t axis = 0; axis < XYZ_AXES; ++axis)
	{
		axisFactors[axis] = 1.0;
	}
}

// Convert Cartesian coordinates to motor coordinates
bool CoreBaseKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[]) const
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = (int32_t)roundf(MotorFactor(axis, machinePos) * stepsPerMm[axis]);
	}
	return true;
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool CoreBaseKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) /*override*/
{
	if (mCode == 667)
	{
		bool seen = false;
		for (size_t axis = 0; axis < XYZ_AXES; ++axis)
		{
			if (gb.Seen(GCodes::axisLetters[axis]))
			{
				axisFactors[axis] = gb.GetFValue();
				seen = true;
			}
		}
		if (!seen && !gb.Seen('S'))
		{
			reply.printf("Printer mode is %s with axis factors", GetName());
			for (size_t axis = 0; axis < XYZ_AXES; ++axis)
			{
				reply.catf(" %c:%f", GCodes::axisLetters[axis], axisFactors[axis]);
			}
		}
		return seen;
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

// End
