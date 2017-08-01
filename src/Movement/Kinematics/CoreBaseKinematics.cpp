/*
 * CoreBaseKinematics.cpp
 *
 *  Created on: 7 May 2017
 *      Author: David
 */

#include "CoreBaseKinematics.h"
#include "GCodes/GCodes.h"

CoreBaseKinematics::CoreBaseKinematics(KinematicsType t) : ZLeadscrewKinematics(t)
{
	for (float& af : axisFactors)
	{
		af = 1.0;
	}
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
// This function is used for CoreXY and CoreXZ kinematics, but it overridden for CoreXYU kinematics
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
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}
}

// End
