/*
 * CoreBaseKinematics.cpp
 *
 *  Created on: 7 May 2017
 *      Author: David
 */

#include "CoreBaseKinematics.h"

#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/DDA.h"

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
bool CoreBaseKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) /*override*/
{
	if (mCode == 667)
	{
		bool seen = false;
		for (size_t axis = 0; axis < XYZ_AXES; ++axis)
		{
			if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
			{
				axisFactors[axis] = gb.GetFValue();
				seen = true;
			}
		}
		if (!seen && !gb.Seen('S'))
		{
			reply.printf("Kinematics is %s with axis factors", GetName());
			for (size_t axis = 0; axis < XYZ_AXES; ++axis)
			{
				reply.catf(" %c:%.3f", reprap.GetGCodes().GetAxisLetters()[axis], (double)axisFactors[axis]);
			}
		}
		return seen;
	}
	else
	{
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool CoreBaseKinematics::QueryTerminateHomingMove(size_t axis) const
{
	return DriveIsShared(axis);
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void CoreBaseKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
	if (DriveIsShared(axis))
	{
		float tempCoordinates[MaxAxes];
		const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t axis = 0; axis < numTotalAxes; ++axis)
		{
			tempCoordinates[axis] = dda.GetEndCoordinate(axis, false);
		}
		tempCoordinates[axis] = hitPoint;
		dda.SetPositions(tempCoordinates, numTotalAxes);
	}
	else
	{
		dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
	}
}

// End
