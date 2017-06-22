/*
 * Kinematics.cpp
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#include <Movement/Kinematics/LinearDeltaKinematics.h>
#include "Kinematics.h"
#include "CartesianKinematics.h"
#include "CoreXYKinematics.h"
#include "CoreXZKinematics.h"
#include "ScaraKinematics.h"
#include "CoreXYUKinematics.h"
#include "RepRap.h"
#include "Platform.h"

// Constructor for non-segmented kinematics
Kinematics::Kinematics(KinematicsType t)
	: useSegmentation(false), useRawG0(true), type(t)
{
}

// Constructor for segmented kinematics
Kinematics::Kinematics(KinematicsType t, float segsPerSecond, float minSegLength, bool doUseRawG0)
	: segmentsPerSecond(segsPerSecond), minSegmentLength(minSegLength), useSegmentation(true), useRawG0(doUseRawG0), type(t)
{
}

// Set or report the parameters from a M665, M666 or M669 command
// This is the fallback function for when the derived class doesn't use the specified M-code
bool Kinematics::Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error)
{
	reply.printf("M%u parameters do not apply to %s kinematics", mCode, GetName());
	error = true;
	return false;
}

// Return true if the specified XY position is reachable by the print head reference point.
// This default implementation assumes a rectangular reachable area, so it just uses the bed dimensions give in the M280 command.
bool Kinematics::IsReachable(float x, float y) const
{
	const Platform& platform = reprap.GetPlatform();
	return x >= platform.AxisMinimum(X_AXIS) && y >= platform.AxisMinimum(Y_AXIS) && x <= platform.AxisMaximum(X_AXIS) && y <= platform.AxisMaximum(Y_AXIS);
}

// Limit the Cartesian position that the user wants to move to
// This default implementation just applies the rectangular limits set up by M208 to those axes that have been homed.
bool Kinematics::LimitPosition(float coords[], size_t numVisibleAxes, uint16_t axesHomed) const
{
	const Platform& platform = reprap.GetPlatform();
	bool limited = false;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if ((axesHomed & (1 << axis)) != 0)
		{
			float& f = coords[axis];
			if (f < platform.AxisMinimum(axis))
			{
				f = platform.AxisMinimum(axis);
				limited = true;
			}
			else if (f > platform.AxisMaximum(axis))
			{
				f = platform.AxisMaximum(axis);
				limited = true;
			}
		}
	}
	return limited;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
// This default is suitable for Cartesian and CoreXY printers.
void Kinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
}

/*static*/ Kinematics *Kinematics::Create(KinematicsType k)
{
	switch (k)
	{
	default:
		return nullptr;
	case KinematicsType::cartesian:
		return new CartesianKinematics();
	case KinematicsType::linearDelta:
		return new LinearDeltaKinematics();
	case KinematicsType::coreXY:
		return new CoreXYKinematics();
	case KinematicsType::coreXZ:
		return new CoreXZKinematics();
	case KinematicsType::scara:
		return new ScaraKinematics();
	case KinematicsType::coreXYU:
		return new CoreXYUKinematics();
	}
}

// End
