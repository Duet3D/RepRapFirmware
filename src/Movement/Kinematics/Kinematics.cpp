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

const char *Kinematics::HomeAllFileName = "homeall.g";
const char * const Kinematics::StandardHomingFileNames[] = AXES_("homex.g", "homey.g", "homez.g", "homeu.g", "homev.g", "homew.g", "homea.g", "homeb.g", "homec.g");

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
bool Kinematics::LimitPosition(float coords[], size_t numVisibleAxes, AxesBitmap axesHomed) const
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

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
// This default is suitable for most kinematics.
const char* Kinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap& alreadyHomed, size_t numVisibleAxes, AxesBitmap& mustHomeFirst) const
{
	const AxesBitmap allAxes = LowestNBits<AxesBitmap>(numVisibleAxes);
	if ((toBeHomed & allAxes) == allAxes)
	{
		return HomeAllFileName;
	}

	// If Z homing is done using a Z probe then X and Y must be homed before Z
	const bool homeZLast = (IsBitSet(toBeHomed, Z_AXIS) && reprap.GetPlatform().HomingZWithProbe());
	const AxesBitmap homeFirst = AxesToHomeBeforeProbing();

	// Return the homing file for the lowest axis that we have been asked to home
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (IsBitSet(toBeHomed, axis) && (axis != Z_AXIS || !homeZLast || (alreadyHomed & homeFirst) == homeFirst))
		{
			return StandardHomingFileNames[axis];
		}
	}

	// Error, we can't home any axes that we were asked to home. It can only be because we can't home the Z axis.
	mustHomeFirst = homeFirst & ~alreadyHomed;
	return nullptr;
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

/*static*/ void Kinematics::PrintMatrix(const char* s, const MathMatrix<floatc_t>& m, size_t maxRows, size_t maxCols)
{
	debugPrintf("%s\n", s);
	if (maxRows == 0)
	{
		maxRows = m.rows();
	}
	if (maxCols == 0)
	{
		maxCols = m.cols();
	}

	for (size_t i = 0; i < maxRows; ++i)
	{
		for (size_t j = 0; j < maxCols; ++j)
		{
			debugPrintf("%7.4f%c", m(i, j), (j == maxCols - 1) ? '\n' : ' ');
		}
	}
}

/*static*/ void Kinematics::PrintVector(const char *s, const floatc_t *v, size_t numElems)
{
	debugPrintf("%s:", s);
	for (size_t i = 0; i < numElems; ++i)
	{
		debugPrintf(" %7.4f", v[i]);
	}
	debugPrintf("\n");
}

// End
