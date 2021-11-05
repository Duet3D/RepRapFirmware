/*
 * RoundBedKinematics.cpp
 *
 *  Created on: 16 Feb 2021
 *      Author: manuel
 */

#include "RoundBedKinematics.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>

RoundBedKinematics::RoundBedKinematics(KinematicsType t, SegmentationType segType) noexcept
	: Kinematics(t, segType), printRadiusSquared(0.0)
{
}


// Return true if the specified XY position is reachable by the print head reference point.
bool RoundBedKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept
{
	if (axes.IsBitSet(X_AXIS) && axes.IsBitSet(Y_AXIS) && (fsquare(axesCoords[X_AXIS]) + fsquare(axesCoords[Y_AXIS]) >= printRadiusSquared))
	{
		return false;
	}
	axes.ClearBit(X_AXIS);
	axes.ClearBit(Y_AXIS);
	return Kinematics::IsReachable(axesCoords, axes);
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap RoundBedKinematics::GetLinearAxes() const noexcept
{
	return AxesBitmap();
}

// End
