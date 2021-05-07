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
bool RoundBedKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes, bool isCoordinated) const noexcept
{
	if (axes.IsBitSet(X_AXIS) && axes.IsBitSet(Y_AXIS) && (fsquare(axesCoords[X_AXIS]) + fsquare(axesCoords[Y_AXIS]) >= printRadiusSquared))
	{
		return false;
	}
	axes.ClearBit(X_AXIS);
	axes.ClearBit(Y_AXIS);
	return Kinematics::IsReachable(axesCoords, axes, isCoordinated);
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void RoundBedKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept
{
	// Limit the speed in the XY plane to the lower of the X and Y maximum speeds, and similarly for the acceleration
	const float xyFactor = fastSqrtf(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[Y_AXIS]));
	if (xyFactor > 0.01)
	{
		const Platform& platform = reprap.GetPlatform();
		const float maxSpeed = min<float>(platform.MaxFeedrate(X_AXIS), platform.MaxFeedrate(Y_AXIS));
		const float maxAcceleration = min<float>(platform.Acceleration(X_AXIS), platform.Acceleration(Y_AXIS));
		dda.LimitSpeedAndAcceleration(maxSpeed/xyFactor, maxAcceleration/xyFactor);
	}
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap RoundBedKinematics::GetLinearAxes() const noexcept
{
	return AxesBitmap();
}

// End
