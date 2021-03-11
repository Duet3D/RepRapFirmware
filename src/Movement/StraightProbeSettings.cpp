/*
 * StraightProbeSettings.cpp
 *
 *  Created on: 4 Oct 2019
 *      Author: manuel
 */

#include "StraightProbeSettings.h"
#include <Platform/RepRap.h>

StraightProbeSettings::StraightProbeSettings() noexcept
{
	Reset();
}

void StraightProbeSettings::Reset() noexcept
{
	movingAxes = AxesBitmap();
	type = StraightProbeType::unset;
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		target[axis] = 0;
	}
}

void StraightProbeSettings::SetCoordsToTarget(float coords[MaxAxes]) const noexcept
{
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		coords[axis] = target[axis];
	}
}

// End
