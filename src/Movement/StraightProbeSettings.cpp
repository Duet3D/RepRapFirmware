/*
 * StraightProbeSettings.cpp
 *
 *  Created on: 4 Oct 2019
 *      Author: manuel
 */

#include "StraightProbeSettings.h"
#include "RepRap.h"

StraightProbeSettings::StraightProbeSettings() {
	Reset();
}

void StraightProbeSettings::Reset() {
	movingAxes = (AxesBitmap) 0;
	type = StraightProbeType::unset;
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		target[axis] = 0;
	}
}

void StraightProbeSettings::SetTarget(const float coords[MaxAxes]) {
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		target[axis] = coords[axis];
	}
}

void StraightProbeSettings::SetCoordsToTarget(float coords[MaxAxes]) const {
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		coords[axis] = target[axis];
	}
}

// End
