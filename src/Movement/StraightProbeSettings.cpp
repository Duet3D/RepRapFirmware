/*
 * StraightProbeSettings.cpp
 *
 *  Created on: 4 Oct 2019
 *      Author: manuel
 */

#include <Movement/StraightProbeSettings.h>
#include "RepRap.h"

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(_ret) OBJECT_MODEL_FUNC_BODY(StraightProbeSettings, _ret)

constexpr ObjectModelTableEntry StraightProbeSettings::objectModelTable[] =
{
	// These entries must be in alphabetical order
	{ "movingAxes", OBJECT_MODEL_FUNC(&(self->movingAxes)), TYPE_OF(uint32_t), 0, ObjectModelEntryFlags::none },
	{ "probeToUse", OBJECT_MODEL_FUNC(&(self->probeToUse)), TYPE_OF(uint32_t), 0, ObjectModelEntryFlags::none },
	//{ "target", OBJECT_MODEL_FUNC(&(self->target)), TYPE_OF(const float *), 0, ObjectModelEntryFlags::none },
	{ "type", OBJECT_MODEL_FUNC(&(self->type)), TYPE_OF(uint32_t), 0, ObjectModelEntryFlags::none }
};

constexpr uint8_t StraightProbeSettings::objectModelTableDescriptor[] = { 1, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE(StraightProbeSettings)

#endif

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
