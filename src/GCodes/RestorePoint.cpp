/*
 * RestorePoint.cpp
 *
 *  Created on: 14 Jun 2017
 *      Author: David
 */

#include "RestorePoint.h"
#include <RepRap.h>
#include <GCodes/GCodes.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RestorePoint, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(RestorePoint, _condition,__VA_ARGS__)

constexpr ObjectModelArrayDescriptor RestorePoint::coordinatesArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetVisibleAxes(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
																			{ return ExpressionValue(((const RestorePoint*)self)->moveCoords[context.GetLastIndex()], 3); }
};

constexpr ObjectModelArrayDescriptor RestorePoint::spindleSpeedsArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return MaxSpindles; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
																			{ return ExpressionValue(((const RestorePoint*)self)->spindleSpeeds[context.GetLastIndex()], 1); }
};

constexpr ObjectModelTableEntry RestorePoint::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. LaserFilamentMonitor members
	{ "coords", 			OBJECT_MODEL_FUNC_NOSELF(&coordinatesArrayDescriptor), 								ObjectModelEntryFlags::none },
	{ "extruderPos",		OBJECT_MODEL_FUNC(self->virtualExtruderPosition, 1),	 							ObjectModelEntryFlags::none },
	{ "feedRate", 			OBJECT_MODEL_FUNC(self->feedRate, 1), 												ObjectModelEntryFlags::none },
#if SUPPORT_IOBITS
	{ "ioBits",				OBJECT_MODEL_FUNC_IF(reprap.GetGCodes().GetMachineType() != MachineType::laser,
													(int32_t)self->laserPwmOrIoBits.ioBits),					ObjectModelEntryFlags::none },
#endif
#if SUPPORT_LASER
	{ "laserPwm",			OBJECT_MODEL_FUNC_IF(reprap.GetGCodes().GetMachineType() == MachineType::laser,
													(float)self->laserPwmOrIoBits.laserPwm/65535.0, 2),			ObjectModelEntryFlags::none },
#endif
	{ "spindleSpeeds",		OBJECT_MODEL_FUNC_NOSELF(&spindleSpeedsArrayDescriptor), 							ObjectModelEntryFlags::none },
	{ "toolNumber",			OBJECT_MODEL_FUNC((int32_t)self->toolNumber),										ObjectModelEntryFlags::none },
};

constexpr uint8_t RestorePoint::objectModelTableDescriptor[] = { 1, 5 + SUPPORT_LASER + SUPPORT_IOBITS };

DEFINE_GET_OBJECT_MODEL_TABLE(RestorePoint)

#endif

RestorePoint::RestorePoint() noexcept
{
	Init();
}

void RestorePoint::Init() noexcept
{
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		moveCoords[i] = 0.0;
	}

	feedRate = DefaultFeedRate * SecondsToMinutes;
	virtualExtruderPosition = 0.0;
	filePos = noFilePosition;
	proportionDone = 0.0;
	initialUserX = initialUserY = 0.0;
	toolNumber = -1;

	for (size_t i = 0; i < MaxSpindles; ++i)
	{
		spindleSpeeds[i] = 0.0;
	}

#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif
}

// End
