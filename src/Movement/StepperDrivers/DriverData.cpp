/*
 * DriverData.cpp
 *
 *  Created on: 25 Sept 2023
 *      Author: David
 */

#include "DriverData.h"

#define OBJECT_MODEL_FUNC(...)							OBJECT_MODEL_FUNC_BODY(DriverData, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)			OBJECT_MODEL_FUNC_IF_BODY(DriverData, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry DriverData::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. DriverData members
	{ "closedLoop",			OBJECT_MODEL_FUNC_IF(self->haveClosedLoopData, self, 1), 	ObjectModelEntryFlags::liveNotPanelDue },
	{ "status",				OBJECT_MODEL_FUNC(self->status.all), 						ObjectModelEntryFlags::liveNotPanelDue },

	// 1. closedLoop members
	{ "currentFraction",	OBJECT_MODEL_FUNC(self, 2), 								ObjectModelEntryFlags::liveNotPanelDue },
	{ "positionError",		OBJECT_MODEL_FUNC(self, 3), 								ObjectModelEntryFlags::liveNotPanelDue },

	// 2. closedLoop.currentFraction members
	{ "avg",				OBJECT_MODEL_FUNC((float)self->averageCurrentFraction, 2), 	ObjectModelEntryFlags::liveNotPanelDue },
	{ "max",				OBJECT_MODEL_FUNC((float)self->maxCurrentFraction, 2), 		ObjectModelEntryFlags::liveNotPanelDue },

	// 3. closedLoop.positionError members
	{ "max",				OBJECT_MODEL_FUNC((float)self->maxAbsPositionError, 2), 	ObjectModelEntryFlags::liveNotPanelDue },
	{ "rms",				OBJECT_MODEL_FUNC((float)self->rmsPositionError, 2), 		ObjectModelEntryFlags::liveNotPanelDue },
};

constexpr uint8_t DriverData::objectModelTableDescriptor[] = { 4, 2, 2, 2, 2 };

DEFINE_GET_OBJECT_MODEL_TABLE(DriverData)

// End
