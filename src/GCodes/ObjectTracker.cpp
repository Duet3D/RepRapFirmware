/*
 * ObjectCancellation.cpp
 *
 *  Created on: 24 Mar 2020
 *      Author: David
 */

#include "ObjectTracker.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <RepRap.h>
#include "GCodes.h"

#if TRACK_OBJECT_NAMES && SUPPORT_OBJECT_MODEL

// Object model tables and functions for class ObjectDirectoryEntry
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC_ODE(...) OBJECT_MODEL_FUNC_BODY(ObjectDirectoryEntry, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_ODE_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(ObjectDirectoryEntry, _condition,__VA_ARGS__)

constexpr ObjectModelArrayDescriptor ObjectDirectoryEntry::xArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const ObjectDirectoryEntry*)self)->x[context.GetLastIndex()]); }
};

constexpr ObjectModelArrayDescriptor ObjectDirectoryEntry::yArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const ObjectDirectoryEntry*)self)->y[context.GetLastIndex()]); }
};

constexpr ObjectModelTableEntry ObjectDirectoryEntry::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. ObjectDirectoryEntry root
	{ "cancelled",	OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetBuildObjects()->IsCancelled(context.GetLastIndex())),	ObjectModelEntryFlags::none },
	{ "name",		OBJECT_MODEL_FUNC_ODE(self->name),																		ObjectModelEntryFlags::none },
	{ "x",			OBJECT_MODEL_FUNC_NOSELF(&xArrayDescriptor),															ObjectModelEntryFlags::none },
	{ "y",			OBJECT_MODEL_FUNC_NOSELF(&yArrayDescriptor),															ObjectModelEntryFlags::none },
};

constexpr uint8_t ObjectDirectoryEntry::objectModelTableDescriptor[] =
{
	1,		// number of sub-tables
	4
};

DEFINE_GET_OBJECT_MODEL_TABLE(ObjectDirectoryEntry)

// Object model tables and functions for class ObjectTracker

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC_OT(...) OBJECT_MODEL_FUNC_BODY(ObjectTracker, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_OT_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(ObjectTracker, _condition,__VA_ARGS__)

constexpr ObjectModelArrayDescriptor ObjectTracker::objectsArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const ObjectTracker*)self)->numObjects; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept
									-> ExpressionValue { return ExpressionValue(&(((const ObjectTracker*)self)->objectDirectory[context.GetLastIndex()]), 0); }
};

constexpr ObjectModelTableEntry ObjectTracker::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. BuildObjects root
	{ "currentObject",	OBJECT_MODEL_FUNC_OT((int32_t)self->currentObjectNumber),			ObjectModelEntryFlags::live },
	{ "m486Names",		OBJECT_MODEL_FUNC_OT(self->usingM486Naming),						ObjectModelEntryFlags::none },
	{ "m486Numbers",	OBJECT_MODEL_FUNC_OT(self->usingM486Labelling),						ObjectModelEntryFlags::none },
	{ "objects",		OBJECT_MODEL_FUNC_NOSELF(&objectsArrayDescriptor),					ObjectModelEntryFlags::none },
};

constexpr uint8_t ObjectTracker::objectModelTableDescriptor[] =
{
	1,		// number of sub-tables
	4
};

DEFINE_GET_OBJECT_MODEL_TABLE(ObjectTracker)

#endif

void ObjectTracker::Init() noexcept
{
	objectsCancelled.Clear();
	currentObjectNumber = -1;
	numObjects = 0;
	currentObjectCancelled = printingJustResumed = usingM486Labelling = false;
#if TRACK_OBJECT_NAMES
	// Clear out all object names in case of late object model requests
	for (size_t i = 0; i < MaxTrackedObjects; ++i)
	{
		objectDirectory[i].Init("");
	}
	objectNames.Reset();
	usingM486Naming = false;
#endif
}

void ObjectTracker::ChangeToObject(GCodeBuffer& gb, int i) noexcept
{
	currentObjectNumber = i;
	if (currentObjectNumber >= (int)numObjects)
	{
		numObjects = currentObjectNumber + 1;
	}
	const bool cancelCurrentObject = currentObjectNumber >= 0 && currentObjectNumber < (int)objectsCancelled.MaxBits() && objectsCancelled.IsBitSet(currentObjectNumber);
	if (cancelCurrentObject && !currentObjectCancelled)
	{
		StopPrinting(gb);
	}
	else if (!cancelCurrentObject && currentObjectCancelled)
	{
		ResumePrinting(gb);
	}
}

GCodeResult ObjectTracker::HandleM486(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	if (gb.Seen('T'))
	{
		// Specify how many objects. May be useful for a user interface.
		numObjects = gb.GetUIValue();
		objectsCancelled.Clear();						// assume this command is only used at the start of a print
		reprap.JobUpdated();
	}

	if (gb.Seen('S'))
	{
		// Specify which object we are about to print
		if (!usingM486Labelling)
		{
			usingM486Labelling = true;
			reprap.JobUpdated();
		}
		const int num = gb.GetIValue();
#if TRACK_OBJECT_NAMES
		if (num >= 0 && num < (int)MaxTrackedObjects && gb.Seen('A'))
		{
			String<StringLength50> objectName;
			gb.GetQuotedString(objectName.GetRef());
			usingM486Naming = true;

			if (num >= (int)numObjects)					// if this is a new object
			{
				CreateObject(num, objectName.c_str());
			}
			else if (strcmp(objectName.c_str(), objectDirectory[num].name) != 0)
			{
				objectNames.FinishedUsing(objectDirectory[num].name);
				SetObjectName(num, objectName.c_str());
				reprap.JobUpdated();
			}
		}
#endif
		ChangeToObject(gb, num);
	}

	if (gb.Seen('P'))
	{
		// Cancel an object
		const int objectToCancel = gb.GetIValue();
		if (objectToCancel >= 0 && objectToCancel < (int)objectsCancelled.MaxBits())
		{
			objectsCancelled.SetBit(objectToCancel);
			if (objectToCancel == currentObjectNumber && !currentObjectCancelled)
			{
				StopPrinting(gb);
			}
			reprap.JobUpdated();
		}
	}

	if (gb.Seen('U'))
	{
		// Resume an object
		const int objectToResume = gb.GetIValue();
		if (objectToResume >= 0 && objectToResume < (int)objectsCancelled.MaxBits())
		{
			objectsCancelled.ClearBit(objectToResume);
			if (objectToResume == currentObjectNumber && currentObjectCancelled)
			{
				ResumePrinting(gb);
			}
			reprap.JobUpdated();
		}
	}

	if (gb.Seen('C') && currentObjectNumber >= 0 && currentObjectNumber < (int)objectsCancelled.MaxBits())
	{
		// Cancel current object
		objectsCancelled.SetBit(currentObjectNumber);
		if (!currentObjectCancelled)
		{
			StopPrinting(gb);
		}
		reprap.JobUpdated();
	}

	return GCodeResult::ok;
}

// We are currently printing, but we must now stop because the current object is cancelled
void ObjectTracker::StopPrinting(GCodeBuffer& gb) noexcept
{
	currentObjectCancelled = true;
	virtualToolNumber = reprap.GetCurrentToolNumber();
}

// We are currently not printing because the current object was cancelled, but now we need to print again
void ObjectTracker::ResumePrinting(GCodeBuffer& gb) noexcept
{
	currentObjectCancelled = false;
	printingJustResumed = true;
	reprap.GetGCodes().SavePosition(rp, gb);					// save the position we should be at for the start of the next move
	if (reprap.GetCurrentToolNumber() != virtualToolNumber)		// if the wrong tool is loaded
	{
		reprap.GetGCodes().StartToolChange(gb, virtualToolNumber, DefaultToolChangeParam);
	}
}

#if TRACK_OBJECT_NAMES

// Create a new entry in the object directory
void ObjectDirectoryEntry::Init(const char *label) noexcept
{
	name = label;
	x[0] = x[1] = y[0] = y[1] = std::numeric_limits<float>::quiet_NaN();
}

// Update the min and max object coordinates to include the coordinates passed, returning true if anything was changed
bool ObjectDirectoryEntry::UpdateObjectCoordinates(const float coords[]) noexcept
{
	bool updated = false;
	if (isnan(x[0]))
	{
		x[0] = x[1] = coords[X_AXIS];
		y[0] = y[1] = coords[Y_AXIS];
		updated = true;
	}
	else
	{
		if (coords[X_AXIS] < x[0])
		{
			x[0] = coords[X_AXIS];
			updated = true;
		}
		else if (coords[X_AXIS] > x[1])
		{
			x[1] = coords[X_AXIS];
			updated = true;
		}

		if (coords[Y_AXIS] < y[0])
		{
			y[0] = coords[Y_AXIS];
			updated = true;
		}
		else if (coords[Y_AXIS] > y[1])
		{
			y[1] = coords[Y_AXIS];
			updated = true;
		}
	}
	return updated;
}

// Update the min and max object coordinates to include the coordinates passed
// We could pass both the start and end coordinates of the printing move, however it is simpler just to pass the end coordinates.
// This is OK because it is very unlikely that there won't be a subsequent extruding move that ends close to the original one.
void ObjectTracker::UpdateObjectCoordinates(const float coords[]) noexcept
{
	if (currentObjectNumber >= 0 && currentObjectNumber < (int)numObjects)
	{
		if (objectDirectory[currentObjectNumber].UpdateObjectCoordinates(coords))
		{
			reprap.JobUpdated();
		}
	}
}

void ObjectTracker::SetObjectName(unsigned int number, const char *label) noexcept
{
	bool err = objectNames.GetRef().copy(label);
	const char * const name = objectNames.LatestCStr();
	err = err || objectNames.Fix();
	objectDirectory[number].name = ((err) ? "" : name);
}

// This is called when we need to create a new named object
void ObjectTracker::CreateObject(unsigned int number, const char *label) noexcept
{
	if (number < MaxTrackedObjects)
	{
		while (numObjects <= number)
		{
			objectDirectory[numObjects].Init("");
			++numObjects;
		}
		SetObjectName(number, label);
		reprap.JobUpdated();
	}
}

// This is called when we have found an object label in a comment
void ObjectTracker::StartObject(GCodeBuffer& gb, const char *label) noexcept
{
	if (!usingM486Naming)
	{
		for (size_t i = 0; i < numObjects; ++i)
		{
			if (strcmp(objectDirectory[i].name, label) == 0)
			{
				ChangeToObject(gb, i);
				return;
			}
		}

		// The object was not found, so add it
		if (numObjects < MaxTrackedObjects)
		{
			const int newObjectNumber = numObjects;
			CreateObject(newObjectNumber, label);
			ChangeToObject(gb, newObjectNumber);
		}
		else
		{
			// Here if the new object won't fit in the directory
			ChangeToObject(gb, -1);
		}
	}
}

// This is called when we have found a "stop printing object" comment
void ObjectTracker::StopObject(GCodeBuffer& gb) noexcept
{
	if (!usingM486Naming)
	{
		ChangeToObject(gb, -1);
	}
}

#endif

// End
