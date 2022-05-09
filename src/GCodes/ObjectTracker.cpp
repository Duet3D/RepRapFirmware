/*
 * ObjectCancellation.cpp
 *
 *  Created on: 24 Mar 2020
 *      Author: David
 */

#include "ObjectTracker.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include "GCodes.h"

// Object model tables and functions for class ObjectTracker

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(ObjectTracker, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(ObjectTracker, _condition,__VA_ARGS__)

constexpr ObjectModelArrayDescriptor ObjectTracker::objectsArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return min<size_t>(((const ObjectTracker*)self)->numObjects, MaxTrackedObjects); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(self, 1); }
};

constexpr ObjectModelArrayDescriptor ObjectTracker::xArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ((const ObjectTracker*)self)->GetXCoordinate(context); }
};

constexpr ObjectModelArrayDescriptor ObjectTracker::yArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ((const ObjectTracker*)self)->GetYCoordinate(context); }
};

constexpr ObjectModelTableEntry ObjectTracker::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. BuildObjects root
	{ "currentObject",	OBJECT_MODEL_FUNC_NOSELF((int32_t)reprap.GetGCodes().GetPrimaryMovementState().currentObjectNumber), ObjectModelEntryFlags::live },
	{ "m486Names",		OBJECT_MODEL_FUNC(self->usingM486Naming),							ObjectModelEntryFlags::none },
	{ "m486Numbers",	OBJECT_MODEL_FUNC(self->usingM486Labelling),						ObjectModelEntryFlags::none },
	{ "objects",		OBJECT_MODEL_FUNC_NOSELF(&objectsArrayDescriptor),					ObjectModelEntryFlags::none },

	// 1. ObjectDirectoryEntry root
	{ "cancelled",	OBJECT_MODEL_FUNC(self->IsCancelled(context.GetLastIndex())),			ObjectModelEntryFlags::none },
	{ "name",		OBJECT_MODEL_FUNC(self->objectDirectory[context.GetLastIndex()].name.IncreaseRefCount()),	ObjectModelEntryFlags::none },
	{ "x",			OBJECT_MODEL_FUNC_NOSELF(&xArrayDescriptor),							ObjectModelEntryFlags::none },
	{ "y",			OBJECT_MODEL_FUNC_NOSELF(&yArrayDescriptor),							ObjectModelEntryFlags::none },
};

constexpr uint8_t ObjectTracker::objectModelTableDescriptor[] =
{
	2,		// number of sub-tables
	4,
	4
};

DEFINE_GET_OBJECT_MODEL_TABLE(ObjectTracker)

void ObjectTracker::Init() noexcept
{
	objectsCancelled.Clear();
	numObjects = 0;
	usingM486Labelling = false;
	// Clear out all object names in case of late object model requests
	for (ObjectDirectoryEntry& ode : objectDirectory)
	{
		ode.Init("");
	}
	usingM486Naming = false;
}

void ObjectTracker::HandleM486T(unsigned int p_numObjects) noexcept
{
	numObjects = p_numObjects;
	objectsCancelled.Clear();							// assume this command is only used at the start of a print
	reprap.JobUpdated();
}

void ObjectTracker::UseM486Labelling() noexcept
{
	if (!usingM486Labelling)
	{
		usingM486Labelling = true;
		reprap.JobUpdated();
	}
}

void ObjectTracker::SetM486Label(unsigned int objectNumber, const char *_ecv_array objectName) noexcept
{
	usingM486Naming = true;

	if (objectNumber >= numObjects)						// if this is a new object
	{
		CreateObject(objectNumber, objectName);
	}
	else if (objectNumber < MaxTrackedObjects && objectName[0] != 0 && strcmp(objectName, objectDirectory[objectNumber].name.Get().Ptr()) != 0)
	{
		objectDirectory[objectNumber].SetName(objectName);
		reprap.JobUpdated();
	}
}

// Cancel an object, returning true if it was not already cancelled
bool ObjectTracker::CancelObject(unsigned int objectNumber) noexcept
{
	if (objectNumber < objectsCancelled.MaxBits() && !objectsCancelled.IsBitSet(objectNumber))
	{
		objectsCancelled.SetBit(objectNumber);
		reprap.JobUpdated();
		return true;
	}
	return false;
}

// Resume an object, returning true if it was not already enabled
bool ObjectTracker::ResumeObject(unsigned int objectNumber) noexcept
{
	if (objectNumber < objectsCancelled.MaxBits() && objectsCancelled.IsBitSet(objectNumber))
	{
		objectsCancelled.ClearBit(objectNumber);
		reprap.JobUpdated();
		return true;
	}
	return false;
}

// Add the object number if it isn't already known, return true if it has been cancelled
bool ObjectTracker::CheckObject(int objectNumber) noexcept
{
	if (objectNumber >= (int)numObjects)
	{
		numObjects = objectNumber + 1;
	}
	return objectNumber >= 0 && objectNumber < (int)objectsCancelled.MaxBits() && objectsCancelled.IsBitSet(objectNumber);
}

// List the objects on the build plate
void ObjectTracker::ListObjects(OutputBuffer *buf) noexcept
{
	if (numObjects == 0)
	{
		buf->copy("No known objects on build plate");
	}
	else
	{
		for (size_t i = 0; i < min<unsigned int>(numObjects, MaxTrackedObjects); ++i)
		{
			const ObjectDirectoryEntry& obj = objectDirectory[i];
			buf->lcatf("%2u%s: X %d to %dmm, Y %d to %dmm, %s",
						i,
						(objectsCancelled.IsBitSet(i) ? " (cancelled)" : ""),
						(int)obj.x[0], (int)obj.x[1],
						(int)obj.y[0], (int)obj.y[1],
						obj.name.Get().Ptr());
		}
		if (numObjects > MaxTrackedObjects)
		{
			buf->lcatf("%u more objects", numObjects - MaxTrackedObjects);
		}
	}
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Write the object details to file, returning true if successful
bool ObjectTracker::WriteObjectDirectory(FileStore *f) const noexcept
{
	bool ok = true;

	// Write the object list
	for (size_t i = 0; ok && i < min<unsigned int>(numObjects, MaxTrackedObjects); ++i)
	{
		String<StringLength100> buf;
		buf.printf("M486 S%u A\"%s\"\n", i, objectDirectory[i].name.Get().Ptr());
		ok = f->Write(buf.c_str());
	}

	if (ok)
	{
		// Write which objects have been cancelled
		ok = objectsCancelled.IterateWhile([f](unsigned int index, bool first) -> bool
											{
												String<StringLength20> buf;
												buf.printf("M486 P%u\n", index);
												return f->Write(buf.c_str());
											});
	}

	// Write the current object
	if (ok)
	{
		String<StringLength20> buf;
		buf.printf("M486 S%d\n", reprap.GetGCodes().GetPrimaryMovementState().currentObjectNumber);
		ok = f->Write(buf.c_str());
	}

	return ok;
}

#endif

// Create a new entry in the object directory
void ObjectDirectoryEntry::Init(const char *label) noexcept
{
	name.Assign(label);
	x[0] = x[1] = y[0] = y[1] = std::numeric_limits<int16_t>::min();
}

// Update the min and max object coordinates to include the coordinates passed, returning true if anything was changed
bool ObjectDirectoryEntry::UpdateObjectCoordinates(const float coords[], AxesBitmap axes) noexcept
{
	bool updated = false;
	if (axes.IsBitSet(X_AXIS))
	{
		const int16_t xVal = lrintf(coords[X_AXIS]);
		if (x[1] == std::numeric_limits<int16_t>::min())
		{
			x[0] = x[1] = xVal;
			updated = true;
		}
		else if (xVal < x[0])
		{
			x[0] = xVal;
			updated = true;
		}
		else if (xVal > x[1])
		{
			x[1] = xVal;
			updated = true;
		}
	}

	if (axes.IsBitSet(Y_AXIS))
	{
		const int16_t yVal = lrintf(coords[Y_AXIS]);
		if (y[1] == std::numeric_limits<int16_t>::min())
		{
			y[0] = y[1] = yVal;
			updated = true;
		}
		else if (yVal < y[0])
		{
			y[0] = yVal;
			updated = true;
		}
		else if (yVal > y[1])
		{
			y[1] = yVal;
			updated = true;
		}
	}

	return updated;
}

void ObjectDirectoryEntry::SetName(const char *label) noexcept
{
	name.Assign(label);
}

// Update the min and max object coordinates to include the coordinates passed
// We could pass both the start and end coordinates of the printing move, however it is simpler just to pass the end coordinates.
// This is OK because it is very unlikely that there won't be a subsequent extruding move that ends close to the original one.
void ObjectTracker::UpdateObjectCoordinates(int objectNumber, const float coords[], AxesBitmap axes) noexcept
{
	if (objectNumber >= 0 && objectNumber < (int)MaxTrackedObjects)
	{
		if (objectDirectory[objectNumber].UpdateObjectCoordinates(coords, axes))
		{
			reprap.JobUpdated();
		}
	}
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
		objectDirectory[number].SetName(label);
		reprap.JobUpdated();
	}
}

// This is called when we have found an object label in a comment
size_t ObjectTracker::GetObjectNumber(const char *_ecv_array label) noexcept
{
	for (size_t i = 0; i < min<size_t>(numObjects, MaxTrackedObjects); ++i)
	{
		if (strcmp(objectDirectory[i].name.Get().Ptr(), label) == 0)
		{
			return i;
		}
	}

	// The object was not found, so add it
	if (numObjects < MaxTrackedObjects)
	{
		const int newObjectNumber = numObjects;
		CreateObject(newObjectNumber, label);
		return newObjectNumber;
	}
	else
	{
		// Here if the new object won't fit in the directory
		return MaxTrackedObjects;
	}
}

ExpressionValue ObjectTracker::GetXCoordinate(const ObjectExplorationContext& context) const noexcept
{
	const int16_t val = objectDirectory[context.GetIndex(1)].x[context.GetIndex(0)];
	return (val == std::numeric_limits<int16_t>::min()) ? ExpressionValue(nullptr) : ExpressionValue((int32_t)val);
}

ExpressionValue ObjectTracker::GetYCoordinate(const ObjectExplorationContext& context) const noexcept
{
	const int16_t val = objectDirectory[context.GetIndex(1)].y[context.GetIndex(0)];
	return (val == std::numeric_limits<int16_t>::min()) ? ExpressionValue(nullptr) : ExpressionValue((int32_t)val);
}

// End
