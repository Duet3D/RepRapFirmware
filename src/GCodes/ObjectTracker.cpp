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

#if TRACK_OBJECT_NAMES

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
	{ "currentObject",	OBJECT_MODEL_FUNC((int32_t)self->currentObjectNumber),				ObjectModelEntryFlags::live },
	{ "m486Names",		OBJECT_MODEL_FUNC(self->usingM486Naming),							ObjectModelEntryFlags::none },
	{ "m486Numbers",	OBJECT_MODEL_FUNC(self->usingM486Labelling),						ObjectModelEntryFlags::none },
	{ "objects",		OBJECT_MODEL_FUNC_NOSELF(&objectsArrayDescriptor),					ObjectModelEntryFlags::none },

#if TRACK_OBJECT_NAMES
	// 1. ObjectDirectoryEntry root
	{ "cancelled",	OBJECT_MODEL_FUNC(self->IsCancelled(context.GetLastIndex())),			ObjectModelEntryFlags::none },
	{ "name",		OBJECT_MODEL_FUNC(self->objectDirectory[context.GetLastIndex()].name.IncreaseRefCount()),	ObjectModelEntryFlags::none },
	{ "x",			OBJECT_MODEL_FUNC_NOSELF(&xArrayDescriptor),							ObjectModelEntryFlags::none },
	{ "y",			OBJECT_MODEL_FUNC_NOSELF(&yArrayDescriptor),							ObjectModelEntryFlags::none },
#endif
};

constexpr uint8_t ObjectTracker::objectModelTableDescriptor[] =
{
	1 + TRACK_OBJECT_NAMES,		// number of sub-tables
	4,
#if TRACK_OBJECT_NAMES
	4
#endif
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
	for (ObjectDirectoryEntry& ode : objectDirectory)
	{
		ode.Init("");
	}
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

GCodeResult ObjectTracker::HandleM486(GCodeBuffer &gb, const StringRef &reply, OutputBuffer*& buf) THROWS(GCodeException)
{
	bool seen = false;
	if (gb.Seen('T'))
	{
		// Specify how many objects. May be useful for a user interface.
		seen = true;
		numObjects = gb.GetUIValue();
		objectsCancelled.Clear();						// assume this command is only used at the start of a print
		reprap.JobUpdated();
	}

	if (gb.Seen('S'))
	{
		// Specify which object we are about to print
		seen = true;
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
			else if (num < (int)MaxTrackedObjects && strcmp(objectName.c_str(), objectDirectory[num].name.Get().Ptr()) != 0)
			{
				objectDirectory[num].SetName(objectName.c_str());
				reprap.JobUpdated();
			}
		}
#endif
		ChangeToObject(gb, num);
	}

	const bool seenC = gb.Seen('C');
	if (seenC || gb.Seen('P'))
	{
		// Cancel an object
		seen = true;
		const int objectToCancel = (seenC) ? currentObjectNumber : (int)gb.GetUIValue();
		if (objectToCancel < 0)
		{
			reply.copy("No current object");
			return GCodeResult::error;
		}
		if (objectToCancel >= (int)objectsCancelled.MaxBits())
		{
			reply.copy("Object number out of range");
			return GCodeResult::error;
		}

		// We don't flag an error if you try to cancel the same object twice
		if (!objectsCancelled.IsBitSet(objectToCancel))
		{
			objectsCancelled.SetBit(objectToCancel);
			if (objectToCancel == currentObjectNumber)
			{
				StopPrinting(gb);
			}
			reprap.JobUpdated();
			reply.printf("Object %d cancelled", objectToCancel);
		}
	}

	if (gb.Seen('U'))
	{
		// Resume an object
		seen = true;
		const int objectToResume = gb.GetUIValue();
		if (objectToResume >= (int)objectsCancelled.MaxBits())
		{
			reply.copy("Object number out of range");
			return GCodeResult::error;
		}

		// We don't flag an error if you try to resume an object that is not cancelled
		if (objectsCancelled.IsBitSet(objectToResume))
		{
			objectsCancelled.ClearBit(objectToResume);
			if (objectToResume == currentObjectNumber)
			{
				ResumePrinting(gb);
			}
			reprap.JobUpdated();
			reply.printf("Object %d resumed", objectToResume);
		}
	}

	if (!seen)
	{
#if TRACK_OBJECT_NAMES
		// List objects on build plate
		if (!OutputBuffer::Allocate(buf))
		{
			return GCodeResult::notFinished;
		}

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
#else
		if (numObjects == 0)
		{
			reply.copy("No known objects on build plate");
		}
		else
		{
			reply.printf("%u objects on build plate", numObjects);
		}
#endif
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

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Write the object details to file, returning true if successful
bool ObjectTracker::WriteObjectDirectory(FileStore *f) const noexcept
{
	bool ok = true;

	// Write the object list
#if TRACK_OBJECT_NAMES
	for (size_t i = 0; ok && i < min<unsigned int>(numObjects, MaxTrackedObjects); ++i)
	{
		String<StringLength100> buf;
		buf.printf("M486 S%u A\"%s\"\n", i, objectDirectory[i].name.Get().Ptr());
		ok = f->Write(buf.c_str());
	}
#else
	{
		String<StringLength20> buf;
		buf.printf("M486 T%u", numObjects);
		ok = f->Write(buf.c_str());
	}
#endif

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
		buf.printf("M486 S%d\n", currentObjectNumber);
		ok = f->Write(buf.c_str());
	}

	return ok;
}

#endif

#if TRACK_OBJECT_NAMES

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
void ObjectTracker::UpdateObjectCoordinates(const float coords[], AxesBitmap axes) noexcept
{
	if (currentObjectNumber >= 0 && currentObjectNumber < (int)MaxTrackedObjects)
	{
		if (objectDirectory[currentObjectNumber].UpdateObjectCoordinates(coords, axes))
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
void ObjectTracker::StartObject(GCodeBuffer& gb, const char *label) noexcept
{
	if (!usingM486Naming)
	{
		for (size_t i = 0; i < min<size_t>(numObjects, MaxTrackedObjects); ++i)
		{
			if (strcmp(objectDirectory[i].name.Get().Ptr(), label) == 0)
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
			ChangeToObject(gb, MaxTrackedObjects);
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

#endif

// End
