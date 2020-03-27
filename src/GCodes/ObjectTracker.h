/*
 * ObjectTracker.h
 *
 *  Created on: 24 Mar 2020
 *      Author: David
 */

#ifndef SRC_GCODES_OBJECTTRACKER_H_
#define SRC_GCODES_OBJECTTRACKER_H_

#include <RepRapFirmware.h>
#include "GCodeResult.h"
#include "RestorePoint.h"
#include <ObjectModel/ObjectModel.h>
#include <General/StringBuffer.h>

#if TRACK_OBJECT_NAMES

struct ObjectDirectoryEntry INHERIT_OBJECT_MODEL
{
	const char *name;					// pointer to the object name within the string buffer
	float x[2], y[2];					// lowest and highest extrusion coordinates

	void Init(const char *label) noexcept;
	bool UpdateObjectCoordinates(const float coords[]) noexcept;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(x)
	OBJECT_MODEL_ARRAY(y)
};

#endif

class ObjectTracker
#if TRACK_OBJECT_NAMES
	INHERIT_OBJECT_MODEL
#endif
{
public:
	ObjectTracker() noexcept
		: numObjects(0)
#if TRACK_OBJECT_NAMES
		  , objectNames(stringBufferStorage, ARRAY_SIZE(stringBufferStorage))
#endif
	{ }

	void Init() noexcept;
	bool IsCurrentObjectCancelled() const noexcept { return currentObjectCancelled; }
	bool IsFirstMoveSincePrintingResumed() const noexcept { return printingJustResumed; }
	void DoneMoveSincePrintingResumed() noexcept { printingJustResumed = false; }
	GCodeResult HandleM486(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);	// Handle M486
	const RestorePoint& GetInitialPosition() const noexcept { return rp; }
	void SetVirtualTool(int toolNum) noexcept { virtualToolNumber = toolNum; }

#if TRACK_OBJECT_NAMES
	void StartObject(GCodeBuffer& gb, const char *label) noexcept;
	void StopObject(GCodeBuffer& gb) noexcept;
	void UpdateObjectCoordinates(const float coords[]) noexcept;
	bool IsCancelled(size_t objectNumber) const noexcept { return objectsCancelled.IsBitSet(objectNumber); }
#endif

protected:

#if TRACK_OBJECT_NAMES
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(objects)
#endif

private:
	typedef Bitmap<uint32_t> ObjectCancellationBitmap;	// Type of a bitmap used to represent objects on the build plate that have been cancelled

	void ChangeToObject(GCodeBuffer& gb, int i) noexcept;
	void StopPrinting(GCodeBuffer& gb) noexcept;
	void ResumePrinting(GCodeBuffer& gb) noexcept;

#if TRACK_OBJECT_NAMES
	void CreateObject(unsigned int number, const char *label) noexcept;
	void SetObjectName(unsigned int number, const char *label) noexcept;
#endif

	RestorePoint rp;									// The user coordinates at the point of restarting moves after skipping an object
	ObjectCancellationBitmap objectsCancelled;			// Which object numbers have been cancelled. The number of bits in this is the maximum number of objects we can track.
	unsigned int numObjects;							// How many objects we know about, if known
	int currentObjectNumber;							// the current object number, or a negative value if it isn't an object
	int virtualToolNumber;								// the number of the tool that was active when we cancelled an object

#if TRACK_OBJECT_NAMES
	ObjectDirectoryEntry objectDirectory[MaxTrackedObjects];
	StringBuffer objectNames;
	char stringBufferStorage[ObjectNamesStringSpace];
	bool usingM486Naming;
#endif

	bool usingM486Labelling;
	bool currentObjectCancelled;						// true if the current object should not be printed
	bool printingJustResumed;							// true if we have just restarted printing
};

#endif /* SRC_GCODES_OBJECTTRACKER_H_ */
