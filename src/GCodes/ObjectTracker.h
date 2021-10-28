/*
 * ObjectTracker.h
 *
 *  Created on: 24 Mar 2020
 *      Author: David
 */

#ifndef SRC_GCODES_OBJECTTRACKER_H_
#define SRC_GCODES_OBJECTTRACKER_H_

#include <RepRapFirmware.h>
#include "RestorePoint.h"
#include <ObjectModel/ObjectModel.h>
#include <Platform/Heap.h>

#if TRACK_OBJECT_NAMES

struct ObjectDirectoryEntry
{
	AutoStringHandle name;					// pointer to the object name within the string buffer
	int16_t x[2], y[2];						// lowest and highest extrusion coordinates

	void Init(const char *label) noexcept;
	bool UpdateObjectCoordinates(const float coords[], AxesBitmap axes) noexcept;
	void SetName(const char *label) noexcept;
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
	{ }

	void Init() noexcept;
	bool IsCurrentObjectCancelled() const noexcept { return currentObjectCancelled; }
	bool IsFirstMoveSincePrintingResumed() const noexcept { return printingJustResumed; }
	void DoneMoveSincePrintingResumed() noexcept { printingJustResumed = false; }
	GCodeResult HandleM486(GCodeBuffer& gb, const StringRef &reply, OutputBuffer*& buf) THROWS(GCodeException);	// Handle M486
	const RestorePoint& GetInitialPosition() const noexcept { return rp; }
	void SetVirtualTool(int toolNum) noexcept { virtualToolNumber = toolNum; }

#if TRACK_OBJECT_NAMES
	void StartObject(GCodeBuffer& gb, const char *label) noexcept;
	void StopObject(GCodeBuffer& gb) noexcept;
	void UpdateObjectCoordinates(const float coords[], AxesBitmap axes) noexcept;
	bool IsCancelled(size_t objectNumber) const noexcept { return objectsCancelled.IsBitSet(objectNumber); }
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteObjectDirectory(FileStore *f) const noexcept;
#endif

protected:

#if TRACK_OBJECT_NAMES
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(objects)
	OBJECT_MODEL_ARRAY(x)
	OBJECT_MODEL_ARRAY(y)
#endif

private:
#if SAME70 || SAME5x
	typedef Bitmap<uint64_t> ObjectCancellationBitmap;	// Type of a bitmap used to represent objects on the build plate that have been cancelled
#else
	typedef Bitmap<uint32_t> ObjectCancellationBitmap;	// Type of a bitmap used to represent objects on the build plate that have been cancelled
#endif
	static_assert(MaxTrackedObjects <= ObjectCancellationBitmap::MaxBits());

	void ChangeToObject(GCodeBuffer& gb, int i) noexcept;
	void StopPrinting(GCodeBuffer& gb) noexcept;
	void ResumePrinting(GCodeBuffer& gb) noexcept;

#if TRACK_OBJECT_NAMES
	void CreateObject(unsigned int number, const char *label) noexcept;
	ExpressionValue GetXCoordinate(const ObjectExplorationContext& context) const noexcept;
	ExpressionValue GetYCoordinate(const ObjectExplorationContext& context) const noexcept;
#endif

	RestorePoint rp;									// The user coordinates at the point of restarting moves after skipping an object
	ObjectCancellationBitmap objectsCancelled;			// Which object numbers have been cancelled. The number of bits in this is the maximum number of objects we can track.
	unsigned int numObjects;							// How many objects we know about, if known
	int currentObjectNumber;							// the current object number, or a negative value if it isn't an object
	int virtualToolNumber;								// the number of the tool that was active when we cancelled an object

#if TRACK_OBJECT_NAMES
	ObjectDirectoryEntry objectDirectory[MaxTrackedObjects];
	bool usingM486Naming;
#endif

	bool usingM486Labelling;
	bool currentObjectCancelled;						// true if the current object should not be printed
	bool printingJustResumed;							// true if we have just restarted printing
};

#endif /* SRC_GCODES_OBJECTTRACKER_H_ */
