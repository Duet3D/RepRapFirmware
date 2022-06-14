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

struct ObjectDirectoryEntry
{
	AutoStringHandle name;					// pointer to the object name within the string buffer
	int16_t x[2], y[2];						// lowest and highest extrusion coordinates

	void Init(const char *label) noexcept;
	bool UpdateObjectCoordinates(const float coords[], AxesBitmap axes) noexcept;
	void SetName(const char *label) noexcept;
};

class ObjectTracker
	INHERIT_OBJECT_MODEL
{
public:
	ObjectTracker() noexcept
		: numObjects(0)
	{ }

	void Init() noexcept;

	// Functions to implement aspects of M486
	void HandleM486T(unsigned int p_numObjects) noexcept;
	void UseM486Labelling() noexcept;
	void SetM486Label(unsigned int objectNumber, const char *_ecv_array label) noexcept;
	bool CancelObject(unsigned int objectNumber) noexcept;						// cancel an object, returning true if it was not already cancelled
	bool ResumeObject(unsigned int objectNumber) noexcept;						// resume an object, returning true if it was not already enabled
	void ListObjects(OutputBuffer *buf) noexcept;								// list the objects on the build plate

	size_t GetObjectNumber(const char *label) noexcept;
	void UpdateObjectCoordinates(int objectNumber, const float coords[], AxesBitmap axes) noexcept;
	bool IsCancelled(size_t objectNumber) const noexcept { return objectsCancelled.IsBitSet(objectNumber); }
	bool IsUsingM486Naming() const noexcept { return usingM486Naming; }
	bool CheckObject(int objectNumber) noexcept;								// Add the object number if it isn't already known, return true if it has been cancelled

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteObjectDirectory(FileStore *f) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
#if SAME70 || SAME5x
	typedef Bitmap<uint64_t> ObjectCancellationBitmap;	// Type of a bitmap used to represent objects on the build plate that have been cancelled
#else
	typedef Bitmap<uint32_t> ObjectCancellationBitmap;	// Type of a bitmap used to represent objects on the build plate that have been cancelled
#endif
	static_assert(MaxTrackedObjects <= ObjectCancellationBitmap::MaxBits());

	void CreateObject(unsigned int number, const char *label) noexcept;
	ExpressionValue GetXCoordinate(const ObjectExplorationContext& context) const noexcept;
	ExpressionValue GetYCoordinate(const ObjectExplorationContext& context) const noexcept;

	ObjectCancellationBitmap objectsCancelled;			// Which object numbers have been cancelled. The number of bits in this is the maximum number of objects we can track.
	unsigned int numObjects;							// How many objects we know about, if known

	ObjectDirectoryEntry objectDirectory[MaxTrackedObjects];
	bool usingM486Naming;
	bool usingM486Labelling;
};

#endif /* SRC_GCODES_OBJECTTRACKER_H_ */
