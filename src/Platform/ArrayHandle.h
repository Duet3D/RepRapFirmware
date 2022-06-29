/*
 * ArrayHandle.h
 *
 *  Created on: 12 Jun 2022
 *      Author: David
 */

#ifndef SRC_PLATFORM_ARRAYHANDLE_H_
#define SRC_PLATFORM_ARRAYHANDLE_H_

#include <RepRapFirmware.h>
#include "Heap.h"

class ExpressionValue;

// Note: ArrayHandle is a union member in ExpressionValue, therefore it must be no larger than 32 bits and it cannot have a non-trivial destructor, copy constructor etc.
// This means that when an object containing an ArrayHandle is copied or destroyed, that object must handle the reference count.
// Classes other than ExpressionValue should use AutoArrayHandle instead;
class ArrayHandle
{
public:
	ArrayHandle() noexcept { slotPtr = nullptr; }										// build an empty array

	void Allocate(size_t numElements) THROWS(GCodeException);
	void AssignElement(size_t index, ExpressionValue& val) THROWS(GCodeException);

	size_t GetNumElements() const noexcept;												// get the number of elements
	void GetElement(size_t index, ExpressionValue& rslt) const THROWS(GCodeException);	// get the specified element
	void Delete() noexcept;
	const ArrayHandle& IncreaseRefCount() const noexcept;
	bool IsNull() const noexcept { return slotPtr == nullptr; }

protected:
	Heap::IndexSlot * null slotPtr;
};

// Version of ArrayHandle that updates the reference counts automatically
class AutoArrayHandle : public ArrayHandle
{
public:
	AutoArrayHandle() noexcept : ArrayHandle() { }
	AutoArrayHandle(const AutoArrayHandle& other) noexcept;
	AutoArrayHandle(AutoArrayHandle&& other) noexcept;
	AutoArrayHandle& operator=(const AutoArrayHandle& other) noexcept;
	~AutoArrayHandle();
};

#endif /* SRC_PLATFORM_ARRAYHANDLE_H_ */
