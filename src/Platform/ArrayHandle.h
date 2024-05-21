/*
 * ArrayHandle.h
 *
 *  Created on: 12 Jun 2022
 *      Author: David
 */

#ifndef SRC_PLATFORM_ARRAYHANDLE_H_
#define SRC_PLATFORM_ARRAYHANDLE_H_

#include <RepRapFirmware.h>
#include <ObjectModel/TypeCode.h>
#include "Heap.h"

#define CHECK_HEAP_LOCKED		(1)

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
	void AssignIndexed(const ExpressionValue& ev, size_t numIndices, const uint32_t *indices) THROWS(GCodeException) pre(numIndeces != 0);

	size_t GetNumElements() const noexcept;												// get the number of elements
	bool GetElement(size_t index, ExpressionValue& rslt) const noexcept;				// return true and get the specified element if the index is in range
	TypeCode GetElementType(size_t index) const noexcept;
	void Delete() noexcept;
	const ArrayHandle& IncreaseRefCount() const noexcept;
	bool IsNull() const noexcept { return slotPtr == nullptr; }

protected:
	Heap::IndexSlot * null slotPtr;

private:
	static Heap::IndexSlot *MakeUnique(volatile ArrayHandle *ah) THROWS(GCodeException);
	static void InternalAssignIndexed(volatile ArrayHandle *ah, const ExpressionValue& ev, size_t numIndices, const uint32_t *indices) THROWS(GCodeException) pre(numIndeces != 0);
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
