/*
 * ArrayHandle.cpp
 *
 *  Created on: 12 Jun 2022
 *      Author: David
 */

#include "ArrayHandle.h"
#include <ObjectModel/ObjectModel.h>

// Declare placement new operator for class ExpressionValue so that we don't have to include <new>
void* operator new(std::size_t, ExpressionValue* arg) noexcept { return arg; }

struct ArrayStorageSpace
{
	uint16_t length;								// length of this object in bytes including this length field, always rounded up to a multiple of 4
	uint16_t count;									// number of elements in the array
	ExpressionValue elements[];						// the array elements
};

// Allocate space for an array
void ArrayHandle::Allocate(size_t numElements) THROWS(GCodeException)
{
	WriteLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
	Heap::IndexSlot * const slot = Heap::AllocateHandle();
	Heap::StorageSpace * const space = Heap::AllocateSpace(sizeof(ArrayStorageSpace) + numElements * sizeof(ExpressionValue));
	slot->storage = space;
	if (space->length < sizeof(ArrayStorageSpace) + numElements * sizeof(ExpressionValue))
	{
		Heap::DeleteSlot(slot);
		throw GCodeException("Array too large");
	}

	ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(space);
	aSpace->count = numElements;
	for (size_t i = 0; i < numElements; ++i)
	{
		new (&aSpace->elements[i]) ExpressionValue;
	}
	slotPtr = slot;
}

void ArrayHandle::AssignElement(size_t index, ExpressionValue &val) THROWS(GCodeException)
{
	if (slotPtr != nullptr)
	{
		WriteLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
		ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
		if (index < aSpace->count)
		{
			aSpace->elements[index] = val;
			return;
		}
	}
	throw GCodeException("Array index out of bounds");
}

size_t ArrayHandle::GetNumElements() const noexcept
{
	if (slotPtr == nullptr)
	{
		return 0;
	}
	ReadLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
	return reinterpret_cast<const ArrayStorageSpace*>(slotPtr->storage)->count;
}

void ArrayHandle::GetElement(size_t index, ExpressionValue &rslt) const THROWS(GCodeException)
{
	if (slotPtr != nullptr)
	{
		ReadLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
		ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
		if (index < aSpace->count)
		{
			rslt = aSpace->elements[index];
			return;
		}
	}
	throw GCodeException("Array index out of bounds");
}

TypeCode ArrayHandle::GetElementType(size_t index) const noexcept
{
	if (slotPtr != nullptr)
	{
		ReadLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
		ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
		if (index < aSpace->count)
		{
			return aSpace->elements[index].GetType();
		}
	}
	return TypeCode::None;
}

// Decrease the reference count for the array referred to and delete it if it reaches zero
void ArrayHandle::Delete() noexcept
{
	if (slotPtr != nullptr)
	{
		ReadLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
		if (slotPtr->refCount == 1)
		{
			// The call to Heap:::DeleteSlot will deallocate the slot, so release the contents first
			ArrayStorageSpace *const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
			for (size_t i = 0; i < aSpace->count; ++i)
			{
				aSpace->elements[i].~ExpressionValue();			// call destructor on the elements
			}
		}
		Heap::DeleteSlot(slotPtr);
		slotPtr = nullptr;										// clear the pointer to the handle entry
	}
}

const ArrayHandle& ArrayHandle::IncreaseRefCount() const noexcept
{
	if (slotPtr != nullptr)
	{
		Heap::IncreaseRefCount(slotPtr);
	}
	return *this;
}

// AutoArrayHandle members

AutoArrayHandle::AutoArrayHandle(const AutoArrayHandle &other) noexcept
	: ArrayHandle(other)
{
	IncreaseRefCount();
}

AutoArrayHandle::AutoArrayHandle(AutoArrayHandle &&other) noexcept
	: ArrayHandle(other)
{
	other.slotPtr = nullptr;
}

AutoArrayHandle& AutoArrayHandle::operator =(const AutoArrayHandle &other) noexcept
{
	if (slotPtr != other.slotPtr)
	{
		Delete();
		slotPtr = other.slotPtr;
		IncreaseRefCount();
	}
	return *this;
}

AutoArrayHandle::~AutoArrayHandle()
{
	ArrayHandle::Delete();
}

// End
