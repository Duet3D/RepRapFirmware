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

// Version of struct StorageSpace to represent an array. This overlays struct StorageSpace, so the 'length' field corresponds to the same field in that struct.
struct ArrayStorageSpace
{
	uint16_t length;								// length of this object in bytes including this length field, always rounded up to a multiple of 4. Overlaps the 'length' field
	uint16_t count;									// number of elements in the array
	ExpressionValue elements[];						// the array elements
};

// Allocate space for an array and initialise its elements to null expressions
void ArrayHandle::Allocate(size_t numElements) THROWS(GCodeException)
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasWriteLock();
#endif
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

// Assign an element without making the array unique first
void ArrayHandle::AssignElement(size_t index, ExpressionValue &val) THROWS(GCodeException)
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasWriteLock();
#endif
	if (slotPtr != nullptr)
	{
		ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
		if (index < aSpace->count)
		{
			aSpace->elements[index] = val;
			return;
		}
	}
	throw GCodeException("array index out of bounds");
}

// Make an array unique and assign an element, possibly nested
void ArrayHandle::AssignIndexed(const ExpressionValue& ev, size_t numIndices, const uint32_t *indices) THROWS(GCodeException)
{
	WriteLocker locker(Heap::heapLock);							// prevent other tasks modifying the heap
	InternalAssignIndexed(this, ev, numIndices, indices);
}

// Make an array unique and assign an element, possibly nested
void ArrayHandle::InternalAssignIndexed(volatile ArrayHandle *ah, const ExpressionValue& ev, size_t numIndices, const uint32_t *indices) THROWS(GCodeException)
{
	while (true)
	{
		if (indices[0] >= (const_cast<ArrayHandle*>(ah))->GetNumElements())
		{
			throw GCodeException("array index out of bounds");
		}

		Heap::IndexSlot *const newSlotPtr = MakeUnique(ah);

		// CAUTION: the object that 'ah' points to may have been moved by garbage collection inside the call to MakeUnique; so don't refer to 'ah' any more until we reassign it
		ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(newSlotPtr->storage);
		if (numIndices == 1)
		{
			aSpace->elements[indices[0]] = ev;
			return;
		}

		if (aSpace->elements[indices[0]].GetType() != TypeCode::HeapArray)
		{
			throw GCodeException("attempt to index into a non-array");
		}

		ah = &(aSpace->elements[indices[0]].ahVal);
		++indices;
		--numIndices;
	}
}

// Get the number of elements in a heap array
// Caller must have a read lock or a write lock on heapLock before calling this!
size_t ArrayHandle::GetNumElements() const noexcept
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasReadOrWriteLock();
#endif
	if (slotPtr == nullptr)
	{
		return 0;
	}
#if CHECK_HANDLES
	Heap::CheckSlotGood(slotPtr);
#endif
	const Heap::StorageSpace *null storage = slotPtr->storage;
	if (storage == nullptr)
	{
		return 0;
	}
	return reinterpret_cast<const ArrayStorageSpace*>(storage)->count;
}

// Retrieve an array element, returning false if the index is out of bounds
// Caller must have a read lock on heapLock before calling this!
bool ArrayHandle::GetElement(size_t index, ExpressionValue &rslt) const noexcept
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasReadLock();
#endif
	if (slotPtr != nullptr)
	{
#if CHECK_HANDLES
		Heap::CheckSlotGood(slotPtr);
#endif
		const Heap::StorageSpace *const _ecv_null storage = slotPtr->storage;
		if (storage != nullptr)
		{
			const ArrayStorageSpace *const aSpace = reinterpret_cast<const ArrayStorageSpace*>(storage);
			if (index < aSpace->count)
			{
				rslt = aSpace->elements[index];
				return true;
			}
		}
	}
	return false;
}

// Retrieve the type of an array element, or TypeCode::None if the index is out of range
// Caller must have a read lock on heapLock before calling this!
TypeCode ArrayHandle::GetElementType(size_t index) const noexcept
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasReadLock();
#endif
	if (slotPtr != nullptr)
	{
#if CHECK_HANDLES
		Heap::CheckSlotGood(slotPtr);
#endif
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

// Make this handle refer to a non-shared array (the individual elements may be shared). Caller must already own a write lock on the heap.
// Return a pointer to the (possibly new) slot that the array is stored in
Heap::IndexSlot *ArrayHandle::MakeUnique(volatile ArrayHandle *ah) THROWS(GCodeException)
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasWriteLock();
#endif
	Heap::IndexSlot * const oldSlotPtr = ah->slotPtr;
	if (oldSlotPtr != nullptr && oldSlotPtr->refCount > 1)
	{
		// CAUTION: the object that 'ah' points to may be moved by garbage collection when we allocate space for a copy of the array.
		// Therefore we first allocate a slot for the new array with no associated storage, and make our array handle point to it.
		// Only then do we allocate storage for the new array, copy the data over, and make the new slot point to it.
		Heap::IndexSlot *const newSlotPtr = Heap::AllocateHandle();				// this sets the ref count of the new slot to 1 and the storage to nullptr
		ah->slotPtr = newSlotPtr;
		--(oldSlotPtr->refCount);
		const size_t numElements = reinterpret_cast<const ArrayStorageSpace*>(oldSlotPtr->storage)->count;

		// From now on in this scope we must not reference 'ah' because the object it points to may be moved by the call to AllocateSpace
		Heap::StorageSpace *const space = Heap::AllocateSpace(sizeof(ArrayStorageSpace) + numElements * sizeof(ExpressionValue));
		newSlotPtr->storage = space;
		if (space->length < sizeof(ArrayStorageSpace) + numElements * sizeof(ExpressionValue))
		{
			Heap::DeleteSlot(newSlotPtr);
			throw GCodeException("Array too large");
		}

		ArrayStorageSpace *const oldSpace = reinterpret_cast<ArrayStorageSpace*>(oldSlotPtr->storage);
		ArrayStorageSpace *const newSpace = reinterpret_cast<ArrayStorageSpace*>(space);
		newSpace->count = numElements;
		for (size_t i = 0; i < numElements; ++i)
		{
			new (&newSpace->elements[i]) ExpressionValue(oldSpace->elements[i]);
		}
		return newSlotPtr;
	}
	return oldSlotPtr;
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
