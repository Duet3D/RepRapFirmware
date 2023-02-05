/*
 * StringHandle.cpp
 *
 *  Created on: 12 Jun 2022
 *      Author: David
 */

#include "StringHandle.h"

// StringHandle members
// Build a handle from a single null-terminated string
StringHandle::StringHandle(const char *s) noexcept : StringHandle(s, strlen(s)) { }

// Build a handle from a character array and a length
StringHandle::StringHandle(const char *s, size_t len) noexcept
{
	if (len == 0)
	{
		slotPtr = nullptr;
	}
	else
	{
		WriteLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
		InternalAssign(s, len);
	}
}

#if 0	// This constructor is currently unused, but may be useful in future
// Build a handle by concatenating two strings
StringHandle::StringHandle(const char *s1, const char *s2) noexcept
{
	const size_t len = strlen(s1) + strlen(s2);
	if (len == 0)
	{
		slotPtr = nullptr;
	}
	else
	{
		WriteLocker locker(heapLock);							// prevent other tasks modifying the heap
		IndexSlot * const slot = AllocateHandle();
		StorageSpace * const space = AllocateSpace(len + 1);
		SafeStrncpy(space->data, s1, space->length);
		SafeStrncat(space->data, s2, space->length);
		slot->storage = space;
		slot->refCount = 1;
		slotPtr = slot;
	}
}
#endif

void StringHandle::Assign(const char *s) noexcept
{
	Delete();
	const size_t len = strlen(s);
	if (len != 0)
	{
		WriteLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
		InternalAssign(s, len);
	}
}

// Assign the string. Caller must hold a write lock on the heap.
void StringHandle::InternalAssign(const char *s, size_t len) noexcept
{
	Heap::IndexSlot * const slot = Heap::AllocateHandle();
	Heap::StorageSpace * const space = Heap::AllocateSpace(sizeof(Heap::StorageSpace) + len + 1);
	SafeStrncpy(space->data, s, min(len + 1, space->length - sizeof(Heap::StorageSpace)));
	slot->storage = space;
	slotPtr = slot;
}

void StringHandle::Delete() noexcept
{
	if (slotPtr != nullptr)
	{
		ReadLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
		Heap::DeleteSlot(slotPtr);
		slotPtr = nullptr;										// clear the pointer to the handle entry
	}
}

const StringHandle& StringHandle::IncreaseRefCount() const noexcept
{
	if (slotPtr != nullptr)
	{
		Heap::IncreaseRefCount(slotPtr);
	}
	return *this;
}

ReadLockedPointer<const char> StringHandle::Get() const noexcept
{
	if (slotPtr == nullptr)
	{
		return ReadLockedPointer<const char>(nullptr, "");		// a null handle means an empty string
	}

	ReadLocker locker(Heap::heapLock);

#if CHECK_HANDLES
	Heap::CheckSlotGood(slotPtr);
#endif

	return ReadLockedPointer<const char>(locker, slotPtr->storage->data);
}

size_t StringHandle::GetLength() const noexcept
{
	if (slotPtr == nullptr)
	{
		return 0;
	}

	ReadLocker locker(Heap::heapLock);

#if CHECK_HANDLES
	Heap::CheckSlotGood(slotPtr);
#endif

	return strlen(slotPtr->storage->data);
}

// AutoStringHandle members

AutoStringHandle::AutoStringHandle(const AutoStringHandle& other) noexcept
	: StringHandle(other)
{
	IncreaseRefCount();
}

AutoStringHandle::AutoStringHandle(AutoStringHandle&& other) noexcept
	: StringHandle(other)
{
	other.slotPtr = nullptr;
}

AutoStringHandle& AutoStringHandle::operator=(const AutoStringHandle& other) noexcept
{
	if (slotPtr != other.slotPtr)
	{
		Delete();
		slotPtr = other.slotPtr;
		IncreaseRefCount();
	}
	return *this;
}

AutoStringHandle::~AutoStringHandle()
{
	StringHandle::Delete();
}

// End
