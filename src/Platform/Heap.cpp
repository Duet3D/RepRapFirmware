/*
 * Heap.cpp
 *
 *  Created on: 5 Mar 2021
 *      Author: David
 *
 * The string heap uses two structures.
 * Each index block is an array of pointers to the actual data. This allows the data to be moved when the heap is compacted. The first pointer in the block points to the next index block.
 * The heap itself is a sequence of blocks. Each block comprises a 2-byte length count followed by the null-terminated string. The length count is always even and the lowest bit is set if the block is free.
 */

#include "Heap.h"
#include <Platform/Tasks.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <General/String.h>
#include <atomic>

#define CHECK_HANDLES	(1)							// set nonzero to check that handles are valid before dereferencing them

constexpr size_t IndexBlockSlots = 99;				// number of 4-byte handles per index block, plus one for link to next index block
constexpr size_t HeapBlockSize = 2048;				// the size of each heap block

struct StorageSpace
{
	uint16_t length;
	char data[];									// array of unspecified length at the end of a struct is a GNU extension (valid in C but not valid in standard C++)
};

struct IndexSlot
{
	StorageSpace *storage;
	std::atomic<unsigned int> refCount;

	IndexSlot() noexcept : storage(nullptr), refCount(0) { }
};

struct IndexBlock
{
	void* operator new(size_t count) { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	IndexBlock() noexcept : next(nullptr) { }

	IndexBlock *next;
	IndexSlot slots[IndexBlockSlots];
};

struct HeapBlock
{
	void* operator new(size_t count) { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	HeapBlock(HeapBlock *pNext) noexcept : next(pNext), allocated(0) { }
	HeapBlock *next;
	size_t allocated;
	char data[HeapBlockSize];
};

ReadWriteLock StringHandle::heapLock;
IndexBlock *StringHandle::indexRoot = nullptr;
HeapBlock *StringHandle::heapRoot = nullptr;
size_t StringHandle::handlesAllocated = 0;
std::atomic<size_t> StringHandle::handlesUsed = 0;
size_t StringHandle::heapAllocated = 0;
size_t StringHandle::heapUsed = 0;
std::atomic<size_t> StringHandle::heapToRecycle = 0;
unsigned int StringHandle::gcCyclesDone = 0;

/*static*/ void StringHandle::GarbageCollect() noexcept
{
	WriteLocker locker(heapLock);
	GarbageCollectInternal();
}

/*static*/ void StringHandle::GarbageCollectInternal() noexcept
{
#if CHECK_HANDLES
	RRF_ASSERT(heapLock.GetWriteLockOwner() == TaskBase::GetCallerTaskHandle());
#endif

	heapUsed = 0;
	for (HeapBlock *currentBlock = heapRoot; currentBlock != nullptr; )
	{
		// Skip any used blocks at the start because they won't be moved
		char *p = currentBlock->data;
		while (p < currentBlock->data + currentBlock->allocated)
		{
			const size_t len = reinterpret_cast<StorageSpace*>(p)->length;
			if (len & 1u)					// if this slot has been marked as free
			{
				break;
			}
			p += len + sizeof(StorageSpace::length);
		}

		if (p < currentBlock->data + currentBlock->allocated)					// if we found an unused block before we reached the end
		{
			char* startSkip = p;

			for (;;)
			{
				// Find the end of the unused blocks
				while (p < currentBlock->data + currentBlock->allocated)
				{
					const size_t len = reinterpret_cast<StorageSpace*>(p)->length;
					if ((len & 1u) == 0)
					{
						break;
					}
					p += (len & ~1u) + sizeof(StorageSpace::length);
				}

				if (p >= currentBlock->data + currentBlock->allocated)
				{
					currentBlock->allocated = startSkip - currentBlock->data;	// the unused blocks were at the end so just change the allocated size
					break;
				}
				else
				{
					// Find all the contiguous blocks
					char *startUsed = p;
					unsigned int numHandlesToAdjust = 0;
					while (p < currentBlock->data + currentBlock->allocated)
					{
						const size_t len = reinterpret_cast<StorageSpace*>(p)->length;
						if (len & 1u)
						{
							break;
						}
						++numHandlesToAdjust;
						p += len + sizeof(StorageSpace::length);
					}

					// Move the contiguous blocks down
					memmove(startSkip, startUsed, p - startUsed);
					//TODO make this more efficient by building up a small table of several adjustments, so we need to make fewer passes through the index
					AdjustHandles(startUsed, p, startUsed - startSkip, numHandlesToAdjust);
					startSkip += p - startUsed;
				}
			}
		}

		heapUsed += currentBlock->allocated;
		currentBlock = currentBlock->next;
	}

	heapToRecycle = 0;
	++gcCyclesDone;
}

// Find all handles pointing to storage between startAddr and endAddr and move the pointers down by amount moveDown
/*static*/ void StringHandle::AdjustHandles(char *startAddr, char *endAddr, size_t moveDown, unsigned int numHandles) noexcept
{
	for (IndexBlock *indexBlock = indexRoot; indexBlock != nullptr; indexBlock = indexBlock->next)
	{
		for (size_t i = 0; i < IndexBlockSlots; ++i)
		{
			char * const p = (char *)indexBlock->slots[i].storage;
			if (p != nullptr && p >= startAddr && p < endAddr)
			{
				indexBlock->slots[i].storage = reinterpret_cast<StorageSpace*>(p - moveDown);
				--numHandles;
				if (numHandles == 0)
				{
					return;
				}
			}
		}
	}
}

/*static*/ bool StringHandle::CheckIntegrity(const StringRef& errmsg) noexcept
{
	ReadLocker lock(heapLock);

	// Check that all heap block entries end at the allocated length
	unsigned int numHeapErrors = 0;
	for (HeapBlock *currentBlock = heapRoot; currentBlock != nullptr; currentBlock = currentBlock->next)
	{
		const char *p = currentBlock->data;
		while (p != currentBlock->data + currentBlock->allocated)
		{
			if (p > currentBlock->data + currentBlock->allocated)
			{
				++numHeapErrors;
				break;
			}
			p += (reinterpret_cast<const StorageSpace*>(p)->length & (~1u)) + sizeof(StorageSpace::length);
		}
	}

	if (numHeapErrors != 0)
	{
		errmsg.printf("%u bad heap blocks", numHeapErrors);
		return false;
	}

	// Check that all handles point into allocated heap block entries
	unsigned int numHandleErrors = 0, numHandleFreeErrors = 0;
	for (IndexBlock *curBlock = indexRoot; curBlock != nullptr; curBlock = curBlock->next)
	{
		for (size_t i = 0; i < IndexBlockSlots; ++i)
		{
			const char *st = (const char*)curBlock->slots[i].storage;
			if (st != nullptr)
			{
				bool found = false;
				for (HeapBlock *currentBlock = heapRoot; currentBlock != nullptr; currentBlock = currentBlock->next)
				{
					const char *p = currentBlock->data;
					const char * const limit = currentBlock->data + currentBlock->allocated;
					if (st >= p && st < limit)
					{
						while (p < limit)
						{
							if (p == st)
							{
								found = true;
								if (reinterpret_cast<const StorageSpace*>(p)->length & 1u)
								{
									++numHandleFreeErrors;
								}
								break;
							}
							p += (reinterpret_cast<const StorageSpace*>(p)->length & (~1u)) + sizeof(StorageSpace::length);
						}
						break;
					}
				}

				if (!found)
				{
					++numHandleErrors;
				}
			}
		}
	}

	if (numHandleErrors != 0 || numHandleFreeErrors != 0)
	{
		errmsg.printf("%u bad handles, %u handles with freed storage", numHandleErrors, numHandleFreeErrors);
		return false;

	}

	return true;
}

// Allocate a new handle. Must own the write lock when calling this.
/*static*/ IndexSlot *StringHandle::AllocateHandle() noexcept
{
#if CHECK_HANDLES
	RRF_ASSERT(heapLock.GetWriteLockOwner() == TaskBase::GetCallerTaskHandle());
#endif

	IndexBlock *prevIndexBlock = nullptr;
	for (IndexBlock* curBlock = indexRoot; curBlock != nullptr; )
	{
		// Search for a free slot in this block
		for (size_t i = 0; i < IndexBlockSlots; ++i)
		{
			if (curBlock->slots[i].storage == nullptr)
			{
				curBlock->slots[i].refCount = 0;
				++handlesUsed;
				return &curBlock->slots[i];
			}
		}
		prevIndexBlock = curBlock;
		curBlock = curBlock->next;
	}

	// If we get here then we didn't find a free handle entry
	IndexBlock * const newIndexBlock = new IndexBlock;
	handlesAllocated += IndexBlockSlots;

	if (prevIndexBlock == nullptr)
	{
		indexRoot = newIndexBlock;
	}
	else
	{
		prevIndexBlock->next = newIndexBlock;
	}

	++handlesUsed;
	return &newIndexBlock->slots[0];
}

// Allocate the requested space. If 'length' is above the maximum supported size, it will be truncated.
/*static*/ StorageSpace *StringHandle::AllocateSpace(size_t length) noexcept
{
#if CHECK_HANDLES
	RRF_ASSERT(heapLock.GetWriteLockOwner() == TaskBase::GetCallerTaskHandle());
#endif

	length = min<size_t>((length + 1) & (~1u), HeapBlockSize - sizeof(StorageSpace::length));	// round to an even length to keep things aligned and limit to max size

	bool collected = false;
	do
	{
		for (HeapBlock *currentBlock = heapRoot; currentBlock != nullptr; currentBlock = currentBlock->next)
		{
			if (HeapBlockSize - sizeof(StorageSpace::length) >= currentBlock->allocated + length)		// if the data will fit at the end of the current block
			{
				StorageSpace * const ret = reinterpret_cast<StorageSpace*>(currentBlock->data + currentBlock->allocated);
				ret->length = length;
				currentBlock->allocated += length + sizeof(StorageSpace::length);
				heapUsed += length + sizeof(StorageSpace::length);
				return ret;
			}
		}

		// There is no space in any existing heap block. Decide whether to garbage collect and try again, or allocate a new block.
		if (collected || heapToRecycle < length * 4)
		{
			break;
		}
		GarbageCollectInternal();
		collected = true;
	} while (true);

	// Create a new heap block
	heapRoot = new HeapBlock(heapRoot);
	heapAllocated += HeapBlockSize;
	heapUsed += length + sizeof(StorageSpace::length);
	StorageSpace * const ret2 = reinterpret_cast<StorageSpace*>(heapRoot->data);
	ret2->length = length;
	heapRoot->allocated = length + sizeof(StorageSpace::length);
	return ret2;
}

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
		WriteLocker locker(heapLock);							// prevent other tasks modifying the heap
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
		WriteLocker locker(heapLock);							// prevent other tasks modifying the heap
		InternalAssign(s, len);
	}
}

void StringHandle::InternalAssign(const char *s, size_t len) noexcept
{
	IndexSlot * const slot = AllocateHandle();
	StorageSpace * const space = AllocateSpace(len + 1);
	SafeStrncpy(space->data, s, space->length);
	slot->storage = space;
	slot->refCount = 1;
	slotPtr = slot;
}

void StringHandle::Delete() noexcept
{
	if (slotPtr != nullptr)
	{
		ReadLocker locker(heapLock);							// prevent other tasks modifying the heap
		InternalDelete();
	}
}

const StringHandle& StringHandle::IncreaseRefCount() const noexcept
{
	if (slotPtr != nullptr)
	{
		++slotPtr->refCount;
	}
	return *this;
}

// Caller must have at least a read lock when calling this
void StringHandle::InternalDelete() noexcept
{
	RRF_ASSERT(slotPtr->refCount != 0);
	RRF_ASSERT(slotPtr->storage != nullptr);
	if (--slotPtr->refCount == 0)
	{
		heapToRecycle += slotPtr->storage->length + sizeof(StorageSpace::length);
		slotPtr->storage->length |= 1;									// flag the space as unused
		slotPtr->storage = nullptr;							// release the handle entry
		--handlesUsed;
	}
	slotPtr = nullptr;										// clear the pointer to the handle entry
}

ReadLockedPointer<const char> StringHandle::Get() const noexcept
{
	if (slotPtr == nullptr)
	{
		return ReadLockedPointer<const char>(nullptr, "");	// a null handle means an empty string
	}

	ReadLocker locker(heapLock);

#if CHECK_HANDLES
	// Check that the handle points into an index block
	RRF_ASSERT(((uint32_t)slotPtr & 3) == 0);
	bool ok = false;
	for (IndexBlock *indexBlock = indexRoot; indexBlock != nullptr; indexBlock = indexBlock->next)
	{
		if (slotPtr >= &indexBlock->slots[0] && slotPtr < &indexBlock->slots[IndexBlockSlots])
		{
			ok = true;
			break;
		}
	}
	RRF_ASSERT(ok);
	RRF_ASSERT(slotPtr->refCount != 0);
#endif

	return ReadLockedPointer<const char>(locker, slotPtr->storage->data);
}

size_t StringHandle::GetLength() const noexcept
{
	if (slotPtr == nullptr)
	{
		return 0;
	}

	ReadLocker locker(heapLock);

#if CHECK_HANDLES
	// Check that the handle points into an index block and is not null
	RRF_ASSERT(((uint32_t)slotPtr & 3) == 0);
	bool ok = false;
	for (IndexBlock *indexBlock = indexRoot; indexBlock != nullptr; indexBlock = indexBlock->next)
	{
		if (slotPtr >= &indexBlock->slots[0] && slotPtr < &indexBlock->slots[IndexBlockSlots])
		{
			ok = true;
			break;
		}
	}
	RRF_ASSERT(ok);
	RRF_ASSERT(slotPtr->refCount != 0);
#endif

	return strlen(slotPtr->storage->data);
}

/*static*/ void StringHandle::Diagnostics(MessageType mt) noexcept
{
	String<StringLength256> temp;
	const bool ok = CheckIntegrity(temp.GetRef());
	if (ok)
	{
		temp.copy("Heap OK");
	}
	temp.catf(", handles allocated/used %u/%u, heap memory allocated/used/recyclable %u/%u/%u, gc cycles %u\n",
					handlesAllocated, (unsigned int)handlesUsed, heapAllocated, heapUsed, (unsigned int)heapToRecycle, gcCyclesDone);
	reprap.GetPlatform().Message(mt, temp.c_str());
}

// AutoStringHandle members

AutoStringHandle::AutoStringHandle(const AutoStringHandle& other) noexcept
{
	IncreaseRefCount();
}

AutoStringHandle::AutoStringHandle(AutoStringHandle&& other) noexcept
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
