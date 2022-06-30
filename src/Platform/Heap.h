/*
 * Heap.h
 *
 *  Created on: 5 Mar 2021
 *      Author: David
 */

#ifndef SRC_PLATFORM_HEAP_H_
#define SRC_PLATFORM_HEAP_H_

#include <RepRapFirmware.h>
#include <RTOSIface/RTOSIface.h>			// for class ReadLockedPointer

#include <atomic>

#define CHECK_HANDLES	(1)							// set nonzero to check that handles are valid before dereferencing them

namespace Heap
{
	struct StorageSpace
	{
		uint16_t length;								// length of this object in bytes including this length field, always rounded up to a multiple of 4
		char data[];									// array of unspecified length at the end of a struct is a GNU extension (valid in C but not valid in standard C++)
	};

	struct IndexSlot
	{
		StorageSpace *null storage;
		std::atomic<unsigned int> refCount;

		IndexSlot() noexcept : storage(nullptr), refCount(0) { }
	};

	IndexSlot *AllocateHandle() noexcept;
	StorageSpace *AllocateSpace(size_t length) noexcept;
	void CheckSlotGood(IndexSlot *slotPtr) noexcept;
	void IncreaseRefCount(IndexSlot *slotPtr) noexcept;
	void DeleteSlot(IndexSlot *slotPtr) noexcept;
	void GarbageCollect() noexcept;
	bool CheckIntegrity(const StringRef& errmsg) noexcept;
	void Diagnostics(MessageType mt, Platform& p) noexcept;

	extern ReadWriteLock heapLock;
}

#endif /* SRC_PLATFORM_HEAP_H_ */
