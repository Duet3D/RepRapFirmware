/*
 * Heap.h
 *
 *  Created on: 5 Mar 2021
 *      Author: David
 */

#ifndef SRC_PLATFORM_HEAP_H_
#define SRC_PLATFORM_HEAP_H_

#include <RTOSIface/RTOSIface.h>
#include <General/StringRef.h>
#include <Platform/MessageType.h>

class IndexSlot;
class StorageSpace;
class HeapBlock;
class IndexBlock;

class StringHandle
{
public:
	StringHandle() noexcept { slotPtr = nullptr; }
	StringHandle(const char *s) noexcept;
	StringHandle(const char *s, size_t len) noexcept;

	ReadLockedPointer<const char> Get() const noexcept;
	size_t GetLength() const noexcept;
	void Delete() noexcept;
	bool IsNull() const noexcept { return slotPtr == nullptr; }
	void Assign(const char *s) noexcept;

	static void GarbageCollect() noexcept;
//	static size_t GetWastedSpace() noexcept { return spaceToRecycle; }
//	static size_t GetIndexSpace() noexcept { return totalIndexSpace; }
//	static size_t GetHeapSpace() noexcept { return totalHeapSpace; }
	static bool CheckIntegrity(const StringRef& errmsg) noexcept;
	static void Diagnostics(MessageType mt) noexcept;

private:
	void InternalAssign(const char *s, size_t len) noexcept;
	void InternalDelete() noexcept;

	static IndexSlot *AllocateHandle() noexcept;
	static StorageSpace *AllocateSpace(uint16_t length) noexcept;
	static void ReleaseSpace(StorageSpace *ptr) noexcept;
	static void GarbageCollectInternal() noexcept;
	static void AdjustHandles(char *startAddr, char *endAddr, size_t moveDown, unsigned int numHandles) noexcept;

	IndexSlot *slotPtr;

	static ReadWriteLock heapLock;
	static IndexBlock *indexRoot;
	static HeapBlock *heapRoot;
	static size_t spaceToRecycle;
	static size_t totalIndexSpace;
	static size_t totalHeapSpace;
	static unsigned int gcCyclesDone;
};

#endif /* SRC_PLATFORM_HEAP_H_ */
