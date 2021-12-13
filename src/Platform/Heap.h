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

class IndexSlot;
class StorageSpace;
class HeapBlock;
class IndexBlock;

// Note: StringHandle is a union member in ExpressionValue, therefore it cannot have a non-trivial destructor, copy constructor etc.
// This means that when an object containing a StringHandle is copied or destroyed, that object must handle the reference count.
// Classes other than ExpressionValue should use AutoStringHandle instead;
class StringHandle
{
public:
	StringHandle() noexcept { slotPtr = nullptr; }
	explicit StringHandle(const char *s) noexcept;
	StringHandle(const char *s, size_t len) noexcept;

#if 0	// unused
	StringHandle(const char *s1, const char *s2) noexcept;
#endif

	ReadLockedPointer<const char> Get() const noexcept;
	size_t GetLength() const noexcept;
	void Delete() noexcept;
	const StringHandle& IncreaseRefCount() const noexcept;
	bool IsNull() const noexcept { return slotPtr == nullptr; }
	void Assign(const char *s) noexcept;

	static void GarbageCollect() noexcept;
//	static size_t GetWastedSpace() noexcept { return spaceToRecycle; }
//	static size_t GetIndexSpace() noexcept { return totalIndexSpace; }
//	static size_t GetHeapSpace() noexcept { return totalHeapSpace; }
	static bool CheckIntegrity(const StringRef& errmsg) noexcept;
	static void Diagnostics(MessageType mt, Platform& p) noexcept;

protected:
	void InternalAssign(const char *s, size_t len) noexcept;
	void InternalDelete() noexcept;

	static IndexSlot *AllocateHandle() noexcept;
	static StorageSpace *AllocateSpace(size_t length) noexcept;
	static void GarbageCollectInternal() noexcept;
	static void AdjustHandles(char *startAddr, char *endAddr, size_t moveDown, unsigned int numHandles) noexcept;

	IndexSlot * null slotPtr;

	static ReadWriteLock heapLock;
	static IndexBlock *indexRoot;
	static HeapBlock *heapRoot;
	static size_t handlesAllocated;
	static std::atomic<size_t> handlesUsed;
	static size_t heapAllocated;
	static size_t heapUsed;
	static std::atomic<size_t> heapToRecycle;
	static unsigned int gcCyclesDone;
};

// Version of StringHandle that updates the reference counts automatically
class AutoStringHandle : public StringHandle
{
public:
	AutoStringHandle() noexcept : StringHandle() { }
	explicit AutoStringHandle(const char *s) noexcept : StringHandle(s) { }
	AutoStringHandle(const char *s, size_t len) noexcept : StringHandle(s, len) { }
	AutoStringHandle(const AutoStringHandle& other) noexcept;
	AutoStringHandle(AutoStringHandle&& other) noexcept;
	AutoStringHandle& operator=(const AutoStringHandle& other) noexcept;
	~AutoStringHandle();
};

#endif /* SRC_PLATFORM_HEAP_H_ */
