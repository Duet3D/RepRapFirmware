/*
 * StringHandle.h
 *
 *  Created on: 12 Jun 2022
 *      Author: David
 */

#ifndef SRC_PLATFORM_STRINGHANDLE_H_
#define SRC_PLATFORM_STRINGHANDLE_H_

#include <RepRapFirmware.h>
#include "Heap.h"

// Note: StringHandle is a union member in ExpressionValue, therefore it must be no larger than 32 bits and it cannot have a non-trivial destructor, copy constructor etc.
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

protected:
	void InternalAssign(const char *s, size_t len) noexcept;

	Heap::IndexSlot * null slotPtr;
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

#endif /* SRC_PLATFORM_STRINGHANDLE_H_ */
