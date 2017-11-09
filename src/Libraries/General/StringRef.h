/*
 * StringRef.h
 *
 *  Created on: 10 Jan 2016
 *      Author: David
 */

#ifndef STRINGREF_H_
#define STRINGREF_H_

#include <cstddef>	// for size_t
#include <cstdarg>	// for va_args
#include <cstring>	// for strlen

// Need to declare strnlen here because it isn't ISO standard
size_t strnlen(const char *s, size_t n);

// Class to describe a string buffer, including its length. This saves passing buffer lengths around everywhere.
class StringRef
{
	char *p;		// pointer to the storage
	size_t len;		// number of characters in the storage

public:
	StringRef(char *pp, size_t pl) : p(pp), len(pl) { p[0] = 0; }

	size_t Length() const { return len; }
	size_t strlen() const;
	bool IsEmpty() const { return p[0] == 0; }
	char *Pointer() { return p; }
	const char *Pointer() const { return p; }

	char& operator[](size_t index) { return p[index]; }
	char operator[](size_t index) const { return p[index]; }

	void Clear() const { p[0] = 0; }

	int printf(const char *fmt, ...) const __attribute__ ((format (printf, 2, 3)));
	int vprintf(const char *fmt, va_list vargs) const;
	int catf(const char *fmt, ...) const __attribute__ ((format (printf, 2, 3)));
	int vcatf(const char *fmt, va_list vargs) const;
	size_t copy(const char* src) const;
	size_t cat(const char *src) const;
	size_t cat(char c) const;
	size_t StripTrailingSpaces() const;
	size_t Prepend(const char *src) const;

};

// Class to describe a string which we can get a StringRef reference to
template<size_t Len> class String
{
public:
	String() { storage[0] = 0; }

	StringRef GetRef() { return StringRef(storage, Len + 1); }
	const char *c_str() const { return storage; }
	size_t strlen() const { return strnlen(storage, Len); }
	bool IsEmpty() const { return storage[0] == 0; }
	const char *Pointer() const { return storage; }
	char& operator[](size_t index) { return storage[index]; }
	char operator[](size_t index) const { return storage[index]; }

	void Clear() const { storage[0] = 0; }

private:
	char storage[Len + 1];
};

#endif /* STRINGREF_H_ */
