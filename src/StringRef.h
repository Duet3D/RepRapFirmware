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

#undef printf

// Class to describe a string buffer, including its length. This saves passing buffer lengths around everywhere.
class StringRef
{
	char *p;		// pointer to the storage
	size_t len;		// number of characters in the storage

public:
	StringRef(char *pp, size_t pl) : p(pp), len(pl) { }

	size_t Length() const { return len; }
	size_t strlen() const;
	char *Pointer() { return p; }
	const char *Pointer() const { return p; }

	char& operator[](size_t index) { return p[index]; }
	char operator[](size_t index) const { return p[index]; }

	void Clear() { p[0] = 0; }

	int printf(const char *fmt, ...);
	int vprintf(const char *fmt, va_list vargs);
	int catf(const char *fmt, ...);
	size_t copy(const char* src);
	size_t cat(const char *src);
};

#endif /* STRINGREF_H_ */
