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

#include "Strnlen.h"

// Class to describe a string buffer, including its length. This saves passing buffer lengths around everywhere.
class StringRef
{
	char *p;				// pointer to the storage
	size_t len;				// number of characters in the storage
public:
	StringRef(char *pp, size_t pl) : p(pp), len(pl) { }

	size_t Capacity() const { return len - 1; }
	size_t strlen() const;
	bool IsEmpty() const { return p[0] == 0; }

	const char *c_str() const { return p; }
	char *Pointer() const { return p; }						// use Pointer() only in the very care case that we need direct write access to the storage!

	char& operator[](size_t index) const { return p[index]; }

	void Clear() const { p[0] = 0; }

	int printf(const char *fmt, ...) const __attribute__ ((format (printf, 2, 3)));
	int vprintf(const char *fmt, va_list vargs) const;
	int catf(const char *fmt, ...) const __attribute__ ((format (printf, 2, 3)));
	int vcatf(const char *fmt, va_list vargs) const;
	bool copy(const char* src) const;						// returns true if buffer is too small
	bool cat(const char *src) const;						// returns true if buffer is too small
	bool cat(char c) const;									// returns true if buffer is too small
	size_t StripTrailingSpaces() const;
	bool Prepend(const char *src) const;					// returns true if buffer is too small
};

// Class to describe a string which we can get a StringRef reference to
template<size_t Len> class String
{
public:
	String() { storage[0] = 0; }

	StringRef GetRef() { return StringRef(storage, Len + 1); }
	const char *c_str() const { return storage; }
	size_t strlen() const { return Strnlen(storage, Len); }
	bool IsEmpty() const { return storage[0] == 0; }
//	char *Pointer() { return storage; }
	char& operator[](size_t index) { return storage[index]; }
	char operator[](size_t index) const { return storage[index]; }
	constexpr size_t Capacity() const { return Len; }
	bool EndsWith(char c) const;

	void Clear() { storage[0] = 0; }
	int printf(const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
	int vprintf(const char *fmt, va_list vargs);
	int catf(const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
	int vcatf(const char *fmt, va_list vargs);
	bool copy(const char *src) { return GetRef().copy(src); }	// returns true if buffer is too small
	bool cat(const char *src) { return GetRef().cat(src); }		// returns true if buffer is too small
	bool cat(char c) { return GetRef().cat(c); }				// returns true if buffer is too small
	bool Prepend(const char *src) const;						// returns true if buffer is too small

	void CopyAndPad(const char *src);
	bool ConstantTimeEquals(String<Len> other) const;

	void Truncate(size_t len);

private:
	char storage[Len + 1];
};

// Copy some text into this string and pad it with nulls so we can do a constant time compare
template<size_t Len> void String<Len>::CopyAndPad(const char* src)
{
	memset(storage, 0, Len + 1);
	copy(src);
}

// Do a constant time compare. Both this string and the other one much be padded with nulls.
template<size_t Len> bool String<Len>::ConstantTimeEquals(String<Len> other) const
{
	char rslt = 0;
	for (size_t i = 0; i < Len; ++i)
	{
		rslt |= (storage[i] ^ other.storage[i]);
	}
	return rslt == 0;
}

template<size_t Len> inline int String<Len>::vprintf(const char *fmt, va_list vargs)
{
	return GetRef().vprintf(fmt, vargs);
}

template<size_t Len> inline int String<Len>::vcatf(const char *fmt, va_list vargs)
{
	return GetRef().vcatf(fmt, vargs);
}

template<size_t Len> int String<Len>::printf(const char *fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	const int ret = GetRef().vprintf(fmt, vargs);
	va_end(vargs);
	return ret;
}

template<size_t Len> int String<Len>::catf(const char *fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	const int ret = GetRef().vcatf(fmt, vargs);
	va_end(vargs);
	return ret;
}

template<size_t Len> void String<Len>::Truncate(size_t len)
{
	if (len < strlen())
	{
		storage[len] = 0;
	}
}

template<size_t Len> bool String<Len>::EndsWith(char c) const
{
	const size_t len = strlen();
	return len != 0 && storage[len - 1] == c;
}

#endif /* STRINGREF_H_ */
