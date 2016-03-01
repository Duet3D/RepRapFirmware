/*
 * StringRef.cpp
 *
 *  Created on: 10 Jan 2016
 *      Author: David
 */

#include "StringRef.h"
#include <cstring>
#include <cstdio>
#undef printf			// some idiot defined printf as a macro inside cstdio, which prevents us using it as a member function name

//*************************************************************************************************
// StringRef class member implementations

size_t StringRef::strlen() const
{
	return strnlen(p, len - 1);
}

int StringRef::printf(const char *fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	int ret = vsnprintf(p, len, fmt, vargs);
	va_end(vargs);
	return ret;
}

int StringRef::vprintf(const char *fmt, va_list vargs)
{
	return vsnprintf(p, len, fmt, vargs);
}

int StringRef::catf(const char *fmt, ...)
{
	size_t n = strlen();
	if (n + 1 < len)		// if room for at least 1 more character and a null
	{
		va_list vargs;
		va_start(vargs, fmt);
		int ret = vsnprintf(p + n, len - n, fmt, vargs);
		va_end(vargs);
		return ret + n;
	}
	return 0;
}

// This is quicker than printf for printing constant strings
size_t StringRef::copy(const char* src)
{
	size_t length = strnlen(src, len - 1);
	memcpy(p, src, length);
	p[length] = 0;
	return length;
}

// This is quicker than catf for printing constant strings
size_t StringRef::cat(const char* src)
{
	size_t length = strlen();
	size_t toCopy = strnlen(src, len - length - 1);
	memcpy(p + length, src, toCopy);
	length += toCopy;
	p[length] = 0;
	return length;
}

// End




