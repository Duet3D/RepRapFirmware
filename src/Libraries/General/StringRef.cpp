/*
 * StringRef.cpp
 *
 *  Created on: 10 Jan 2016
 *      Author: David
 */

#include "StringRef.h"
#include <cstring>
#include <cstdio>
#include "WMath.h"

// Need to define strnlen here because it isn't ISO standard
size_t strnlen(const char *s, size_t n)
{
	size_t rslt = 0;
	while (rslt < n && s[rslt] != 0)
	{
		++rslt;
	}
	return rslt;
}

//*************************************************************************************************
// StringRef class member implementations

size_t StringRef::strlen() const
{
	return strnlen(p, len - 1);
}

int StringRef::printf(const char *fmt, ...) const
{
	va_list vargs;
	va_start(vargs, fmt);
	const int ret = vsnprintf(p, len, fmt, vargs);
	va_end(vargs);
	return ret;
}

int StringRef::vprintf(const char *fmt, va_list vargs) const
{
	return vsnprintf(p, len, fmt, vargs);
}

int StringRef::catf(const char *fmt, ...) const
{
	const size_t n = strlen();
	if (n + 1 < len)		// if room for at least 1 more character and a null
	{
		va_list vargs;
		va_start(vargs, fmt);
		const int ret = vsnprintf(p + n, len - n, fmt, vargs);
		va_end(vargs);
		return ret + n;
	}
	return 0;
}

int StringRef::vcatf(const char *fmt, va_list vargs) const
{
	const size_t n = strlen();
	if (n + 1 < len)		// if room for at least 1 more character and a null
	{
		return vsnprintf(p + n, len - n, fmt, vargs) + n;
	}
	return 0;
}

// This is quicker than printf for printing constant strings
size_t StringRef::copy(const char* src) const
{
	const size_t length = strnlen(src, len - 1);
	memcpy(p, src, length);
	p[length] = 0;
	return length;
}

// This is quicker than catf for printing constant strings
size_t StringRef::cat(const char* src) const
{
	size_t length = strlen();
	const size_t toCopy = strnlen(src, len - length - 1);
	memcpy(p + length, src, toCopy);
	length += toCopy;
	p[length] = 0;
	return length;
}

// Append a character and return the resulting length
size_t StringRef::cat(char c) const
{
	size_t length = strlen();
	if (length + 1 < len)
	{
		p[length++] = c;
		p[length] = 0;
	}
	return length;
}

// Remove trailing spaces from the string and return its new length
size_t StringRef::StripTrailingSpaces() const
{
	size_t slen = strlen();
	while (slen != 0 && p[slen - 1] == ' ')
	{
		--slen;
		p[slen] = 0;
	}
	return slen;
}

size_t StringRef::Prepend(const char *src) const
{
	const size_t slen = min<size_t>(::strlen(src), len - 1);
	const size_t dlen = strlen();
	const size_t newLen = min<size_t>(dlen + slen, len - 1);
	memmove(p + slen, p, newLen - slen + 1);
	memcpy(p, src, slen);
	return newLen;
}

// End




