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
#include "SafeVsnprintf.h"

#ifdef RTOS
# include "RTOSIface.h"
#endif

//*************************************************************************************************
// StringRef class member implementations

size_t StringRef::strlen() const
{
	return Strnlen(p, len - 1);
}

int StringRef::printf(const char *fmt, ...) const
{
	va_list vargs;
	va_start(vargs, fmt);
	const int ret = SafeVsnprintf(p, len, fmt, vargs);
	va_end(vargs);
	return ret;
}

int StringRef::vprintf(const char *fmt, va_list vargs) const
{
	return SafeVsnprintf(p, len, fmt, vargs);
}

int StringRef::catf(const char *fmt, ...) const
{
	const size_t n = strlen();
	if (n + 1 < len)		// if room for at least 1 more character and a null
	{
		va_list vargs;
		va_start(vargs, fmt);
		const int ret = SafeVsnprintf(p + n, len - n, fmt, vargs);
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
		return SafeVsnprintf(p + n, len - n, fmt, vargs) + n;
	}
	return 0;
}

// This is quicker than printf for printing constant strings
bool StringRef::copy(const char* src) const
{
	const size_t slen = ::strlen(src);
	const bool overflow = (slen >= len);
	const size_t length = (overflow) ? len - 1 : slen;
	memcpy(p, src, length);
	p[length] = 0;
	return overflow;
}

// This is quicker than catf for printing constant strings
bool StringRef::cat(const char* src) const
{
	const size_t length = strlen();
	const size_t slen = ::strlen(src);
	const bool overflow = (length + slen >= len);
	const size_t toCopy = (overflow) ? len - length - 1 : slen;
	memcpy(p + length, src, toCopy);
	p[length + toCopy] = 0;
	return overflow;
}

// Append a character
bool StringRef::cat(char c) const
{
	const size_t length = strlen();
	if (length + 1 < len)
	{
		p[length] = c;
		p[length + 1] = 0;
		return false;
	}
	return true;
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

bool StringRef::Prepend(const char *src) const
{
	const size_t slen = ::strlen(src);
	const size_t dlen = strlen();
	if (slen + dlen < len)
	{
		memmove(p + slen, p, dlen + 1);
		memcpy(p, src, slen);
		return false;
	}
	return true;
}

// End
