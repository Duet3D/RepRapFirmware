/*
 * vsnprintf.cpp
 *
 *  Created on: 8 Apr 2018

	Original copyright 2001, 2002 Georges Menie (www.menie.org)
	stdarg version contributed by Christian Ettinger
	Converted to C++ and adapted to support floating point formats by D. Crocker

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	Changes for the FreeRTOS ports:

	- The dot in "%-8.8s"
	- The specifiers 'l' (long) and 'L' (long long)
	- The specifier 'u' for unsigned
	- Dot notation for IP addresses:
	  sprintf("IP = %xip\n", 0xC0A80164);
      will produce "IP = 192.168.1.100\n"
	  sprintf("IP = %pip\n", pxIPv6_Address);
*/

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <cmath>

#include "Strnlen.h"

// The following should be enough for 32-bit int/long and 64-bit long long
constexpr size_t MaxLongDigits = 10;	// to print 4294967296
constexpr size_t MaxUllDigits = 20;		// to print 18446744073709551616

struct xPrintFlags
{
	int base;
	int width;
	int printLimit;
	unsigned int
		letBase : 8,
		padZero : 1,
		padRight : 1,
		isSigned : 1,
		isNumber : 1,
		isString : 1,
		long32 : 1,
		long64 : 1;
};

struct SStringBuf
{
	char *str;
	const char *orgStr;
	const char *nulPos;
	int curLen;
	struct xPrintFlags flags;

	SStringBuf(char *s, size_t maxLen);
	void Init();
};

SStringBuf::SStringBuf(char *apBuf, size_t maxLen)
{
	str = apBuf;
	orgStr = apBuf;
	nulPos = apBuf + maxLen - 1;
	curLen = 0;
	Init();
}

void SStringBuf::Init()
{
	memset(&flags, 0, sizeof(flags));
}

/*-----------------------------------------------------------*/

// Store the specified character in the string buffer.
// If it won't fit leaving room for a null, store a null and return false.
// If it is null, store it and return false.
// Else store it and return true.
static bool strbuf_printchar(SStringBuf& apStr, char c)
{
	if (c != 0 && apStr.str < apStr.nulPos)
	{
		*apStr.str++ = c;
		apStr.curLen++;
		return true;
	}
	*apStr.str = '\0';
	return false;
}

/*-----------------------------------------------------------*/

// Print the string s to the string buffer adding any necessary padding
static bool prints(SStringBuf& apBuf, const char *apString )
{
	int count;
	if (apBuf.flags.printLimit > 0 && apBuf.flags.isString)
	{
		// It's a string so printLimit is the max number of characters to print from the string.
		// Don't call strlen on it because it might not be null terminated, use Strnlen instead.
		count = (int)Strnlen(apString, apBuf.flags.printLimit);
	}
	else
	{
		count = (int)strlen(apString);
	}

	int rightSpacesNeeded = 0;
	const bool hasMinimumDigits = (apBuf.flags.isNumber && apBuf.flags.printLimit > 0);
	if (hasMinimumDigits || apBuf.flags.width > 0)
	{
		// We may have some padding to do
		int leftSpacesNeeded = 0, leftZerosNeeded = 0;
		if (hasMinimumDigits && count < apBuf.flags.printLimit)
		{
			leftZerosNeeded = apBuf.flags.printLimit - count;
		}
		if (count + leftZerosNeeded < apBuf.flags.width)
		{
			const int remainingPaddingNeeded = apBuf.flags.width - (count + leftZerosNeeded);
			if (apBuf.flags.padRight)
			{
				rightSpacesNeeded = remainingPaddingNeeded;
			}
			else if (apBuf.flags.padZero)
			{
				leftZerosNeeded += remainingPaddingNeeded;
			}
			else
			{
				leftSpacesNeeded = remainingPaddingNeeded;
			}
		}

		// Do the left padding
		while (leftSpacesNeeded > 0)
		{
			if (!strbuf_printchar(apBuf, ' '))
			{
				return false;
			}
			--leftSpacesNeeded;
		}
		while (leftZerosNeeded > 0)
		{
			if (!strbuf_printchar(apBuf, '0'))
			{
				return false;
			}
			--leftZerosNeeded;
		}
	}

	// Now print the actual string
	while (count > 0)
	{
		if (!strbuf_printchar(apBuf, *apString++))
		{
			return false;
		}
		--count;
	}

	// Now the right padding
	while (rightSpacesNeeded > 0)
	{
		if (!strbuf_printchar(apBuf, ' '))
		{
			return false;
		}
		--rightSpacesNeeded;
	}

	return true;
}

/*-----------------------------------------------------------*/

static bool printll(SStringBuf& apBuf, long long i)
{
	apBuf.flags.isNumber = true;	/* Parameter for prints */
	if (i == 0LL)
	{
		return prints(apBuf, "0");
	}

	bool neg = false;
	unsigned long long u = i;
	if ((apBuf.flags.isSigned) && (apBuf.flags.base == 10) && (i < 0LL))
	{
		neg = true;
		u = -i;
	}

	char print_buf[MaxUllDigits + 2];
	char *s = print_buf + sizeof print_buf - 1;
	*s = '\0';
	while (u != 0)
	{
		const lldiv_t lldiv_result = lldiv(u, (long long)apBuf.flags.base);
		unsigned int t = lldiv_result.rem;
		if (t >= 10)
		{
			t += apBuf.flags.letBase - '0' - 10;
		}
		*--s = t + '0';
		u = lldiv_result.quot;
	}

	if (neg)
	{
		if (apBuf.flags.width != 0 && apBuf.flags.padZero)
		{
			if (!strbuf_printchar(apBuf, '-'))
			{
				return false;
			}
			--apBuf.flags.width;
		}
		else
		{
			*--s = '-';
		}
	}

	return prints(apBuf, s);
}

/*-----------------------------------------------------------*/

static bool printi(SStringBuf& apBuf, int i)
{
	apBuf.flags.isNumber = true;	/* Parameter for prints */

	if (i == 0)
	{
		return prints(apBuf, "0");
	}

	bool neg = false;
	unsigned int u = i;
	unsigned base = apBuf.flags.base;
	if ((apBuf.flags.isSigned) && (base == 10) && (i < 0))
	{
		neg = true;
		u = -i;
	}

	char print_buf[MaxLongDigits + 2];
	char *s = print_buf + sizeof print_buf - 1;
	*s = '\0';

	switch (base)
	{
	case 16:
		while (u != 0)
		{
			unsigned int t = u & 0xF;
			if (t >= 10)
			{
				t += apBuf.flags.letBase - '0' - 10;
			}
			*--s = t + '0';
			u >>= 4;
		}
		break;

	case 8:
	case 10:
		// GCC compiles very efficient
		while (u != 0)
		{
			const unsigned int t = u % base;
			*--s = t + '0';
			u /= base;
		}
		break;
#if 0
	// The generic case, not yet in use
	default:
		while (u != 0)
		{
			const unsigned int t = u % base;
			if (t >= 10)
			{
				t += apBuf.flags.letBase - '0' - 10;
			}
			*--s = t + '0';
			u /= base;
		}
		break;
#endif
	}

	if (neg)
	{
		if (apBuf.flags.width && apBuf.flags.padZero)
		{
			if (!strbuf_printchar(apBuf, '-'))
			{
				return false;
			}
			--apBuf.flags.width;
		}
		else
		{
			*--s = '-';
		}
	}

	return prints(apBuf, s);
}

/*-----------------------------------------------------------*/

// Print a number in scientific format
// apBuf.flags.printLimit is the number of decimal digits required
static bool printFloat(SStringBuf& apBuf, double d, char formatLetter)
{
	if (std::isnan(d))
	{
		return prints(apBuf, "nan");
	}
	if (std::isinf(d))
	{
		return prints(apBuf, "inf");
	}

	double ud = fabs(d);
	if (ud > (double)LONG_LONG_MAX && (formatLetter == 'f' || formatLetter == 'F'))
	{
		--formatLetter;			// number is too big to print easily in fixed point format, so use exponent format
	}

	int exponent = 0;
	if (formatLetter == 'e' || formatLetter == 'E')
	{
		// Using exponent format, so calculate the exponent and normalise ud to be >=1.0 but <=10.0
		// The following loops are inefficient, however we don't expect to print very large or very small numbers
		while (ud > (double)100000.0)
		{
			ud /= (double)100000.0;
			exponent += 5;
		}
		while (ud > (double)10.0)
		{
			ud /= (double)10.0;
			++exponent;
		}
		if (ud != (double)0.0)
		{
			while (ud < (double)0.00001)
			{
				ud *= (double)100000.0;
				exponent -= 5;
			}
			while (ud < (double)1.0)
			{
				ud *= (double)10.0;
				--exponent;
			}
		}
		// ud is now at least 1.0 but less than 10.0 and exponent is the exponent
	}

	// Multiply ud by 10 to the power of the number of decimal digits required, or until it becomes too big to print easily
	if (apBuf.flags.printLimit < 0)
	{
		apBuf.flags.printLimit = 6;					// set the default number of decimal digits
	}
	int digitsAfterPoint = 0;
	long limit = 10;
	while (digitsAfterPoint < apBuf.flags.printLimit && ud < LONG_LONG_MAX/10 && limit <= LONG_LONG_MAX/10)
	{
		ud *= (double)10.0;
		limit *= 10;
		 ++digitsAfterPoint;
	}

	char print_buf[MaxUllDigits + MaxLongDigits + 5];
	char *s = print_buf + sizeof print_buf - 1;
	*s = '\0';

	long long u = llrint(ud);

	if (formatLetter == 'e' || formatLetter == 'E')
	{
		// Rounding ud may have caused 9.99999... to become 10
		if (ud >= limit)
		{
			ud /= 10;
			++exponent;
		}

		// Store the exponent
		int iexp = abs(exponent);
		do
		{
			*--s = (iexp % 10) + '0';
			iexp = iexp/10;
		} while (iexp != 0);
		*--s = (exponent < 0) ? '-' : '+';
		*--s = formatLetter;
	}

	// Store the non-exponent part
	do
	{
		if (digitsAfterPoint == 0)
		{
			*--s = '.';
		}
		--digitsAfterPoint;

		const lldiv_t lldiv_result = lldiv(u, 10);
		*--s = (char)((unsigned int)lldiv_result.rem + '0');
		u = lldiv_result.quot;
	}
	while (u != 0 || digitsAfterPoint >= 0);

	if (d < (double)0.0)
	{
		if (apBuf.flags.width != 0 && apBuf.flags.padZero)
		{
			if (!strbuf_printchar(apBuf, '-'))
			{
				return false;
			}
			--apBuf.flags.width;
		}
		else
		{
			*--s = '-';
		}
	}

	return prints(apBuf, s);
}

/*-----------------------------------------------------------*/

static void tiny_print(SStringBuf& apBuf, const char *format, va_list args)
{
	for (;;)
	{
		char ch;
		while ((ch = *format++) != '%')
		{
			if (!strbuf_printchar(apBuf, ch))		// note: this returns false if ch == 0
			{
				return;
			}
		}

		// If we get here then ch == '%'. Get the next character.
		ch = *format++;
		if (ch == '\0')
		{
			break;
		}
		if (ch == '%')
		{
			if (strbuf_printchar(apBuf, ch) == 0)
			{
				return;
			}
			continue;
		}

		apBuf.Init();

		if (ch == '-')
		{
			ch = *format++;
			apBuf.flags.padRight = true;
		}
		while (ch == '0')
		{
			ch = *format++;
			apBuf.flags.padZero = true;
		}
		if (ch == '*')
		{
			ch = *format++;
			apBuf.flags.width = va_arg(args, int);
		}
		else
		{
			while(ch >= '0' && ch <= '9')
			{
				apBuf.flags.width *= 10;
				apBuf.flags.width += ch - '0';
				ch = *format++;
			}
		}
		if (ch == '.')
		{
			ch = *format++;
			if (ch == '*')
			{
				apBuf.flags.printLimit = va_arg(args, int);
				ch = *format++;
			}
			else
			{
				while (ch >= '0' && ch <= '9')
				{
					apBuf.flags.printLimit *= 10;
					apBuf.flags.printLimit += ch - '0';
					ch = *format++;
				}
			}
		}
		if (apBuf.flags.printLimit == 0)
		{
			apBuf.flags.printLimit = -1;		// -1: make it unlimited
		}
		if (ch == 's')
		{
			const char *s = va_arg(args, const char *);
			apBuf.flags.isString = true;
			if (!prints(apBuf, (s != nullptr) ? s : "(null)"))
			{
				break;
			}
			continue;
		}
		if (ch == 'c')
		{
			// char are converted to int then pushed on the stack
			if (!strbuf_printchar(apBuf, (char)va_arg(args, int)))
			{
				return;
			}

			continue;
		}
		if (ch == 'l')
		{
			ch = *format++;
			apBuf.flags.long32 = 1;
			// Makes no difference as u32 == long
		}
		if (ch == 'L')
		{
			ch = *format++;
			apBuf.flags.long64 = 1;
			// Does make a difference
		}

		if (ch == 'f' || ch == 'e' || ch == 'F' || ch == 'E')
		{
			if (!printFloat(apBuf, va_arg(args, double), ch))
			{
				break;
			}
			continue;
		}

		apBuf.flags.base = 10;
		apBuf.flags.letBase = 'a';

		if (ch == 'd' || ch == 'u' || ch == 'i')
		{
			apBuf.flags.isSigned = (ch != 'u');
			if (apBuf.flags.long64)
			{
				if (!printll(apBuf, va_arg(args, long long)))
				{
					break;
				}
			}
			else if (!printi(apBuf, va_arg(args, int)))
			{
				break;
			}
			continue;
		}

		apBuf.flags.base = 16;		// from here all hexadecimal
		if (ch == 'x' || ch == 'X' || ch == 'p' || ch == 'o')
		{
			if (ch == 'X')
			{
				apBuf.flags.letBase = 'A';
			}
			else if (ch == 'o')
			{
				apBuf.flags.base = 8;
			}
			if (apBuf.flags.long64)
			{
				if (!printll(apBuf, va_arg(args, long long)))
				{
					break;
				}
			}
			else if (!printi(apBuf, va_arg(args, int)))
			{
				break;
			}
			continue;
		}
	}
	strbuf_printchar(apBuf, '\0');
}

/*-----------------------------------------------------------*/

int SafeVsnprintf(char *apBuf, size_t aMaxLen, const char *apFmt, va_list args)
{
	SStringBuf strBuf(apBuf, aMaxLen);
	tiny_print(strBuf, apFmt, args);
	return strBuf.curLen;
}

int SafeSnprintf(char* buffer, size_t buf_size, const char* format, ...)
{
	va_list vargs;
	va_start(vargs, format);
	const int ret = SafeVsnprintf(buffer, buf_size, format, vargs);
	va_end(vargs);
	return ret;
}

// End
