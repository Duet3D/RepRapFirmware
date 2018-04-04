/*
 * strtod.cpp
 *
 *  Created on: 4 Apr 2018
 *      Author: David
 *
 * This is a replacement for strtof() in the C standard library. That version has two problems:
 * 1. It is not reentrant. We can make it so by defining configUSE_NEWLIB_REENTRANT in FreeRTOS, but that makes the tasks much bigger.
 * 2. It allocates and releases heap memory, which is not nice.
 *
 * Limitations of this version
 * 1. Rounding to nearest float may not always be correct.
 * 2. Doesn't handle more than 9 digits (excluding any leading zeros) before the decimal point. Numbers greater than 2^64-1 or less then -2^64-1
 *    will be incorrectly converted unless expressed in scientific notation.
 */

#include <cstdlib>		// to pull in the standard declarations of strtod etc.
#include <cctype>
#include <cmath>
#include <climits>

extern "C" double strtod(const char *s, char **p)
{
	// 1. Check for a sign
	const bool negative = (*s == '-');
	if (negative || *s == '+')
	{
		++s;
	}

	// 2. Read digits before decimal point, E or e. We use floating point for this in case of very large numbers.
	double valueBeforePoint = 0;
	while (isdigit(*s))
	{
		valueBeforePoint = (valueBeforePoint * 10) + (*s - '0');
		++s;
	}

	// 3. Check for decimal point before E or e
	unsigned long valueAfterPoint = 0;
	long digitsAfterPoint = 0;
	if (*s == '.')
	{
		++s;

		// 3b. Read the digits (if any) after the decimal point
		bool overflowed = false;
		while (isdigit(*s))
		{
			if (!overflowed)
			{
				const unsigned int digit = *s - '0';
				if (valueAfterPoint <= (ULONG_MAX - digit)/10)
				{
					valueAfterPoint = (valueAfterPoint * 10) + digit;
					++digitsAfterPoint;
				}
				else
				{
					overflowed = true;
					if (digit >= 5 && valueAfterPoint != ULONG_MAX)
					{
						++valueAfterPoint;			// do approximate rounding
					}
				}
			}
			++s;
		}
	}

	// 5. Check for exponent part
	long exponent = 0;
	if (toupper(*s) == 'E')
	{
		++s;

		// 5a. Check for signed exponent
		const bool expNegative = (*s == '-');
		if (expNegative || *s == '+')
		{
			++s;
		}

		// 5b. Read exponent digits
		while (isdigit(*s))
		{
			exponent = (10 * exponent) + (*s - '0');	// could overflow, but anyone using such large numbers is being very silly
			++s;
		}

		if (expNegative)
		{
			exponent = -exponent;
		}
	}

	// 6. Compute the composite value
	double retvalue;
	if (valueAfterPoint != 0)
	{
		if (valueBeforePoint == 0)
		{
			retvalue = (double)valueAfterPoint * pow(10, exponent - digitsAfterPoint);
		}
		else
		{
			retvalue = ((double)valueAfterPoint/pow(10, digitsAfterPoint) + valueBeforePoint) * pow(10, exponent);
		}
	}
	else
	{
		retvalue = valueBeforePoint * pow(10, exponent);
	}

	// 7. Set end pointer
	if (p != nullptr)
	{
		*p = const_cast<char *>(s);
	}

	// 8. Adjust sign if necessary
	return (negative) ? -retvalue : retvalue;
}

extern "C" float strtof(const char *s, char **p)
{
	return (float)strtod(s, p);
}

// We need to define this one too because it is called internally, probably by sscanf
extern "C" double _strtod_r(struct _reent *r, const char *s, char **p)
{
	return strtod(s, p);
}

// End
