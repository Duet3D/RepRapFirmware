/*
 * Copyright (c) 1999 Kungliga Tekniska H�gskolan
 * (Royal Institute of Technology, Stockholm, Sweden).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of KTH nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY KTH AND ITS CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL KTH OR ITS CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/*
 * DC 2020/01/10: removed code for those format specifiers that need a locale and others that we don't need,
 *  such as GNU-specific format specifiers, AM/PM specifiers, 12-hour clock specifiers, 2-digit year specifiers, and month and day number specifiers.
 * The only format specifiers currently used by RepRapFirmware are: YmdHMS
 */

#define _GNU_SOURCE
#include <ecv_duet3d.h>
#include <stddef.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <stdlib.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>

#include <General/SafeStrtod.h>

static constexpr int16_t _DAYS_BEFORE_MONTH[12] =
{0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define SET_MDAY 1
#define SET_MON  2
#define SET_YEAR 4
#define SET_WDAY 8
#define SET_YDAY 16
#define SET_YMD  (SET_YEAR | SET_MON | SET_MDAY)

/*
 * tm_year is relative this year
 */
constexpr int tm_year_base = 1900;

/*
 * Return TRUE iff `year' was a leap year.
 * Needed for strptime.
 */
static int is_leap_year (int year) noexcept
{
    return (year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0);
}

/* Needed for strptime. */
static int first_day (int year) noexcept
{
    int ret = 4;

    while (--year >= 1970)
    {
    	ret = (ret + 365 + is_leap_year (year)) % 7;
    }
    return ret;
}

#undef 	isspace_l
#define isspace_l(_c, _l)			isspace(_c)
#define strtol_l(_b, _s, _n, _l)	StrToI32(_b, _s)

const char *SafeStrptime(const char *_ecv_array buf, const char *_ecv_array format, struct tm *timeptr) noexcept
{
    char c;
    uint8_t ymd = 0;

    for (; (c = *format) != '\0'; ++format)
    {
		if (isspace_l ((unsigned char) c, locale))
		{
			while (isspace_l ((unsigned char) *buf, locale))
			++buf;
		}
		else if (c == '%' && format[1] != '\0')
		{
			c = *++format;
			if (c == 'E' || c == 'O')
			{
				c = *++format;
			}

			switch (c)
			{
			case 'd' :
			case 'e' :
				{
					const char *s;
					const int ret = strtol_l (buf, &s, 10, locale);
					if (s == buf) { return nullptr; }
					timeptr->tm_mday = ret;
					buf = s;
					ymd |= SET_MDAY;
				}
				break;

			case 'H' :
				{
					const char *s;
					const int ret = strtol_l (buf, &s, 10, locale);
					if (s == buf) { return nullptr; }
					timeptr->tm_hour = ret;
					buf = s;
				}
				break;

			case 'm' :
				{
					const char *s;
					const int ret = strtol_l (buf, &s, 10, locale);
					if (s == buf) { return nullptr; }
					timeptr->tm_mon = ret - 1;
					buf = s;
					ymd |= SET_MON;
				}
				break;

			case 'M' :
				{
					const char *s;
					const int ret = strtol_l (buf, &s, 10, locale);
					if (s == buf) { return nullptr; }
					timeptr->tm_min = ret;
					buf = s;
				}
				break;

			case 'n' :
				if (*buf == '\n')
				{
					++buf;
				}
				else
				{
					return nullptr;
				}
				break;

			case 'S' :
				{
					const char *s;
					const int ret = strtol_l (buf, &s, 10, locale);
					if (s == buf) { return nullptr; }
					timeptr->tm_sec = ret;
					buf = s;
				}
				break;

			case 'Y' :
				{
					const char *s;
					const int ret = strtol_l (buf, &s, 10, locale);
					if (s == buf) { return nullptr; }
					timeptr->tm_year = ret - tm_year_base;
					buf = s;
					ymd |= SET_YEAR;
				}
				break;

			case '\0' :
				--format;
				/* FALLTHROUGH */
				/* no break - DC added for Eclipse */
			case '%' :
				if (*buf == '%')
				{
					++buf;
				}
				else
				{
					return nullptr;
				}
				break;

			default :
				if (*buf == '%' || *++buf == c)
				{
					++buf;
				}
				else
				{
					return nullptr;
				}
				break;
			}
		}
		else
		{
			if (*buf == c)
			{
				++buf;
			}
			else
			{
				return nullptr;
			}
		}
    }

    if ((ymd & SET_YMD) == SET_YMD)
    {
		/* all of tm_year, tm_mon and tm_mday, but... */
		if (!(ymd & SET_YDAY))
		{
			/* ...not tm_yday, so fill it in */
			timeptr->tm_yday = _DAYS_BEFORE_MONTH[timeptr->tm_mon] + timeptr->tm_mday;
			if (!is_leap_year(timeptr->tm_year + tm_year_base) || timeptr->tm_mon < 2)
			{
				timeptr->tm_yday--;
			}
			ymd |= SET_YDAY;
		}
    }
    else if ((ymd & (SET_YEAR | SET_YDAY)) == (SET_YEAR | SET_YDAY))
    {
    	/* both of tm_year and tm_yday, but... */
		if (!(ymd & SET_MON))
		{
			/* ...not tm_mon, so fill it in, and/or... */
			if (timeptr->tm_yday < _DAYS_BEFORE_MONTH[1])
			{
				timeptr->tm_mon = 0;
			}
			else
			{
				const int leap = is_leap_year (timeptr->tm_year + tm_year_base);
				int i;
				for (i = 2; i < 12; ++i)
				{
					if (timeptr->tm_yday < _DAYS_BEFORE_MONTH[i] + leap)
					{
						break;
					}
				}
				timeptr->tm_mon = i - 1;
			}
		}

		if (!(ymd & SET_MDAY))
		{
			/* ...not tm_mday, so fill it in */
			timeptr->tm_mday = timeptr->tm_yday - _DAYS_BEFORE_MONTH[timeptr->tm_mon];
			if (!is_leap_year (timeptr->tm_year + tm_year_base) || timeptr->tm_mon < 2)
			{
				timeptr->tm_mday++;
			}
		}
    }

    if ((ymd & (SET_YEAR | SET_YDAY | SET_WDAY)) == (SET_YEAR | SET_YDAY))
    {
		/* fill in tm_wday */
		const int fday = first_day (timeptr->tm_year + tm_year_base);
		timeptr->tm_wday = (fday + timeptr->tm_yday) % 7;
    }

    return buf;
}

extern "C" char * strptime (const char *_ecv_array buf, const char *_ecv_array format, struct tm *timeptr) noexcept
{
	return const_cast<char *>(SafeStrptime (buf, format, timeptr));
}

// End
