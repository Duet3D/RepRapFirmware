/*
 * SafeStrtod.h
 *
 *  Created on: 8 Apr 2018
 *      Author: David
 */

#ifndef SRC_LIBRARIES_GENERAL_SAFESTRTOD_H_
#define SRC_LIBRARIES_GENERAL_SAFESTRTOD_H_

double SafeStrtod(const char *s, const char **p = nullptr);
float SafeStrtof(const char *s, const char **p = nullptr);

inline long SafeStrtol(const char *s, const char **endptr = nullptr, int base = 10)
{
	return strtol(s, const_cast<char**>(endptr), base);
}

inline unsigned long SafeStrtoul(const char *s, const char **endptr = nullptr, int base = 10)
{
	return strtoul(s, const_cast<char**>(endptr), base);
}

inline unsigned long SafeStrtoul(char *s, char **endptr = nullptr, int base = 10)
{
	return strtoul(s, endptr, base);
}

#define strtod(s, p) Do_not_use_strtod_use_SafeStrtod_instead
#define strtof(s, p) Do_not_use_strtof_use_SafeStrtof_instead
#define strtol(s, ...) Do_not_use_strtol_use_SafeStrtol_instead
#define strtoul(s, ...) Do_not_use_strtoul_use_SafeStrtoul_instead
#define atof(s) Do_not_use_atof_use_SafeStrtof_instead

#endif /* SRC_LIBRARIES_GENERAL_SAFESTRTOD_H_ */
