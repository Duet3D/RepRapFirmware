/*
 * SafeVsnprintf.h
 *
 *  Created on: 8 Apr 2018
 *      Author: David
 */

#ifndef SRC_LIBRARIES_GENERAL_SAFEVSNPRINTF_H_
#define SRC_LIBRARIES_GENERAL_SAFEVSNPRINTF_H_

#include <cstdarg>
#include <cstddef>

int SafeVsnprintf(char *buffer, size_t maxLen, const char *format, va_list args);
int SafeSnprintf(char* buffer, size_t maxLen, const char* format, ...) __attribute__ ((format (printf, 3, 4)));

#define vsnprintf(b, m, f, a) static_assert(false, "Do not use vsnprintf, use SafeVsnprintf instead")
#define snprintf(b, m, f, ...) static_assert(false, "Do not use snprintf, use SafeSnprintf instead")

#endif /* SRC_LIBRARIES_GENERAL_SAFEVSNPRINTF_H_ */
