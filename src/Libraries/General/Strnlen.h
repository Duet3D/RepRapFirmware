/*
 * Strnlen.h
 *
 *  Created on: 17 Apr 2018
 *      Author: David
 */

#ifndef SRC_LIBRARIES_GENERAL_STRNLEN_H_
#define SRC_LIBRARIES_GENERAL_STRNLEN_H_

#include <cstddef>

// 'strnlen' isn't ISO standard, so we define our own
size_t Strnlen(const char *s, size_t n);

#endif /* SRC_LIBRARIES_GENERAL_STRNLEN_H_ */
