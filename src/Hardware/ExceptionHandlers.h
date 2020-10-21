/*
 * CrashHandlers.h
 *
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_EXCEPTIONHANDLERS_H_
#define SRC_HARDWARE_EXCEPTIONHANDLERS_H_

#include "SoftwareReset.h"

[[noreturn]] void SoftwareReset(SoftwareResetReason initialReason, const uint32_t *stk = nullptr) noexcept;
[[noreturn]] void OutOfMemoryHandler() noexcept;

#endif /* SRC_HARDWARE_EXCEPTIONHANDLERS_H_ */
