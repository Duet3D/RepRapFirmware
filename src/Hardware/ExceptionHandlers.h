/*
 * CrashHandlers.h
 *
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_EXCEPTIONHANDLERS_H_
#define SRC_HARDWARE_EXCEPTIONHANDLERS_H_

#include <cstdint>

[[noreturn]] void SoftwareReset(uint16_t reason, const uint32_t *stk = nullptr) noexcept;

#endif /* SRC_HARDWARE_EXCEPTIONHANDLERS_H_ */
