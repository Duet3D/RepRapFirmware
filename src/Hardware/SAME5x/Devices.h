/*
 * Devices.h
 *
 *  Created on: 9 Jul 2020
 *      Author: David
 *  License: GNU GPL v3
 */

#ifndef SRC_HARDWARE_SAME5X_DEVICES_H_
#define SRC_HARDWARE_SAME5X_DEVICES_H_

#include "Uart.h"
typedef Uart UARTClass;

extern Uart serialUart0, serialUart1;

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include "SerialCDC.h"

extern SerialCDC serialUSB;

void DeviceInit() noexcept;
void StopAnalogTask() noexcept;

// GCLK numbers not defined in the core
constexpr unsigned int GclkNum25MHz = 2;		// for Ethernet PHY
constexpr unsigned int GclkNum90MHz = 5;		// for SDHC

#endif /* SRC_HARDWARE_SAME5X_DEVICES_H_ */
