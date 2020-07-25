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

extern Uart serialUart0;

#include "SerialCDC.h"

extern SerialCDC serialUSB;

void DeviceInit();

// GCLK numbers not defined in the core
static const unsigned int GclkNum25MHz = 2;		// for Ethernet PHY
static const unsigned int GclkNum90MHz = 5;		// for SDHC

#endif /* SRC_HARDWARE_SAME5X_DEVICES_H_ */
