/*
 * Devices.h
 *
 *  Created on: 9 Jul 2020
 *      Author: David
 *  License: GNU GPL v3
 */

#ifndef SRC_HARDWARE_SAME5X_DEVICES_H_
#define SRC_HARDWARE_SAME5X_DEVICES_H_

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include <SerialCDC.h>

extern SerialCDC serialUSB;
#if CORE_USES_TINYUSB
extern SerialCDC serialUSB2;
#endif

void DeviceInit() noexcept;
void StopAnalogTask() noexcept;
void StopUsbTask() noexcept;

#endif /* SRC_HARDWARE_SAME5X_DEVICES_H_ */
