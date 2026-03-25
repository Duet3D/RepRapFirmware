/*
 * Devices.h
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME70_DEVICES_H_
#define SRC_HARDWARE_SAME70_DEVICES_H_

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include "SerialCDC.h"

extern SerialCDC serialUSB;
#ifdef SERIAL_USB2_DEVICE
extern SerialCDC serialUSB2;
#endif

void DeviceInit() noexcept;
void StopAnalogTask() noexcept;
void StopUsbTask() noexcept;

#endif /* SRC_HARDWARE_SAME70_DEVICES_H_ */
