/*
 * Devices.h
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAM4E_DEVICES_H_
#define SRC_HARDWARE_SAM4E_DEVICES_H_

#include <AsyncSerial.h>

extern AsyncSerial serialUart;
extern AsyncSerial serialWiFi;

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include <SerialCDC.h>

extern SerialCDC serialUSB;

#include <Wire.h>
extern TwoWire Wire;

void DeviceInit() noexcept;
void StopAnalogTask() noexcept;
void StopUsbTask() noexcept;

#endif /* SRC_HARDWARE_SAM4E_DEVICES_H_ */
