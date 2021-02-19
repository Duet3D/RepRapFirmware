/*
 * Devices.h
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAM4S_DEVICES_H_
#define SRC_HARDWARE_SAM4S_DEVICES_H_

#ifndef PCCB
# include <AsyncSerial.h>
typedef AsyncSerial UARTClass;
# include <USARTClass.h>

extern AsyncSerial Serial;
#endif

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include <SerialCDC.h>

extern SerialCDC SerialUSB;

#include <Wire.h>
extern TwoWire Wire;

void DeviceInit() noexcept;
void StopAnalogTask() noexcept;

#endif /* SRC_HARDWARE_SAM4S_DEVICES_H_ */
