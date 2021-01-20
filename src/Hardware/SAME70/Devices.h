/*
 * Devices.h
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME70_DEVICES_H_
#define SRC_HARDWARE_SAME70_DEVICES_H_

#include <AsyncSerial.h>
typedef AsyncSerial UARTClass;
#include <USARTClass.h>

extern AsyncSerial Serial;
extern USARTClass Serial1;

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include "SerialCDC.h"

extern SerialCDC SerialUSB;

void DeviceInit() noexcept;
void StopAnalogTask() noexcept;

#endif /* SRC_HARDWARE_SAME70_DEVICES_H_ */
