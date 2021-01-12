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

extern AsyncSerial serialUart0, serialUart1;

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include "SerialCDC.h"

extern SerialCDC serialUSB;

void DeviceInit() noexcept;

#endif /* SRC_HARDWARE_SAME70_DEVICES_H_ */
