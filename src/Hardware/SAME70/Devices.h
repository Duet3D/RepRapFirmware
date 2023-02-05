/*
 * Devices.h
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME70_DEVICES_H_
#define SRC_HARDWARE_SAME70_DEVICES_H_

#include <AsyncSerial.h>
#include <USARTClass.h>

extern AsyncSerial serialUart1;
extern USARTClass serialUart2;

#if defined(DUET3_MB6HC)
extern AsyncSerial serialWiFi;
#endif

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include "SerialCDC.h"

extern SerialCDC serialUSB;

void DeviceInit() noexcept;
void StopAnalogTask() noexcept;
void StopUsbTask() noexcept;

#endif /* SRC_HARDWARE_SAME70_DEVICES_H_ */
