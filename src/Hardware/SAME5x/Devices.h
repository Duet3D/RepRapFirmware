/*
 * Devices.h
 *
 *  Created on: 9 Jul 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_ATMELSTART_SAME5X_DEVICES_H_
#define SRC_HARDWARE_SAME5X_ATMELSTART_SAME5X_DEVICES_H_

#include "Uart.h"
typedef Uart UARTClass;

extern Uart serialUart0;

#include "SerialCDC.h"

extern SerialCDC serialUSB;

void DeviceInit();

#endif /* SRC_HARDWARE_SAME5X_ATMELSTART_SAME5X_DEVICES_H_ */
