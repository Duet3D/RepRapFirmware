/*
 * I2C.h
 *
 *  Created on: 13 May 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_I2C_H_
#define SRC_HARDWARE_I2C_H_

#include "Wire.h"
#include "RepRapFirmware.h"
#include "Tasks.h"

namespace I2C
{
	void Init();

#ifdef I2C_IFACE

#ifdef RTOS

	uint32_t statusWaitFunc(Twi *twi, uint32_t bitsToWaitFor);

	// Transfer data to/from an I2C peripheral.
	// If the caller needs to do multiple I2C transactions without being interrupted, it should own the i2C mutex before calling this.
	// Otherwise the caller need nort own the mutex because it will be acquired here.
	inline size_t Transfer(uint16_t address, uint8_t *buffer, size_t numToWrite, size_t numToRead)
	{
		MutexLocker Lock(Tasks::GetI2CMutex());
		return I2C_IFACE.Transfer(address, buffer, numToWrite, numToRead, statusWaitFunc);
	}

#else

	// Transfer data to/from an I2C peripheral
	inline size_t Transfer(uint16_t address, uint8_t *buffer, size_t numToWrite, size_t numToRead)
	{
		return I2C_IFACE.Transfer(address, buffer, numToWrite, numToRead);
	}

#endif

#endif

}

#endif /* SRC_HARDWARE_I2C_H_ */
