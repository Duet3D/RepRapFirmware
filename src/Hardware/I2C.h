/*
 * I2C.h
 *
 *  Created on: 13 May 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_I2C_H_
#define SRC_HARDWARE_I2C_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>

#ifdef I2C_IFACE
# include "Wire.h"
#endif

namespace I2C
{
	void Init() noexcept;

#ifdef I2C_IFACE

	uint32_t statusWaitFunc(Twi *twi, uint32_t bitsToWaitFor) noexcept;

	// Transfer data to/from an I2C peripheral.
	// If the caller needs to do multiple I2C transactions without being interrupted, it should own the i2C mutex before calling this.
	// Otherwise the caller need not own the mutex because it will be acquired here.
	inline size_t Transfer(uint16_t address, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept
	{
		MutexLocker Lock(Tasks::GetI2CMutex());
		return I2C_IFACE.Transfer(address, buffer, numToWrite, numToRead, statusWaitFunc);
	}

#endif

}

#endif /* SRC_HARDWARE_I2C_H_ */
