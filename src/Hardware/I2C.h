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
# if SAM4S || SAM4E			// these boards use the legacy TWI-based Wire driver; all other boards use SharedI2CMaster
#  include <Devices.h>
#  include "Wire.h"
# else
#  include <I2C/SharedI2CMaster.h>
# endif
#endif

namespace I2C
{
	// Set up the I2C bus if not already done. Returns false and sets reply if the bus pins are not available
	bool Init(const StringRef& reply) noexcept;

	// Version for callers that cannot report or handle failure (the DueX expansion on Duet 2, where the bus pins are dedicated)
	void Init() noexcept;

#ifdef I2C_IFACE

# if SAM4S || SAM4E

	uint32_t statusWaitFunc(Twi *twi, uint32_t bitsToWaitFor) noexcept;

	// Transfer data to/from an I2C peripheral.
	// If the caller needs to do multiple I2C transactions without being interrupted, it should own the i2C mutex before calling this.
	// Otherwise the caller need not own the mutex because it will be acquired here.
	inline size_t Transfer(uint16_t address, uint8_t *_ecv_array buffer, size_t numToWrite, size_t numToRead) noexcept
	{
		MutexLocker Lock(Tasks::GetI2CMutex());
		return I2C_IFACE.Transfer(address, buffer, numToWrite, numToRead, statusWaitFunc);
	}

# else

	extern SharedI2CMaster *_ecv_null sharedI2C;

	constexpr uint32_t I2CBusTimeoutMillis = 25;			// how long we wait to acquire the I2C bus

	// Transfer data to/from an I2C peripheral. Returns the number of bytes transferred (write + read), or fewer if the transfer failed.
	// The first numToWrite bytes of the buffer are written; any bytes read are appended after them
	inline size_t Transfer(uint16_t address, uint8_t *_ecv_array buffer, size_t numToWrite, size_t numToRead) noexcept
	{
		if (sharedI2C == nullptr || !sharedI2C->Take(I2CBusTimeoutMillis))
		{
			return 0;
		}
		const bool ok = sharedI2C->Transfer(address, buffer, buffer + numToWrite, numToWrite, numToRead);
		sharedI2C->Release();
		return (ok) ? numToWrite + numToRead : 0;
	}

# endif

#endif

}

#endif /* SRC_HARDWARE_I2C_H_ */
