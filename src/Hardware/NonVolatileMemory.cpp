/*
 * NonVolatileMemory.cpp
 *
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#include "NonVolatileMemory.h"

#if SAM4E || SAM4S || SAME70
# include <Cache.h>
# include <flash_efc.h>
#endif

NonVolatileMemory::NonVolatileMemory() noexcept : state(NvmState::notRead)
{
}

void NonVolatileMemory::EnsureRead() noexcept
{
	if (state == NvmState::notRead)
	{
#if SAME5x
		memcpy(&buffer, reinterpret_cast<const void *>(SEEPROM_ADDR), sizeof(buffer));
#elif defined(__LPC17xx__)
# error		//TODO
#elif SAM4E || SAM4S || SAME70
		const bool cacheEnabled = Cache::Disable();
		flash_read_user_signature(reinterpret_cast<uint32_t*>(&buffer), sizeof(buffer)/sizeof(uint32_t));
		if (cacheEnabled)
		{
			Cache::Enable();
		}
#else
# error Unsupported processor
#endif
		if (buffer.magic != NVM::MagicValue)
		{
//			debugPrintf("Invalid user area\n");
			memset(&buffer, 0xFF, sizeof(buffer));
			buffer.magic = NVM::MagicValue;
			state = NvmState::eraseAndWriteNeeded;
		}
		else
		{
			state = NvmState::clean;
//			debugPrintf("user area valid\n");
		}
	}
}

void NonVolatileMemory::EnsureWritten() noexcept
{
#if SAME5x
	if (state >= NvmState::writeNeeded)
	{
		// No need to erase on the SAME5x because the EEPROM emulation manages it
        while (NVMCTRL->SEESTAT.bit.BUSY) { }
        memcpy(reinterpret_cast<uint8_t*>(SEEPROM_ADDR), &buffer, sizeof(buffer));
		state = NvmState::clean;
        while (NVMCTRL->SEESTAT.bit.BUSY) { }
	}
#else
	if (state == NvmState::eraseAndWriteNeeded)
	{
		// Erase the page
# if SAM4E || SAM4S || SAME70
		flash_erase_user_signature();
# elif defined(__LPC17xx__)
		LPC_EraseSoftwareResetDataSlots();	// erase the last flash sector
# endif
		state = NvmState::writeNeeded;
	}

	if (state == NvmState::writeNeeded)
	{
# if SAM4E || SAM4S || SAME70
		flash_write_user_signature(&buffer, sizeof(buffer)/sizeof(uint32_t));
# elif defined(__LPC17xx__)
		LPC_WriteSoftwareResetData(slot, buffer, sizeof(buffer));
# else
#  error Unsupported processor
# endif
		state = NvmState::clean;
	}
#endif
}

SoftwareResetData* NonVolatileMemory::GetLastWrittenResetData(unsigned int &slot) noexcept
{
	EnsureRead();
	for (unsigned int i = NumberOfResetDataSlots; i != 0; )
	{
		--i;
		if (buffer.resetData[i].IsValid())
		{
			slot = i;
			return &buffer.resetData[i];
		}
	}
	return nullptr;
}

SoftwareResetData* NonVolatileMemory::AllocateResetDataSlot() noexcept
{
	EnsureRead();
	for (unsigned int i = 0; i < NumberOfResetDataSlots; ++i)
	{
		if (buffer.resetData[i].IsVacant())
		{
			if (state == NvmState::clean)			// need this test because state may already be EraseAndWriteNeeded after EnsureRead
			{
				state = NvmState::writeNeeded;		// assume the caller will write to the allocated slot
			}
			return &buffer.resetData[i];
		}
	}

	// All slots are full, so clear them out and start again
	for (unsigned int i = 0; i < NumberOfResetDataSlots; ++i)
	{
		buffer.resetData[i].Clear();
	}
	state = NvmState::eraseAndWriteNeeded;
	return &buffer.resetData[0];
}

int8_t NonVolatileMemory::GetThermistorLowCalibration(unsigned int inputNumber) noexcept
{
	return GetThermistorCalibration(inputNumber, buffer.thermistorLowCalibration);
}

int8_t NonVolatileMemory::GetThermistorHighCalibration(unsigned int inputNumber) noexcept
{
	return GetThermistorCalibration(inputNumber, buffer.thermistorHighCalibration);
}

void NonVolatileMemory::SetThermistorLowCalibration(unsigned int inputNumber, int8_t val) noexcept
{
	SetThermistorCalibration(inputNumber, val, buffer.thermistorLowCalibration);
}

void NonVolatileMemory::SetThermistorHighCalibration(unsigned int inputNumber, int8_t val) noexcept
{
	SetThermistorCalibration(inputNumber, val, buffer.thermistorHighCalibration);
}

int8_t NonVolatileMemory::GetThermistorCalibration(unsigned int inputNumber, uint8_t *calibArray) noexcept
{
	EnsureRead();
	return (inputNumber >= MaxCalibratedThermistors || calibArray[inputNumber] == 0xFF) ? 0 : calibArray[inputNumber] - 0x7F;
}

void NonVolatileMemory::SetThermistorCalibration(unsigned int inputNumber, int8_t val, uint8_t *calibArray) noexcept
{
	if (inputNumber < MaxCalibratedThermistors)
	{
		EnsureRead();
		const uint8_t oldVal = calibArray[inputNumber];
		const uint8_t newVal = val + 0x7F;
		if (oldVal != newVal)
		{
			// If we are only changing 1 bits to 0 then we don't need to erase
			calibArray[inputNumber] = newVal;
			if ((newVal & ~oldVal) != 0)
			{
				state = NvmState::eraseAndWriteNeeded;
			}
			else if (state == NvmState::clean)
			{
				state = NvmState::writeNeeded;
			}
		}
	}
}

// End
