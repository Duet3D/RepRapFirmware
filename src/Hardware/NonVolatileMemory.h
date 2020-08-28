/*
 * NonVolatileMemory.h
 *
 * Class to manage an area of flash memory that we use to store data that persists over a reset
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_NONVOLATILEMEMORY_H_
#define SRC_HARDWARE_NONVOLATILEMEMORY_H_

#include <Hardware/SoftwareReset.h>

// This class manages nonvolatile settings that are specific to the board, and the software reset data that is stored by the crash handler.
// On most Duets there is a 512-byte User Page that we use for this.
// The SAMC21 and SAME5x processors already store various data in the user page, however both those processor families support EEPROM emulation so we use 512 bytes of that instead.

class NonVolatileMemory
{
public:
	NonVolatileMemory() noexcept;

	void EnsureWritten() noexcept;
	SoftwareResetData *GetLastWrittenResetData(unsigned int &slot) noexcept;
	SoftwareResetData *AllocateResetDataSlot() noexcept;
	int8_t GetThermistorLowCalibration(unsigned int inputNumber) noexcept;
	int8_t GetThermistorHighCalibration(unsigned int inputNumber) noexcept;
	void SetThermistorLowCalibration(unsigned int inputNumber, int8_t val) noexcept;
	void SetThermistorHighCalibration(unsigned int inputNumber, int8_t val) noexcept;

	static constexpr unsigned int NumberOfResetDataSlots = 3;
	static constexpr unsigned int MaxCalibratedThermistors = 8;

private:
	void EnsureRead() noexcept;
	int8_t GetThermistorCalibration(unsigned int inputNumber, uint8_t *calibArray) noexcept;
	void SetThermistorCalibration(unsigned int inputNumber, int8_t val, uint8_t *calibArray) noexcept;

	struct NVM
	{
		uint16_t magic;
		uint8_t thermistorLowCalibration[MaxCalibratedThermistors];		// currently used only by SAME70-based boards
		uint8_t thermistorHighCalibration[MaxCalibratedThermistors];	// currently used only by SAME70-based boards
		uint8_t spare[38];
		// 56 bytes up to here
		SoftwareResetData resetData[NumberOfResetDataSlots];			// 3 slots of 152 bytes each

		static constexpr uint32_t MagicValue = 0x41E5;
	};

	static_assert(sizeof(NVM) == 512);

	enum class NvmState : uint8_t { notRead, clean, writeNeeded, eraseAndWriteNeeded };

	alignas(4) NVM buffer;
	NvmState state;
};

#endif /* SRC_HARDWARE_NONVOLATILEMEMORY_H_ */
