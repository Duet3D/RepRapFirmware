/*
 * LIS3DH.cpp
 *
 *  Created on: 14 Mar 2021
 *      Author: David
 */

#include "LIS3DH.h"

#if SUPPORT_ACCELEROMETERS

#include <Hardware/IoPorts.h>
#include <Movement/StepTimer.h>

constexpr uint32_t Lis3dSpiTimeout = 25;							// timeout while waiting for the SPI bus
constexpr uint32_t DataCollectionTimeout = (1000 * 32)/400 + 2;		// timeout whole collecting data, enough to fill the FIFO at 400Hz
constexpr uint8_t FifoInterruptLevel = 20;							// how full the FIFO must get before we want an interrupt
const SpiMode lisMode = SpiMode::mode3;

static constexpr uint8_t WhoAmIValue = 0x33;

LIS3DH::LIS3DH(SharedSpiDevice& dev, uint32_t freq, Pin p_csPin, Pin p_int1Pin) noexcept
	: SharedSpiClient(dev, freq, lisMode, p_csPin, false), taskWaiting(nullptr), int1Pin(p_int1Pin)
{
}

// Do a quick test to check whether the accelerometer is present, returning true if it is
bool LIS3DH::CheckPresent() noexcept
{
	uint8_t val;
	return ReadRegister(LisRegister::WhoAmI, val) && val == WhoAmIValue;
}

uint8_t LIS3DH::ReadStatus() noexcept
{
	uint8_t val;
	return (ReadRegister(LisRegister::Status, val)) ? val : 0xFF;
}

// Configure the accelerometer to collect for the requested axis at or near the requested sampling rate and the requested resolution in bits.
// Actual collection does not start until the first call to Collect8bitData or Collect16bitData.
bool LIS3DH::Configure(uint16_t& samplingRate, uint8_t& resolution) noexcept
{
	// Set up control registers 1 and 4 according to the selected sampling rate and resolution
	ctrlReg1 = 0;													// collect no axes for now
	uint8_t ctrlReg4 = (1 << 7);									// set block data update
	if (resolution >= 12)											// if high resolution mode
	{
		resolution = 12;
		ctrlReg4 |= (1 << 3);										// set HR
	}
	else if (resolution < 10)										// if low res mode
	{
		resolution = 8;
		ctrlReg1 |= (1 << 3);										// set LP
	}
	else
	{
		resolution = 10;
	}

	uint8_t odr;
	if (samplingRate >= 1000)
	{
		if (resolution >= 10)
		{
			odr = 0x9;												// in 10- or 12-bit mode, so select 1.344kHz (next lowest is 400Hz)
			samplingRate = 1344;
		}
		else if (samplingRate >= 5000)
		{
			odr = 0x9;												// select 5.376kHz
			samplingRate = 5376;
		}
		else
		{
			odr = 0x8;												// select 1.6kHz
			samplingRate = 1600;
		}
	}
	else
	{
		odr = 0x7;
		samplingRate = 400;											// we don't support lower than 400Hz because it isn't useful
	}

	ctrlReg1 |= (odr << 4);

	// Set up the control registers, except set ctrlReg1 to 0 to select power down mode
	dataBuffer[0] = 0;							// ctrlReg1: for now select power down mode
	dataBuffer[1] = 0;							// ctrlReg2: high pass filter not used
	dataBuffer[2] = (1u << 2);					// ctrlReg3: enable fifo watermark interrupt
	dataBuffer[3] = ctrlReg4;
	dataBuffer[4] = (1u << 6);					// ctrlReg5: enable fifo
	dataBuffer[5] = 0;							// ctrlReg6: INT2 disabled, active high interrupts
	bool ok = WriteRegisters(LisRegister::Ctrl1, 6);
	if (ok)
	{
		// Set the fifo mode
		ok = WriteRegister(LisRegister::FifoControl, (2u << 6) | (FifoInterruptLevel - 1));
	}
	return ok;
}

void Int1Interrupt(CallbackParameter p) noexcept;		// forward declaration

// Start collecting data
bool LIS3DH:: StartCollecting(uint8_t axes) noexcept
{
	ctrlReg1 |= (axes & 7);

	// Clear the fifo
	uint8_t val;
	while (ReadRegister(LisRegister::FifoSource, val) && (val & (1u << 5)) == 0)		// while fifo not empty
	{
		ReadRegisters(LisRegister::OutXL, 6);
	}

	totalNumRead = 0;
	const bool ok = WriteRegister(LisRegister::Ctrl1, ctrlReg1);
	return ok && attachInterrupt(int1Pin, Int1Interrupt, InterruptMode::rising, this);
}

// Collect some data from the FIFO, suspending until the data is available
unsigned int LIS3DH::CollectData(const uint16_t **collectedData, uint16_t &dataRate, bool &overflowed) noexcept
{
	// Wait until we have some data
	taskWaiting = TaskBase::GetCallerTaskHandle();
	while (!digitalRead(int1Pin))
	{
		if (!TaskBase::Take(DataCollectionTimeout))
		{
			return 0;
		}
	}
	taskWaiting = nullptr;

	// Get the fifo status to see how much data we can read and whether the fifo overflowed
	uint8_t fifoStatus;
	if (!ReadRegister(LisRegister::FifoSource, fifoStatus))
	{
		return 0;
	}

	uint8_t numToRead = fifoStatus & 0x1F;
	if (numToRead == 0 && (fifoStatus & 0x20) == 0)
	{
		numToRead = 32;
	}

	if (numToRead != 0)
	{
		// Read the data
		// When the auto-increment bit is set in the register number, after reading register 0x2D it wraps back to 0x28
		// The datasheet doesn't mention this but ST app note AN3308 does
		if (!ReadRegisters(LisRegister::OutXL, 6 * numToRead))
		{
			return 0;
		}

		*collectedData = reinterpret_cast<const uint16_t*>(dataBuffer);
		overflowed = (fifoStatus & 0x40) != 0;
		const uint32_t interval = lastInterruptTime - firstInterruptTime;
		dataRate = (totalNumRead == 0 || interval == 0)
					? 0
					: (totalNumRead * StepTimer::StepClockRate)/interval;
		totalNumRead += numToRead;
	}
	return numToRead;
}

// Stop collecting data
void LIS3DH::StopCollecting() noexcept
{
	WriteRegister(LisRegister::Ctrl1, 0);
}

// Read registers into dataBuffer
bool LIS3DH::ReadRegisters(LisRegister reg, size_t numToRead) noexcept
{
	if (!Select(Lis3dSpiTimeout))
	{
		return false;
	}
	transferBuffer[1] = (uint8_t)reg | 0xC0;		// set auto increment and read bits
	const bool ret = TransceivePacket(transferBuffer + 1, transferBuffer + 1, 1 + numToRead);
	Deselect();
	return ret;
}

// Write registers from dataBuffer
bool LIS3DH::WriteRegisters(LisRegister reg, size_t numToWrite) noexcept
{
	if ((uint8_t)reg < 0x1E)						// don't overwrite the factory calibration values
	{
		return false;
	}
	if (!Select(Lis3dSpiTimeout))
	{
		return false;
	}
	transferBuffer[1] = (uint8_t)reg | 0x40;		// set auto increment bit
	const bool ret = TransceivePacket(transferBuffer + 1, transferBuffer + 1, 1 + numToWrite);
	Deselect();
	return ret;
}

bool LIS3DH::ReadRegister(LisRegister reg, uint8_t& val) noexcept
{
	const bool ret = ReadRegisters(reg, 1);
	if (ret)
	{
		val = dataBuffer[0];
	}
	return ret;
}

bool LIS3DH::WriteRegister(LisRegister reg, uint8_t val) noexcept
{
	dataBuffer[0] = val;
	return WriteRegisters(reg, 1);
}

void LIS3DH::Int1Isr() noexcept
{
	const uint32_t now = StepTimer::GetTimerTicks();
	if (totalNumRead == 0)
	{
		firstInterruptTime = now;
	}
	lastInterruptTime = now;
	TaskBase::GiveFromISR(taskWaiting);
	taskWaiting = nullptr;
}

void Int1Interrupt(CallbackParameter p) noexcept
{
	static_cast<LIS3DH*>(p.vp)->Int1Isr();
}

#endif

// End
