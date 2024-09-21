/*
 * LIS3DH.h
 *
 *  Created on: 14 Mar 2021
 *      Author: David
 */

#ifndef SRC_HARDWARE_LIS3DH_H_
#define SRC_HARDWARE_LIS3DH_H_

#include <RepRapFirmware.h>

#if SUPPORT_ACCELEROMETERS

#include <Hardware/Spi/SharedSpiClient.h>

class LISAccelerometer : public SharedSpiClient
{
public:
	LISAccelerometer(SharedSpiDevice& dev, uint32_t freq, Pin p_csPin, Pin p_int1Pin) noexcept;

	// Do a quick test to check whether the accelerometer is present, returning true if it is
	bool CheckPresent() noexcept;

	// Return the type name of the accelerometer. Only valid after checkPresent returns true.
	const char *GetTypeName() const noexcept;

	// Configure the accelerometer to collect at or near the requested sampling rate and the requested resolution in bits.
	bool Configure(uint16_t& p_samplingRate, uint8_t& p_resolution) noexcept;

	// Start collecting data
	bool StartCollecting(uint8_t axes) noexcept;

	// Collect some data from the FIFO, suspending until the data is available
	unsigned int CollectData(const uint16_t *_ecv_array *collectedData, uint16_t &dataRate, bool &overflowed) noexcept;

	// Stop collecting data
	void StopCollecting() noexcept;

	// Get a status byte
	uint8_t ReadStatus() noexcept;

	// Used by the ISR
	void Int1Isr() noexcept;

	// Used by diagnostics
	bool HasInterruptError() const noexcept { return interruptError; }

private:
	enum class LisRegister : uint8_t
	{
		WhoAmI = 0x0f,
		Ctrl_0x20 = 0x20,			// this is CTRL_REG1 on the LIS3DH and LIS2DW, and CTRL_REG4 on the LIS3DSH
		CtrlReg2_2DW = 0x21,		// this is CTRL_REG2 on the LIS2DW
		CtrlReg6_3DSH = 0x25,
		Status = 0x27,				// this is STATUS on LIS3DH and LIS2DW, and STAT on LIS3DSH
		OutXL = 0x28,
		FifoControl = 0x2E,
		FifoSource = 0x2F			// this is FIFO_SRC on the LID3DH, FIFO_SAMPLES on LID2DW, and STATUS on LID3DSH
	};

	bool ReadRegisters(LisRegister reg, size_t numToRead) noexcept;
	bool WriteRegisters(LisRegister reg, size_t numToWrite) noexcept;
	bool ReadRegister(LisRegister reg, uint8_t& val) noexcept;
	bool WriteRegister(LisRegister reg, uint8_t val) noexcept;

	volatile TaskHandle taskWaiting;
	uint32_t firstInterruptTime;
	uint32_t lastInterruptTime;
	uint32_t totalNumRead;
	AccelerometerType accelerometerType;
	bool interruptError;
	uint8_t currentAxis;
	uint8_t ctrlReg_0x20;
	Pin int1Pin;
	alignas(2) uint8_t transferBuffer[2 + (6 * 32)];			// 1 dummy byte for alignment, one register address byte, 192 data bytes to read entire FIFO
	uint8_t *_ecv_array const dataBuffer = transferBuffer + 2;
};

#endif

#endif /* SRC_HARDWARE_LIS3DH_H_ */
