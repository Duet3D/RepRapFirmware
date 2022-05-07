/*
 * ILI9488.cpp
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#include "ILI9488.h"

#if SUPPORT_ILI9488_LCD

LcdILI9488::LcdILI9488(const LcdFont * const fnts[], size_t nFonts, uint8_t sercomNum) noexcept
	: TFTLcd(320, 480, fnts, nFonts, SpiMode::mode0, sercomNum)
{
}

// Get the display type
const char *_ecv_array LcdILI9488::GetDisplayTypeName() const noexcept
{
	return "480x320 TFT with ILI9488 controller";
}

// Initialise the TFT screen
void LcdILI9488::HardwareInit() noexcept
{
	SendCommand(CmdReset);
	delay(ResetDelayMillis + 1);

	//TODO
}

// Flush just some data, returning true if this needs to be called again
bool LcdILI9488::FlushSome() noexcept
{
	// See if there is anything to flush
	if (dirtyColumnsEnd > dirtyColumnsStart && dirtyRowsEnd > dirtyRowsStart)
	{
		// Flush the first dirty row
		//TODO
		++dirtyRowsStart;
		return dirtyRowsStart < dirtyRowsEnd;
	}
	return false;
}

void LcdILI9488::SendCommand(uint8_t cmd) noexcept
{
	spiBuffer[0] = cmd | 0x0100;
	SendBuffer(1);
}

void LcdILI9488::SendCommand(uint8_t cmd, size_t numData, uint8_t data[]) noexcept
{
	spiBuffer[0] = cmd | 0x0100;
	for (size_t i = 0; i < numData; ++i)
	{
		spiBuffer[i + 1] = (uint16_t)data[i];
	}
	SendBuffer(numData + 1);
}

void LcdILI9488::SetGraphicsAddress(PixelNumber r, PixelNumber cBegin, PixelNumber cEnd) noexcept
{
	spiBuffer[0] = CmdPageAddressSet | 0x0100;
	spiBuffer[1] = spiBuffer[3] = r >> 8;
	spiBuffer[2] = spiBuffer[4] = r & 0x00FF;
	spiBuffer[5] = CmdColumnAddressSet;
	spiBuffer[6] = cBegin >> 8;
	spiBuffer[7] = cBegin & 0x00FF;
	spiBuffer[8] = cEnd >> 8;
	spiBuffer[9] = cEnd & 0x00FF;
	SendBuffer(10);
}

void LcdILI9488::SendBuffer(size_t numWords) const noexcept
{
	digitalWrite(csPin, csPol);
	delayMicroseconds(1);
	spiDev.TransceivePacketNineBit(spiBuffer, nullptr, numWords);
	delayMicroseconds(1);
	digitalWrite(csPin, !csPol);
}

#endif

// End
