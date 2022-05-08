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
	currentRowColMode = 2;						// force row or column mode to be selected
	ClearBlock(0, 0, numRows, numCols, false);
	SendCommand(CmdDisplayOn);
}

// Clear part of the display
void LcdILI9488::ClearBlock(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right, bool foreground) noexcept
{
	if (left < right)
	{
		while (top < bottom)
		{
			const PixelNumber rowsToDo = ((unsigned int)(bottom - top) * (unsigned int)(right - left) < MaxPixelsPerTransaction)
											? (unsigned int)(bottom - top)
												: MaxPixelsPerTransaction/(unsigned int)(right - left);
			uint16_t *_ecv_array p = SetRowMode(spiBuffer, true);
			p = SetGraphicsAddress(p, top, top + rowsToDo - 1, left, right - 1);
			*p++ = CmdMemoryWrite;
			p = SetPixelData(p, (foreground) ? fgColour : bgColour, rowsToDo * (right - left));
			SendBuffer(p - spiBuffer);
			top += rowsToDo;
		}
	}
}

// Set, clear or invert a pixel
//  x = x-coordinate of the pixel, measured from left hand edge of the display
//  y = y-coordinate of the pixel, measured down from the top of the display
//  mode = whether we want to set or clear the pixel
void LcdILI9488::SetPixel(PixelNumber y, PixelNumber x, bool mode) noexcept
{
	// When writing a single pixel we don't care whether we are in row or column mode
	uint16_t *_ecv_array p = SetGraphicsAddress(spiBuffer, y, y, x, x);
	*p++ = CmdMemoryWrite;
	p = SetPixelData(p, (mode) ? fgColour : bgColour, 1);
	SendBuffer(p - spiBuffer);
}

// Draw a bitmap
//  x0 = x-coordinate of the top left, measured from left hand edge of the display. Currently, must be a multiple of 8.
//  y0 = y-coordinate of the top left, measured down from the top of the display
//  width = width of bitmap in pixels. Currently, must be a multiple of 8.
//  rows = height of bitmap in pixels
//  data = bitmap image, must be ((width/8) * rows) bytes long
void LcdILI9488::BitmapImage(PixelNumber top, PixelNumber left, PixelNumber height, PixelNumber width, const uint8_t data[]) noexcept
{
	//TODO
}

// Draw a bitmap row
//  x0 = x-coordinate of the top left, measured from left hand edge of the display
//  y0 = y-coordinate of the top left, measured down from the top of the display
//  width = width of bitmap in pixels
//  data = bitmap image, must be ((width + 7)/8) bytes long
void LcdILI9488::BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept
{
	//TODO
}

// Write one column of character data at (row, column)
void LcdILI9488::WriteColumnData(uint16_t columnData, uint8_t ySize) noexcept
{
	uint16_t *_ecv_array p = SetRowMode(spiBuffer, false);
	p = SetGraphicsAddress(p, row, row + ySize - 1, column, column);
	*p++ = CmdMemoryWrite;
	for (uint8_t i = 0; i < ySize; ++i)
	{
		p = SetPixelData(p, (columnData & 1u) ? fgColour : bgColour, 1);
		columnData >>= 1;
	}
	SendBuffer(p - spiBuffer);
}

// Send a parameterless command
void LcdILI9488::SendCommand(uint8_t cmd) noexcept
{
	spiBuffer[0] = cmd;
	SendBuffer(1);
}

uint16_t *_ecv_array LcdILI9488::SetGraphicsAddress(uint16_t *_ecv_array buffer, PixelNumber rBegin, PixelNumber rEnd, PixelNumber cBegin, PixelNumber cEnd) noexcept
{
	buffer[0] = CmdPageAddressSet;
	buffer[1] = (rBegin >> 8) | 0x0100;
	buffer[2] = (rBegin & 0x00FF) | 0x0100;
	buffer[3] = (rEnd >> 8) | 0x0100;
	buffer[4] = (rEnd & 0x00FF) | 0x0100;
	buffer[5] = CmdColumnAddressSet;
	buffer[6] = (cBegin >> 8) | 0x0100;
	buffer[7] = (cBegin & 0x00FF) | 0x0100;
	buffer[8] = (cEnd >> 8) | 0x0100;
	buffer[9] = (cEnd & 0x00FF) | 0x0100;
	return buffer + 10;
}

uint16_t *_ecv_array LcdILI9488::SetRowMode(uint16_t *_ecv_array buffer, bool rowMode) noexcept
{
	if (currentRowColMode == (uint8_t)rowMode)
	{
		buffer[0] = CmdMemoryAccessControl;
		buffer[1] = ((uint16_t)rowMode << 5) | 0x0100;
		currentRowColMode = (uint8_t)rowMode;
		return buffer + 2;
	}
	else
	{
		return buffer;
	}
}

uint16_t *_ecv_array LcdILI9488::SetPixelData(uint16_t *_ecv_array buffer, Colour pixelColour, unsigned int numPixels) noexcept
{
	while (numPixels != 0)
	{
		buffer[0] = (pixelColour.red << 2) | 0x0100;
		buffer[1] = (pixelColour.green << 2) | 0x0100;
		buffer[2] = (pixelColour.blue << 2) | 0x0100;
		buffer += 3;
		--numPixels;
	}
	return buffer;
}

void LcdILI9488::SendBuffer(size_t numWords) const noexcept
{
	digitalWrite(csPin, csPol);
	delayMicroseconds(1);						// ILI9488 needs >= 60ns from CS falling to rising edge of clock
	spiDev.TransceivePacketNineBit(spiBuffer, nullptr, numWords);
	delayMicroseconds(1);						// ILI9488 needs >= 15ns from last falling edge of clock to CS rising
	digitalWrite(csPin, !csPol);
}

#endif

// End
