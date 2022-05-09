/*
 * ILI9488.cpp
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#include "ILI9488.h"
#include <AnalogOut.h>

#if SUPPORT_ILI9488_LCD

constexpr PwmFrequency BacklightPwmFrequency = 1000;

LcdILI9488::LcdILI9488(const LcdFont * const fnts[], size_t nFonts, uint8_t sercomNum) noexcept
	: TFTLcd(320, 480, fnts, nFonts, SpiMode::mode0, sercomNum)
{
}

LcdILI9488::~LcdILI9488()
{
	AnalogOut::Write(LcdBacklightPin, 0.0, BacklightPwmFrequency);
}

// Get the display type
const char *_ecv_array LcdILI9488::GetDisplayTypeName() const noexcept
{
	return "480x320 TFT with ILI9488 controller";
}

// Initialise the TFT screen
void LcdILI9488::HardwareInit() noexcept
{
	const uint16_t Init1[] =	{ CmdPositiveGammaControl,		0x100, 0x103, 0x109, 0x108, 0x116, 0x10A, 0x13F, 0x178, 0x14C, 0x109, 0x10A, 0x108, 0x116, 0x11A, 0x100 };
	const uint16_t Init2[] =	{ CmdNegativeGammaControl,		0x100, 0x116, 0x119, 0x103, 0x10F, 0x105, 0x132, 0x145, 0x146, 0x104, 0x10E, 0x10D, 0x135, 0x137, 0x10F };
	const uint16_t Init3[] =	{ CmdPowerControl1,				0x117, 0x115 };				// Power Control 1, Vreg1out, Vreg2out
	const uint16_t Init4[] =	{ CmdPowerControl2,				0x141 };      				// Power Control 2, VGH, VGL
	const uint16_t Init5[] =	{ CmdVComControl1,				0x100, 0x112, 0x180 };		// Power Control 3, Vcom
	const uint16_t Init6[] =	{ CmdMemoryAccessControl,		0x48 };						// Memory Access
	const uint16_t Init7[] =	{ CmdInterfacePixelFormat,		0x166 };      				// Interface Pixel Format 18 bit
	const uint16_t Init8[] =	{ CmdInterfaceModeControl,		0x180 };     				// Interface Mode Control SDO not used
	const uint16_t Init9[] =	{ CmdFrameRateControlNormal,	0x1A0 };					// Frame rate 60Hz
	const uint16_t Init10[] =	{ CmdDisplayInversionControl,	0x102 };					// Display Inversion Control 2-dot
	const uint16_t Init11[] =	{ CmdDisplayFunctionControl,	0x102, 0x102 };      		// Display Function Control RGB/MCU Interface Control, MCU, Source, Gate scan direction
	const uint16_t Init12[] =	{ CmdSetImageFunction,			0x100 };					// Set Image Function, Disable 24 bit data
	const uint16_t Init13[] =	{ CmdAdjustControl3,			0xA9, 0x51, 0x2C, 0x82 };	// Adjust Control, D7 stream, loose

	SendCommand(CmdReset);
	delay(ResetDelayMillis + 1);

	SendBuffer(Init1, ARRAY_SIZE(Init1));
	SendBuffer(Init2, ARRAY_SIZE(Init2));
	SendBuffer(Init3, ARRAY_SIZE(Init3));
	SendBuffer(Init4, ARRAY_SIZE(Init4));
	SendBuffer(Init5, ARRAY_SIZE(Init5));
	SendBuffer(Init6, ARRAY_SIZE(Init6));
	SendBuffer(Init7, ARRAY_SIZE(Init7));
	SendBuffer(Init8, ARRAY_SIZE(Init8));
	SendBuffer(Init9, ARRAY_SIZE(Init9));
	SendBuffer(Init10, ARRAY_SIZE(Init10));
	SendBuffer(Init11, ARRAY_SIZE(Init11));
	SendBuffer(Init12, ARRAY_SIZE(Init12));
	SendBuffer(Init13, ARRAY_SIZE(Init13));

	SendCommand(0x11);												// Sleep out
	delay(120);

	currentRowColMode = 2;											// force row or column mode to be selected
	ClearBlock(0, 0, numRows, numCols, false);
	SendCommand(CmdDisplayOn);
	AnalogOut::Write(LcdBacklightPin, 1.0, BacklightPwmFrequency);
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
			uint16_t *_ecv_array p = SetColumnMode(spiBuffer, false);
			p = SetGraphicsAddress(p, top, top + rowsToDo - 1, left, right - 1);
			*p++ = CmdMemoryWrite;
			p = SetPixelData(p, (foreground) ? fgColour : bgColour, rowsToDo * (right - left));
			SendBuffer(spiBuffer, p - spiBuffer);
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
	SendBuffer(spiBuffer, p - spiBuffer);
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
void LcdILI9488::WriteColumnData(uint32_t columnData, uint8_t ySize) noexcept
{
	uint16_t *_ecv_array p = SetColumnMode(spiBuffer, true);
	p = SetGraphicsAddress(p, row, row + ySize - 1, column, column);
	*p++ = CmdMemoryWrite;
	for (uint8_t i = 0; i < ySize; ++i)
	{
		p = SetPixelData(p, (columnData & 1u) ? fgColour : bgColour, 1);
		columnData >>= 1;
	}
	SendBuffer(spiBuffer, p - spiBuffer);
}

// Send a parameterless command
void LcdILI9488::SendCommand(uint8_t cmd) noexcept
{
	spiBuffer[0] = cmd;
	SendBuffer(spiBuffer, 1);
}

uint16_t *_ecv_array LcdILI9488::SetGraphicsAddress(uint16_t *_ecv_array buffer, PixelNumber rBegin, PixelNumber rEnd, PixelNumber cBegin, PixelNumber cEnd) noexcept
{
	buffer[0] = CmdPageAddressSet;
	buffer[1] = (cBegin >> 8) | 0x0100;
	buffer[2] = (cBegin & 0x00FF) | 0x0100;
	buffer[3] = (cEnd >> 8) | 0x0100;
	buffer[4] = (cEnd & 0x00FF) | 0x0100;
	buffer[5] = CmdColumnAddressSet;
	buffer[6] = (rBegin >> 8) | 0x0100;
	buffer[7] = (rBegin & 0x00FF) | 0x0100;
	buffer[8] = (rEnd >> 8) | 0x0100;
	buffer[9] = (rEnd & 0x00FF) | 0x0100;
	return buffer + 10;
}

uint16_t *_ecv_array LcdILI9488::SetColumnMode(uint16_t *_ecv_array buffer, bool columnMode) noexcept
{
	if (currentRowColMode == (uint8_t)columnMode)
	{
		buffer[0] = CmdMemoryAccessControl;
		buffer[1] = ((uint16_t)columnMode << 5) | 0x0100;
		currentRowColMode = (uint8_t)columnMode;
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
		// On the ER-TFTM035-6 display the red and blue pixels appear to be swapped, so we must send them in the order blue-green-red
		buffer[0] = (pixelColour.blue << 2) | 0x0100;
		buffer[1] = (pixelColour.green << 2) | 0x0100;
		buffer[2] = (pixelColour.red << 2) | 0x0100;
		buffer += 3;
		--numPixels;
	}
	return buffer;
}

void LcdILI9488::SendBuffer(const uint16_t *_ecv_array buf, size_t numWords) const noexcept
{
	digitalWrite(csPin, csPol);
	delayMicroseconds(1);						// ILI9488 needs >= 60ns from CS falling to rising edge of clock
	spiDev.TransceivePacketNineBit(buf, nullptr, numWords);
	delayMicroseconds(1);						// ILI9488 needs >= 15ns from last falling edge of clock to CS rising
	digitalWrite(csPin, !csPol);
}

#endif

// End
