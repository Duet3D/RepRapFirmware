/*
 * TFTLcd.cpp
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#include "TFTLcd.h"

TFTLcd::TFTLcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts) noexcept
	: Lcd(nr, nc, fnts, nFonts),
	  fgColour(Colours::White), bgColour(Colours::Blue),
	  bufferStartRow(0), dirtyColumnsStart(nc), dirtyColumnsEnd(0)
{
	// Find the tallest font
	PixelNumber br = 0;
	for (size_t fn = 0; fn < nFonts; ++fn)
	{
		if (fnts[fn]->height > br)
		{
			br = fnts[fn]->height;
		}
	}
	bufferRows = br;
	dirtyRowsStart = br;
	dirtyRowsEnd = 0;
	imageBuffer = new Colour[br * nc];
}

TFTLcd::~TFTLcd()
{
	delete imageBuffer;
	pinMode(csPin, INPUT_PULLUP);
}

// Get the SPI frequency
uint32_t TFTLcd::GetSpiFrequency() const noexcept
{
	//TODO
	return 0;
}

// Initialize the display
void TFTLcd::Init(Pin p_csPin, Pin p_a0Pin, bool csPolarity, uint32_t freq, uint8_t p_contrastRatio, uint8_t p_resistorRatio) noexcept
{
	csPin = p_csPin;
	//TODO
	HardwareInit();
}

// Clear part of the display and select non-inverted text.
void TFTLcd::Clear(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept
{
	while (top < bottom)
	{
		EnsureRowInBuffer(top);
		Colour *p = GetImagePointer(top, left);
		for (PixelNumber col = left; col < right; ++col)
		{
			*p++ = bgColour;
		}
		if (dirtyRowsStart > top - bufferStartRow) { dirtyRowsStart = top - bufferStartRow; }
		if (dirtyRowsEnd <= top - bufferStartRow) { dirtyRowsEnd = (top - bufferStartRow) + 1; }
		if (dirtyColumnsStart > left) { dirtyColumnsStart = left; }
		if (dirtyColumnsEnd < right) { dirtyColumnsEnd = right; }
	}
	textInverted = false;
}

// Set, clear or invert a pixel
//  x = x-coordinate of the pixel, measured from left hand edge of the display
//  y = y-coordinate of the pixel, measured down from the top of the display
//  mode = whether we want to set or clear the pixel
void TFTLcd::SetPixel(PixelNumber y, PixelNumber x, bool mode) noexcept
{
	EnsureRowInBuffer(y);
	*GetImagePointer(y, x) = (mode) ? fgColour : bgColour;
	if (dirtyRowsStart > y - bufferStartRow) { dirtyRowsStart = y - bufferStartRow; }
	if (dirtyRowsEnd <= y - bufferStartRow) { dirtyRowsEnd = (y - bufferStartRow) + 1; }
	if (dirtyColumnsStart > x) { dirtyColumnsStart = x; }
	if (dirtyColumnsEnd <= x) { dirtyColumnsEnd = x + 1; }
}

// Draw a bitmap
//  x0 = x-coordinate of the top left, measured from left hand edge of the display. Currently, must be a multiple of 8.
//  y0 = y-coordinate of the top left, measured down from the top of the display
//  width = width of bitmap in pixels. Currently, must be a multiple of 8.
//  rows = height of bitmap in pixels
//  data = bitmap image, must be ((width/8) * rows) bytes long
void TFTLcd::BitmapImage(PixelNumber top, PixelNumber left, PixelNumber height, PixelNumber width, const uint8_t data[]) noexcept
{
	//TODO
}

// Draw a bitmap row
//  x0 = x-coordinate of the top left, measured from left hand edge of the display
//  y0 = y-coordinate of the top left, measured down from the top of the display
//  width = width of bitmap in pixels
//  data = bitmap image, must be ((width + 7)/8) bytes long
void TFTLcd::BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept
{
	//TODO
}

// Write one column of character data at (row, column)
void TFTLcd::WriteColumnData(uint16_t columnData, uint8_t ySize) noexcept
{
	SetBufferStartRow(row);
	if (ySize > bufferRows) { ySize = bufferRows; }		// should never happen
	Colour *p = GetImagePointer(row, column);
	for (uint8_t i = 0; i < ySize; ++i)
	{
		*p = (columnData & 1u) ? fgColour : bgColour;
		p += numCols;
		columnData >>= 1;
	}
	if (dirtyRowsStart > row - bufferStartRow) { dirtyRowsStart = row - bufferStartRow; }
	if (dirtyRowsEnd <= row + ySize - bufferStartRow) { dirtyRowsEnd = (row + ySize - bufferStartRow) + 1; }
	if (dirtyColumnsStart > column) { dirtyColumnsStart = column; }
	if (dirtyColumnsEnd <= column) { dirtyColumnsEnd = column + 1; }
}

// Set the image buffer to start at a particular row, flushing it if necessary
void TFTLcd::SetBufferStartRow(PixelNumber r) noexcept
{
	if (r != bufferStartRow)
	{
		if (IsDirty())
		{
			FlushAll();
		}
		bufferStartRow = r;
		dirtyColumnsStart = numCols;
		dirtyColumnsEnd = 0;
	}
}

// Ensure that the image buffer includes the specified row, flushing if necessary
void TFTLcd::EnsureRowInBuffer(PixelNumber r) noexcept
{
	if (bufferStartRow > r || bufferStartRow + bufferRows <= r)
	{
		SetBufferStartRow(r);
	}
}

// End
