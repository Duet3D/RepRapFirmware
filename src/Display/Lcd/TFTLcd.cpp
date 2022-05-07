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
	imageBuffer = new uint16_t[br * nc];
}

TFTLcd::~TFTLcd()
{
}

// Set the image buffer to start at a particular row, flushing it if necessary
void TFTLcd::SetBufferStartRow(PixelNumber r) noexcept
{
	if (r != bufferStartRow)
	{
		if (IsDirty())
		{
			FlushBuffer();
		}
		bufferStartRow = r;
		dirtyColumnsStart = numCols;
		dirtyColumnsEnd = 0;
	}
}

// End
