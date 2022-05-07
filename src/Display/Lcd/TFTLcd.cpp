/*
 * TFTLcd.cpp
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#include <Display/Lcd/TFTLcd.h>

TFTLcd::TFTLcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts) noexcept
	: Lcd(nr, nc, fnts, nFonts),
	  fgColour(Colours::White), bgColour(Colours::Blue)
{
	// TODO
}

TFTLcd::~TFTLcd()
{
}

// End
