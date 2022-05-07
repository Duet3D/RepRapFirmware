/*
 * ILI9488.cpp
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#include "ILI9488.h"

#if SUPPORT_ILI9488_LCD

LcdILI9488::LcdILI9488(const LcdFont * const fnts[], size_t nFonts) noexcept
	: TFTLcd(320, 480, fnts, nFonts)
{
	//TODO
}

// Get the display type
const char *_ecv_array LcdILI9488::GetDisplayTypeName() const noexcept
{
	return "480x320 TFT with ILI9488 controller";
}

// Initialise the TFT screen
void LcdILI9488::HardwareInit() noexcept
{
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

#endif

// End
