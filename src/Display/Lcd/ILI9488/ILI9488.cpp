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

#endif

// End
