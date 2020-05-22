/*
 * Fonts.h
 *
 *  Created on: 23 mei 2020
 *      Author: Martijn Schiedon
 */

#ifndef SRC_DISPLAY_FONTS_FONTS_H_
#define SRC_DISPLAY_FONTS_FONTS_H_

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

// Struct for describing a font table, always held in PROGMEM
struct LcdFont
{
	const uint8_t *ptr;			// pointer to font table
	uint16_t startCharacter;	// Unicode code point of the first character in the font
	uint16_t endCharacter;		// Unicode code point of the last character in the font
	uint8_t height;				// row height in pixels - only this number of pixels will be fetched and drawn - maximum 16 in this version of the software
	uint8_t width;				// max character width in pixels (the font table contains this number of bytes or words per character, plus 1 for the active width)
	uint8_t numSpaces;			// number of space columns between characters before kerning
};

#endif

#endif /* SRC_DISPLAY_FONTS_FONTS_H_ */
