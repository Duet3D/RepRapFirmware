/*
 * ER3301_1.h
 *
 *  Created on: 27 May 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_LCD_FONTS_ER3301_1_H_
#define SRC_DISPLAY_LCD_FONTS_ER3301_1_H_

#include <RepRapFirmware.h>

#if USE_FONT_CHIP

#include <Hardware/Spi/SharedSpiClient.h>

class ER3301_1
{
public:
	ER3301_1(Pin csPin) noexcept;

	// Return the count of available fonts
	unsigned int GetNumFonts() const noexcept;

	// Return the height of the specified font
	uint16_t GetFontHeight(unsigned int fontNumber) const noexcept;

	// Get the inter-character spacing in the specified font
	uint16_t GetSpacing(unsigned int fontNumber) const noexcept;

	// Fetch the specified character into the buffer and return its width
	uint16_t GetCharacter(unsigned int fontNumber, uint16_t codePoint) noexcept;

	// Fetch the data for the next column and advance the column counter
	uint32_t GetColumnData(uint16_t column) noexcept;

	static constexpr size_t MaxBytesPerCharacter = 34;

private:
	static constexpr uint32_t SpiFrequency = 4000000;			// the SPI frequency we use (max supported is 20MHz in Read mode, 30MHz in FastRead mode)

	SharedSpiClient spiClient;
	uint32_t currentAddress;									// the address for which we have a character in the buffer
	uint16_t nativeHeight;										// the native height of the character in the buffer
	uint16_t nativeWidth;										// the width of the character in the buffer, after removing a leading space if we need to
	uint16_t columnOffset;										// the number of words of leading spaces we need to skip (we need to manage spaces ourselves to do auto kerning)
	bool doubleSize;											// true if the current font scales the size by two

	struct CharData
	{
		uint8_t cmd;											// space for the Read command
		uint8_t addr[3];										// the 3-byte address we want, MSB first
		uint16_t width;											// the width of the character
		uint16_t data[MaxBytesPerCharacter/2 - 1];				// the character data, one word per row
	};

	CharData charBuffer;										// buffer to hold the SPI command and the fetched font data
};

#endif

#endif /* SRC_DISPLAY_LCD_FONTS_ER3301_1_H_ */
