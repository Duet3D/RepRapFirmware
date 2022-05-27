/*
 * ER3301_1.cpp
 *
 *  Created on: 27 May 2022
 *      Author: David
 *
 *  Support for ER3303-1 font chip included on some variants of the ER-TFTM035-6 display
 */

#include "ER3301_1.h"

#if USE_FONT_CHIP

#include <Hardware/Spi/SharedSpiDevice.h>

// This font chip contains several fonts. We are only interested in the 12-dot and 16 dot high Unicode Latin/Greek Cyrillic and Arabic fonts.
// We number those as fonts 0 and 1. Fonts 2 and 3 are the same ones expanded to double size.

struct FontTableEntry
{
	uint16_t startCharacter;
	uint16_t endCharacter;
	uint32_t startAddress;
};

struct FontDescriptor
{
	const FontTableEntry *tableStart;
	const FontTableEntry *tableEnd;
	uint16_t height;
	uint16_t spaceWidth;
	unsigned int bytesPerCharacter;
};

static constexpr FontTableEntry Font12[] =
{
	{ 0x0020, 0x007F, 0x19AD22 },						// Latin
	{ 0x00A0, 0x017F, 0x19AD22 + 96 * 26},				// Latin
	{ 0x0384, 0x03CE, 0x19AD22 + 350 * 26 },			// Greek
	{ 0x0400, 0x045F, 0x19AD22 + 425 * 26 },			// Cyrillic
	{ 0x0490, 0x04A3, 0x19AD22 + (425 + 96) * 26 },		// Cyrillic
	{ 0x04AE, 0x04B3, 0x19AD22 + (425 + 117) * 26 },	// Cyrillic
	{ 0x04B8, 0x04BB, 0x19AD22 + (425 + 122) * 26 },	// Cyrillic
	{ 0x04D8, 0x04D9, 0x19AD22 + (425 + 126) * 26 },	// Cyrillic
	{ 0x04E8, 0x04E9, 0x19AD22 + (425 + 128) * 26 },	// Cyrillic
	{ 0x0600, 0x06F9, 0x1AA0E6 }						// Arabic
};

static constexpr FontTableEntry Font16[] =
{
	{ 0x0020, 0x007F, 0x19E580 },						// Latin
	{ 0x00A0, 0x017F, 0x19E580 + 96 * 34},				// Latin
	{ 0x0384, 0x03CE, 0x19E580 + 350 * 34 },			// Greek
	{ 0x0400, 0x045F, 0x19E580 + 425 * 34 },			// Cyrillic
	{ 0x0490, 0x04A3, 0x19E580 + (425 + 96) * 34 },		// Cyrillic
	{ 0x04AE, 0x04B3, 0x19E580 + (425 + 117) * 34 },	// Cyrillic
	{ 0x04B8, 0x04BB, 0x19E580 + (425 + 122) * 34 },	// Cyrillic
	{ 0x04D8, 0x04D9, 0x19E580 + (425 + 126) * 34 },	// Cyrillic
	{ 0x04E8, 0x04E9, 0x19E580 + (425 + 128) * 34 },	// Cyrillic
	{ 0x0600, 0x06F9, 0x1A2F36 }						// Arabic
};

static constexpr FontDescriptor Fonts[] =
{
	{ Font12, Font12 + ARRAY_SIZE(Font12), 12, 1, 26 },
	{ Font16, Font16 + ARRAY_SIZE(Font16), 16, 1, 34 },
};

constexpr bool FontSizeOK(const FontDescriptor *descriptor, size_t numFonts) noexcept
{
	return descriptor->bytesPerCharacter <= ER3301_1::MaxBytesPerCharacter && (numFonts == 1 || FontSizeOK(descriptor + 1, numFonts - 1));
}

static_assert(FontSizeOK(Fonts, ARRAY_SIZE(Fonts)));

// Search for a character in the font table, returning its address of found, 0 if not found
static uint32_t LookupCharacter(uint16_t codePoint, const FontDescriptor& font) noexcept
{
	const FontTableEntry *table = font.tableStart;
	do
	{
		if (codePoint <= table->endCharacter)
		{
			if (codePoint >= table->startCharacter)
			{
				return table->startAddress + ((codePoint - table->startCharacter) * font.bytesPerCharacter);
			}
			break;
		}
		++table;
	} while (table != font.tableEnd);
	return 0;
}

ER3301_1::ER3301_1(Pin csPin) noexcept
	: spiClient(SharedSpiDevice::GetMainSharedSpiDevice(), SpiFrequency, SpiMode::mode0, csPin, false), currentAddress(0)
{
}

// Return the count of available fonts
unsigned int ER3301_1::GetNumFonts() const noexcept
{
	return 2 * ARRAY_SIZE(Fonts);
}

// Return the height of the specified font
uint16_t ER3301_1::GetFontHeight(unsigned int fontNumber) const noexcept
{
	const uint16_t nativeFontHeight = Fonts[fontNumber % ARRAY_SIZE(Fonts)].height;
	return (fontNumber >= ARRAY_SIZE(Fonts)) ? 2 * nativeFontHeight : nativeFontHeight;
}

// Get the inter-character spacing in the specified font
uint16_t ER3301_1::GetSpacing(unsigned int fontNumber) const noexcept
{
	const uint16_t nativeSpacing = Fonts[fontNumber % ARRAY_SIZE(Fonts)].spaceWidth;
	return (fontNumber >= ARRAY_SIZE(Fonts)) ? 2 * nativeSpacing : nativeSpacing;
}

// Fetch the data for the next column and advance the column counter
uint32_t ER3301_1::GetColumnData(uint16_t column) noexcept
{
	uint32_t ret = 0;
	if (doubleSize)
	{
		for (unsigned int row = 0; row < nativeHeight; ++row)
		{
			const uint16_t data = __builtin_bswap16(charBuffer.data[row]);
			if ((data << ((column >> 1) + columnOffset)) & 0x8000)		// if pixel is set
			{
				ret |= 3u << (row << 1);
			}
		}
	}
	else
	{
		for (unsigned int row = 0; row < nativeHeight; ++row)
		{
			const uint16_t data = __builtin_bswap16(charBuffer.data[row]);
			if ((data << (column + columnOffset)) & 0x8000)				// if pixel is set
			{
				ret |= 1u << row;
			}
		}
	}
	return ret;
}

// Fetch the specified character into the buffer, set nextColumn to zero and return its width
uint16_t ER3301_1::GetCharacter(unsigned int fontNumber, uint16_t codePoint) noexcept
{
	uint32_t addr = LookupCharacter(codePoint, Fonts[fontNumber % ARRAY_SIZE(Fonts)]);
	if (addr == 0)
	{
		addr = LookupCharacter(0x007F, Fonts[fontNumber % ARRAY_SIZE(Fonts)]);
		if (addr == 0)
		{
			return 0;
		}
	}
	if (addr != currentAddress)
	{
		// Fetch the character from the font chip
		spiClient.Select();
		charBuffer.cmd = 0x03;
		charBuffer.addr[0] = (uint8_t)(addr >> 16);
		charBuffer.addr[1] = (uint8_t)(addr >> 8);
		charBuffer.addr[2] = (uint8_t)addr;
		spiClient.TransceivePacket((const uint8_t *)&charBuffer, (uint8_t *)&charBuffer, 4 + Fonts[fontNumber % ARRAY_SIZE(Fonts)].bytesPerCharacter);
		spiClient.Deselect();

		currentAddress = addr;
		nativeHeight = Fonts[fontNumber % ARRAY_SIZE(Fonts)].height;		// this is the native height
		nativeWidth = __builtin_bswap16(charBuffer.width);

		// Check whether there is a leading blank column that we need to remove
		uint16_t pixels = 0;
		for (unsigned int row = 0; row < nativeHeight; ++row)
		{
			pixels |= charBuffer.data[row];
		}
		if ((pixels & 0x0080) != 0 && nativeWidth != 0)			// use 0x0080 instead of 0x8000 because we didn't swap the bytes
		{
			columnOffset = 0;
		}
		else
		{
			// Remove the leading space column
			--nativeWidth;
			columnOffset = 1;
		}
	}
	doubleSize = (fontNumber >= ARRAY_SIZE(Fonts));
	return (doubleSize) ? 2 * nativeWidth : nativeWidth;
}

#endif

// End
