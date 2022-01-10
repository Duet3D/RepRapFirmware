// Driver for 128x64 graphical LCD with ST7920 controller
// D Crocker, Escher Technologies Ltd.

#include "Lcd.h"

#if SUPPORT_12864_LCD

#include <Hardware/SharedSpi/SharedSpiDevice.h>

Lcd::Lcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts, SpiMode mode) noexcept
	: device(SharedSpiDevice::GetMainSharedSpiDevice(), LcdSpiClockFrequency, mode, NoPin, true),
	  numRows(nr), numCols(nc),
	  fonts(fnts), numFonts(nFonts)
{
	imageSize = nr * ((nc + 7)/8);
	image = new uint8_t[imageSize];
}

Lcd::~Lcd()
{
	delete image;
	pinMode(csPin, INPUT_PULLUP);
	pinMode(a0Pin, INPUT_PULLUP);
}

// Initialise. a0Pin is only used by the ST7567.
void Lcd::Init(Pin p_csPin, Pin p_a0Pin, bool csPolarity, uint32_t freq, uint8_t p_contrastRatio, uint8_t p_resistorRatio) noexcept
{
	// All this is SPI-display specific hardware initialisation, which prohibits I2C-display or UART-display support.
	// NOTE: https://github.com/SchmartMaker/RepRapFirmware/tree/ST7565/src/Display did contain this abstraction
	csPin = p_csPin;
	a0Pin = p_a0Pin;
	contrastRatio = p_contrastRatio;
	resistorRatio = p_resistorRatio;
	device.SetClockFrequency(freq);
	device.SetCsPin(csPin);
	device.SetCsPolarity(csPolarity);		// normally active high chip select for ST7920, active low for ST7567
	pinMode(csPin, (csPolarity) ? OUTPUT_LOW : OUTPUT_HIGH);
#ifdef __LPC17xx__
    device.sspChannel = LcdSpiChannel;
#endif

	numContinuationBytesLeft = 0;
	textInverted = false;
	startRow = numRows;
	startCol = numCols;
	endRow = endCol = nextFlushRow = 0;

	HardwareInit();
	currentFontNumber = 0;
}

void Lcd::SetFont(size_t newFont) noexcept
{
	if (newFont < numFonts)
	{
		currentFontNumber = newFont;
	}
}

// Get the current font height
PixelNumber Lcd::GetFontHeight() const noexcept
{
	return fonts[currentFontNumber]->height;
}

// Get the height of a specified font
PixelNumber Lcd::GetFontHeight(size_t fontNumber) const noexcept
{
	if (fontNumber >= numFonts)
	{
		fontNumber = currentFontNumber;
	}
	return fonts[fontNumber]->height;
}

// Flag a rectangle as dirty. Inline because it is called from only two places.
inline void Lcd::SetRectDirty(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept
{
	if (top < startRow) startRow = top;
	if (bottom > endRow) endRow = bottom;
	if (left < startCol) startCol = left;
	if (right > endCol) endCol = right;
}

// Flag a pixel as dirty. The r and c parameters must be no greater than NumRows-1 and NumCols-1 respectively.
void Lcd::SetDirty(PixelNumber r, PixelNumber c) noexcept
{
	SetRectDirty(r, c, r + 1, c + 1);
}

// Write a UTF8 byte.
// If textYpos is off the end of the display, then don't write anything, just update textXpos and lastCharColData
size_t Lcd::write(uint8_t c) noexcept
{
	if (numContinuationBytesLeft == 0)
	{
		if (c < 0x80)
		{
			return writeNative(c);
		}
		else if ((c & 0xE0) == 0xC0)
		{
			charVal = (uint32_t)(c & 0x1F);
			numContinuationBytesLeft = 1;
			return 1;
		}
		else if ((c & 0xF0) == 0xE0)
		{
			charVal = (uint32_t)(c & 0x0F);
			numContinuationBytesLeft = 2;
			return 1;
		}
		else if ((c & 0xF8) == 0xF0)
		{
			charVal = (uint32_t)(c & 0x07);
			numContinuationBytesLeft = 3;
			return 1;
		}
		else if ((c & 0xFC) == 0xF8)
		{
			charVal = (uint32_t)(c & 0x03);
			numContinuationBytesLeft = 4;
			return 1;
		}
		else if ((c & 0xFE) == 0xFC)
		{
			charVal = (uint32_t)(c & 0x01);
			numContinuationBytesLeft = 5;
			return 1;
		}
		else
		{
			return writeNative(0x7F);
		}
	}
	else if ((c & 0xC0) == 0x80)
	{
		charVal = (charVal << 6) | (c & 0x3F);
		--numContinuationBytesLeft;
		if (numContinuationBytesLeft == 0)
		{
			return writeNative((charVal < 0x10000) ? (uint16_t)charVal : 0x007F);
		}
		else
		{
			return 1;
		}
	}
	else
	{
		// Bad UTF8 state
		numContinuationBytesLeft = 0;
		return writeNative(0x7F);
	}
}

size_t Lcd::writeNative(uint16_t ch) noexcept
{
	const LcdFont * const currentFont = fonts[currentFontNumber];
	if (ch == '\n')
	{
		SetCursor(row + currentFont->height + 1, leftMargin);
	}
	else
	{
		const uint8_t startChar = currentFont->startCharacter;
		const uint8_t endChar = currentFont->endCharacter;

		if (ch < startChar || ch > endChar)
		{
			ch = 0x007F;			// replace unsupported characters by square box
		}

		uint8_t ySize = currentFont->height;
		const uint8_t bytesPerColumn = (ySize + 7)/8;
		if (row >= numRows)
		{
			ySize = 0;				// we still execute the code, so that the caller can tell how many columns the text will occupy by writing it off-screen
		}
		else if (row + ySize > numRows)
		{
			ySize = numRows - row;
		}

		const uint8_t bytesPerChar = (bytesPerColumn * currentFont->width) + 1;
		const uint8_t *fontPtr = currentFont->ptr + (bytesPerChar * (ch - startChar));
		const uint16_t cmask = (1u << currentFont->height) - 1;

		uint8_t nCols = *fontPtr++;
		if (lastCharColData != 0)		// if we have written anything other than spaces
		{
			uint8_t numSpaces = currentFont->numSpaces;

			// Decide whether to add a space column first (auto-kerning)
			// We don't add a space column before a space character.
			// We add a space column after a space character if we would have added one between the preceding and following characters.
			uint16_t thisCharColData = *reinterpret_cast<const uint16_t*>(fontPtr) & cmask;
			if (thisCharColData == 0)  // for characters with deliberate space column at the start, e.g. decimal point
			{
				thisCharColData = *reinterpret_cast<const uint16_t*>(fontPtr + 2) & cmask;
			}

			const bool kern = (numSpaces >= 2)
							? ((thisCharColData & lastCharColData) == 0)
							: (((thisCharColData | (thisCharColData << 1)) & (lastCharColData | (lastCharColData << 1))) == 0);
			if (kern)
			{
				--numSpaces;	// kern the character pair
			}
			if (numSpaces != 0 && column < rightMargin)
			{
				// Add a single space column after the character
				if (ySize != 0)
				{
					const uint8_t mask = 0x80 >> (column & 7);
					uint8_t *p = image + ((row * (numCols/8)) + (column/8));
					for (uint8_t i = 0; i < ySize && p < (image + imageSize); ++i)
					{
						const uint8_t oldVal = *p;
						const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;
						if (newVal != oldVal)
						{
							*p = newVal;
							SetDirty(row + i, column);
						}
						p += (numCols/8);
					}
				}
				++column;
			}
		}

		while (nCols != 0 && column < rightMargin)
		{
			uint16_t colData = *reinterpret_cast<const uint16_t*>(fontPtr);
			fontPtr += bytesPerColumn;
			if (colData != 0)
			{
				lastCharColData = colData & cmask;
			}
			if (ySize != 0)
			{
				const uint8_t mask1 = 0x80 >> (column & 7);
				const uint8_t mask2 = ~mask1;
				uint8_t *p = image + ((row * (numCols/8)) + (column/8));
				const uint16_t setPixelVal = (textInverted) ? 0 : 1;
				for (uint8_t i = 0; i < ySize; ++i)
				{
					const uint8_t oldVal = *p;
					const uint8_t newVal = ((colData & 1u) == setPixelVal) ? oldVal | mask1 : oldVal & mask2;
					if (newVal != oldVal)
					{
						*p = newVal;
						SetDirty(row + i, column);
					}
					colData >>= 1;
					p += (numCols/8);
				}
			}
			--nCols;
			++column;
		}

		justSetCursor = false;
	}
	return 1;
}

// Write a space
void Lcd::WriteSpaces(PixelNumber numPixels) noexcept
{
	const LcdFont * const currentFont = fonts[currentFontNumber];
	uint8_t ySize = currentFont->height;
	if (row >= numRows)
	{
		ySize = 0;				// we still execute the code, so that the caller can tell how many columns the text will occupy by writing it off-screen
	}
	else if (row + ySize > numRows)
	{
		ySize = numRows - row;
	}

	while (numPixels != 0 && column < numCols)
	{
		if (ySize != 0)
		{
			const uint8_t mask = 0x80 >> (column & 7);
			uint8_t *p = image + ((row * (numCols/8)) + (column/8));
			for (uint8_t i = 0; i < ySize && p < (image + imageSize); ++i)
			{
				const uint8_t oldVal = *p;
				const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;
				if (newVal != oldVal)
				{
					*p = newVal;
					SetDirty(row + i, column);
				}
				p += (numCols/8);
			}
		}
		--numPixels;
		++column;
	}

	lastCharColData = 0;
	justSetCursor = false;
}

// printf to LCD
int Lcd::printf(const char* fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	int ret = vuprintf([this](char c) -> bool
						{
							if (c != 0)
							{
								write(c);
							}
							return true;
						},
						fmt,
						vargs
					  );
	va_end(vargs);
	return ret;
}

// Set the left margin. This is where the cursor goes to when we print newline.
void Lcd::SetLeftMargin(PixelNumber c) noexcept
{
	leftMargin = min<PixelNumber>(c, numCols);
}

// Set the right margin. In graphics mode, anything written will be truncated at the right margin. Defaults to the right hand edge of the display.
void Lcd::SetRightMargin(PixelNumber r) noexcept
{
	rightMargin = min<PixelNumber>(r, numCols);
}

// Clear a rectangle from the current position to the right margin. The height of the rectangle is the height of the current font.
void Lcd::ClearToMargin() noexcept
{
	const uint8_t fontHeight = fonts[currentFontNumber]->height;
	// TODO make this more efficient by finding the extent of the pixels changed the then calling SetRectDirty just once.
	// Or maybe don't bother, just call SetRectDirty on the whole rectangle
	while (column < rightMargin)
	{
		uint8_t *p = image + ((row * (numCols/8)) + (column/8));
		uint8_t mask = 0xFF >> (column & 7);
		PixelNumber nextColumn;
		if ((column & (~7)) < (rightMargin & (~7)))
		{
			nextColumn = (column & (~7)) + 8;
		}
		else
		{
			mask ^= 0xFF >> (rightMargin & 7);
			nextColumn = rightMargin;
		}

		for (uint8_t i = 0; i < fontHeight && p < (image + imageSize); ++i)
		{
			const uint8_t oldVal = *p;
			const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;
			if (newVal != oldVal)
			{
				*p = newVal;
				SetRectDirty(row + i, column, row + i + 1, nextColumn);
			}
			p += (numCols/8);
		}
		column = nextColumn;
	}
}

// Select normal or inverted text
void Lcd::TextInvert(bool b) noexcept
{
	if (b != textInverted)
	{
		textInverted = b;
		if (!justSetCursor)
		{
			lastCharColData = 0xFFFF;				// force a space when switching between normal and inverted text
		}
	}
}

// Clear a rectangular block of pixels starting at rows, scol ending just before erow, ecol
void Lcd::Clear(PixelNumber sRow, PixelNumber sCol, PixelNumber eRow, PixelNumber eCol) noexcept
{
	if (eCol > numCols) { eCol = numCols; }
	if (eRow > numRows) { eRow = numRows; }
	if (sCol < eCol && sRow < eRow)
	{
		uint8_t sMask = ~(0xFF >> (sCol & 7));		// mask of bits we want to keep in the first byte of each row that we modify
		const uint8_t eMask = 0xFF >> (eCol & 7);	// mask of bits we want to keep in the last byte of each row that we modify
		if ((sCol & ~7) == (eCol & ~7))
		{
			sMask |= eMask;							// special case of just clearing some middle bits
		}
		for (PixelNumber r = sRow; r < eRow; ++r)
		{
			uint8_t * p = image + ((r * (numCols/8)) + (sCol/8));
			uint8_t * const endp = image + ((r * (numCols/8)) + (eCol/8));
			*p &= sMask;
			if (p != endp)
			{
				while (++p < endp)
				{
					*p = 0;
				}
				if ((eCol & 7) != 0)
				{
					*p &= eMask;
				}
			}
		}

		// Flag cleared part as dirty
		if (sCol < startCol) { startCol = sCol; }
		if (eCol >= endCol) { endCol = eCol; }
		if (sRow < startRow) { startRow = sRow; }
		if (eRow >= endRow) { endRow = eRow; }

		SetCursor(sRow, sCol);
		textInverted = false;
		leftMargin = sCol;
		rightMargin = eCol;
	}
}

// Draw a line using the Bresenham Algorithm (thanks Wikipedia)
void Lcd::Line(PixelNumber y0, PixelNumber x0, PixelNumber y1, PixelNumber x1, PixelMode mode) noexcept
{
	int dx = (x1 >= x0) ? x1 - x0 : x0 - x1;
	int dy = (y1 >= y0) ? y1 - y0 : y0 - y1;
	int sx = (x0 < x1) ? 1 : -1;
	int sy = (y0 < y1) ? 1 : -1;
	int err = dx - dy;

	for (;;)
	{
		SetPixel(y0, x0, mode);
		if (x0 == x1 && y0 == y1)
		{
			break;
		}
		int e2 = err + err;
		if (e2 > -dy)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx)
		{
			err += dx;
			y0 += sy;
		}
	}
}

// Draw a circle using the Bresenham Algorithm (thanks Wikipedia)
void Lcd::Circle(PixelNumber x0, PixelNumber y0, PixelNumber radius, PixelMode mode) noexcept
{
	int f = 1 - (int)radius;
	int ddF_x = 1;
	int ddF_y = -2 * (int)radius;
	int x = 0;
	int y = radius;

	SetPixel(y0 + radius, x0, mode);
	SetPixel(y0 - radius, x0, mode);
	SetPixel(y0, x0 + radius, mode);
	SetPixel(y0, x0 - radius, mode);

	while(x < y)
	{
		// keep ddF_x == 2 * x + 1;
		// keep ddF_y == -2 * y;
		// keep f == x*x + y*y - radius*radius + 2*x - y + 1;
		if(f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		SetPixel(y0 + y, x0 + x, mode);
		SetPixel(y0 + y, x0 - x, mode);
		SetPixel(y0 - y, x0 + x, mode);
		SetPixel(y0 - y, x0 - x, mode);
		SetPixel(y0 + x, x0 + y, mode);
		SetPixel(y0 + x, x0 - y, mode);
		SetPixel(y0 - x, x0 + y, mode);
		SetPixel(y0 - x, x0 - y, mode);
	}
}

// Draw a bitmap. x0 and numCols must be divisible by 8.
void Lcd::BitmapImage(PixelNumber x0, PixelNumber y0, PixelNumber width, PixelNumber height, const uint8_t data[]) noexcept
{
	for (PixelNumber r = 0; r < height && r + y0 < numRows; ++r)
	{
		uint8_t *p = image + (((r + y0) * (numCols/8)) + (x0/8));
		uint16_t bitMapOffset = r * (width/8);
		for (PixelNumber c = 0; c < (width/8) && c + (x0/8) < numCols/8; ++c)
		{
			*p++ = data[bitMapOffset++];
		}
	}

	// Assume the whole area has changed
	if (x0 < startCol) startCol = x0;
	if (x0 + width > endCol) endCol = x0 + width;
	if (y0 < startRow) startRow = y0;
	if (y0 + height > endRow) endRow = y0 + height;
}

// Draw a single bitmap row. 'left' and 'width' do not need to be divisible by 8.
void Lcd::BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept
{
	if (width != 0 && top < numRows)									// avoid possible arithmetic underflow or overflowing the buffer
	{
		const uint8_t inv = (invert) ? 0xFF : 0;
		uint8_t firstColIndex = left/8;									// column index of the first byte to write
		const uint8_t lastColIndex = (left + width - 1)/8;				// column index of the last byte to write
		const unsigned int firstDataShift = left % 8;					// number of bits in the first byte that we leave alone
		uint8_t *p = image + (top * numCols/8) + firstColIndex;

		// Do all bytes except the last one
		uint8_t accumulator = *p & (0xFF << (8 - firstDataShift));		// prime the accumulator
		while (firstColIndex < lastColIndex)
		{
			const uint8_t invData = *data ^ inv;
			const uint8_t newVal = accumulator | (invData >> firstDataShift);
			if (newVal != *p)
			{
				*p = newVal;
				SetDirty(top, 8 * firstColIndex);
			}
			accumulator = invData << (8 - firstDataShift);
			++p;
			++data;
			++firstColIndex;
		}

		// Do the last byte. 'accumulator' contains up to 'firstDataShift' of the most significant bits.
		const unsigned int lastDataShift = 7 - ((left + width - 1) % 8);	// number of trailing bits in the last byte that we leave alone, 0 to 7
		const uint8_t lastMask = (1u << lastDataShift) - 1;					// mask for bits we want to keep;
		accumulator |= (*data ^ inv) >> firstDataShift;
		accumulator &= ~lastMask;
		accumulator |= *p & lastMask;
		if (accumulator != *p)
		{
			*p = accumulator;
			SetDirty(top, 8 * firstColIndex);
		}
	}
}

// Flush all of the dirty part of the image to the lcd. Only called during startup and shutdown.
void Lcd::FlushAll() noexcept
{
	while (FlushSome())
	{
		delayMicroseconds(20);			// at 2MHz clock speed we need a delay here, at 1MHz we don't
	}
}

// Set the cursor position
void Lcd::SetCursor(PixelNumber r, PixelNumber c) noexcept
{
	row = r;
	column = c;
	lastCharColData = 0u;    // flag that we just set the cursor position, so no space before next character
	justSetCursor = true;
}

void Lcd::SetPixel(PixelNumber y, PixelNumber x, PixelMode mode) noexcept
{
	if (y < numRows && x < rightMargin)
	{
		uint8_t * const p = image + ((y * (numCols/8)) + (x/8));
		const uint8_t mask = 0x80u >> (x%8);
		const uint8_t oldVal = *p;
		uint8_t newVal;
		switch(mode)
		{
		case PixelMode::PixelClear:
			newVal = oldVal & ~mask;
			break;
		case PixelMode::PixelSet:
			newVal = oldVal | mask;
			break;
		case PixelMode::PixelFlip:
			newVal = oldVal ^ mask;
			break;
		default:
			newVal = oldVal;
			break;
		}

		if (newVal != oldVal)
		{
			*p = newVal;
			SetDirty(y, x);
		}
	}
}

bool Lcd::ReadPixel(PixelNumber x, PixelNumber y) const noexcept
{
	if (y < numRows && x < numCols)
	{
		const uint8_t * const p = image + ((y * (numCols/8)) + (x/8));
		return (*p & (0x80u >> (x%8))) != 0;
	}
	return false;
}

#endif

// End
