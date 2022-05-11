// Driver for 128x64 graphical LCD with ST7920 controller
// D Crocker, Escher Technologies Ltd.

#include "Lcd.h"

#if SUPPORT_DIRECT_LCD

Lcd::Lcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts) noexcept
	: numRows(nr), numCols(nc), fonts(fnts), numFonts(nFonts),
	  currentFontNumber(0), textInverted(false), numContinuationBytesLeft(0)
{
}

Lcd::~Lcd()
{
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
		if (column < rightMargin)					// keep column <= rightMargin in the following code
		{
			const uint16_t startChar = currentFont->startCharacter;
			const uint16_t endChar = currentFont->endCharacter;

			if (ch < startChar || ch > endChar)
			{
				ch = 0x007F;			// replace unsupported characters by square box
			}

			const PixelNumber fontHeight = currentFont->height;
			const PixelNumber fontBytesPerColumn = (fontHeight + 7)/8;
			const PixelNumber fontBytesPerChar = (fontBytesPerColumn * currentFont->width) + 1;
			const uint8_t *fontPtr = currentFont->ptr + (fontBytesPerChar * (ch - startChar));
			const uint32_t cmask = (currentFont->height < 32) ? (1u << currentFont->height) - 1 : 0xFFFFFFFF;

			PixelNumber numFontColumns = *fontPtr++;
			PixelNumber columnsLeft = rightMargin - column;
			PixelNumber numSpaces;
			if (lastCharColData != 0)		// if we have written anything other than spaces
			{
				numSpaces = currentFont->numSpaces;

				// Decide whether to add space columns first, and whether to add one less then usual (auto-kerning)
				uint32_t thisCharColData = *reinterpret_cast<const uint32_t*>(fontPtr) & cmask;
				if (thisCharColData == 0)  // for characters with deliberate space column at the start, e.g. decimal point
				{
					thisCharColData = *reinterpret_cast<const uint32_t*>(fontPtr + fontBytesPerChar) & cmask;
				}

				const bool kern = (numSpaces >= 2)
								? ((thisCharColData & lastCharColData) == 0)
								: (((thisCharColData | (thisCharColData << 1)) & (lastCharColData | (lastCharColData << 1))) == 0);
				if (kern)
				{
					--numSpaces;	// kern the character pair
				}

				if (numSpaces > columnsLeft)
				{
					numSpaces = columnsLeft;
				}
				columnsLeft -= numSpaces;
			}
			else
			{
				numSpaces = 0;
			}

			if (numFontColumns > columnsLeft)
			{
				numFontColumns = columnsLeft;
			}

			PixelNumber ySize = fontHeight;
			if (row >= numRows)
			{
				ySize = 0;				// we still execute the code, so that the caller can tell how many columns the text will occupy by writing it off-screen
			}
			else if (ySize > numRows - row)
			{
				ySize = numRows - row;
			}

			if (ySize != 0)
			{
				StartCharacter(ySize, numSpaces, numFontColumns);
			}
			column += numSpaces;

			while (numFontColumns != 0)
			{
				const uint32_t colData = *reinterpret_cast<const uint32_t*>(fontPtr);
				fontPtr += fontBytesPerColumn;
				if ((colData & cmask) != 0)
				{
					lastCharColData = colData & cmask;
				}
				if (ySize != 0)
				{
					WriteColumnData(ySize, colData);
				}
				--numFontColumns;
				++column;
			}
			if (ySize != 0)
			{
				EndCharacter();
			}
		}

		justSetCursor = false;
	}
	return 1;
}

// Clear part of the display and select non-inverted text.
void Lcd::Clear(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept
{
	ClearBlock(top, left, bottom, right, false);

	SetCursor(top, left);
	textInverted = false;
	leftMargin = left;
	rightMargin = right;
}

// Write some spaces
void Lcd::WriteSpaces(PixelNumber numPixels) noexcept
{
	const LcdFont * const currentFont = fonts[currentFontNumber];
	uint8_t ySize = currentFont->height;
	if (row < numRows)
	{
		if (row + ySize > numRows)
		{
			ySize = numRows - row;
		}
		ClearBlock(row, column, row + ySize, column + numPixels, textInverted);
	}
	column += numPixels;
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
	if (column < rightMargin)
	{
		WriteSpaces(rightMargin - column);
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
			lastCharColData = (GetFontHeight() < 32) ? (1u << GetFontHeight()) - 1 : 0xFFFFFFFF;		// force a space when switching between normal and inverted text
		}
	}
}

// Draw a line using the Bresenham Algorithm (thanks Wikipedia)
void Lcd::Line(PixelNumber y0, PixelNumber x0, PixelNumber y1, PixelNumber x1, bool mode) noexcept
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
void Lcd::Circle(PixelNumber x0, PixelNumber y0, PixelNumber radius, bool mode) noexcept
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

#endif

// End
