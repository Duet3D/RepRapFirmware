/*
 * DisplayDriver.cpp
 *
 *  Created on  : 2018-01-22
 *      Author  : David Crocker
 *  Modified on : 2020-05-16
 *      Author  : Martijn Schiedon
 */

/*
 * Ontology of concepts and source file mapping:
 *
 *    DisplayUnit          -> Display.cpp
 *      + DisplayModule
 *        + DisplayDriver    -> DisplayDriver.cpp
 *          - ST7565         -> ST7565.cpp
 *          - ST7920         -> ST7920.cpp
 *        + Display
 *      + RotaryEncoder      -> RotaryEncoder.cpp
 *      + ResetButton
 *      + BackButton
 *      + CardReader
 *      + Speaker            -> Display.cpp
 *
 * Key:
 *     + = part of (composition)
 *     - = choice (inheritance, based on parent concept)
 *
 */

/*
 * Note: display cache memory is organized as a sequential byte array of ((horizontal pixels / 8) * vertical pixels)
 *
 * Example memory of a 128x64 display (the number is the index):
 * 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
 * 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F
 * 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F
 * 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F
 * 40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F
 * 50 51 52 53 54 55 56 57 58 59 5A 5B 5C 5D 5E 5F
 * 60 61 62 63 64 65 66 67 68 69 6A 6B 6C 6D 6E 6F
 * 70 71 72 73 74 75 76 77 78 79 7A 7B 7C 7D 7E 7F
 */

//TODO: instead of tracking dirty areas using a dirty rectangle, consider a bitmap that tracks dirty tiles, adjusted to the display driver,
//      so e.g. for ST7920 16x8, for UC1701 8x8; this would make flushing at a block level possible, with more or less blocks depending on the
//      capabilities of the CPU (and CPU load), and would make the routine much simpler, and most importantly, the area to flush is
//      not a single big rectangular area requiring flushing many pixels, but consists of multiple smaller areas, with (in total) less pixels to flush.
//      it can be adapted to the topology of the driver better, at the cost of more RAM use.
//TODO: is 'invalidate' not a better word to setting something 'dirty' (e.g. InvalidateRect)
//TODO: consistency and standardization of terms and order; top, left, width, height, x, y, rows, columns, r, c, etc.
//      I think that rows/COM and columns/SEG are display hardware terminology, which should be abstracted by functions that use x and y,
//      and rows/columns and x/y sometimes have different meaning and effect (display row could be 8 pixels high)
//      Also, in e.g. function parameter ordering, I prefer to have horizontal come first, then vertical, so x, y and column, row
//TODO: set contrast option, controlled by Code
//TODO: make the A0/CD pin configurable from GCode.

#include <Display/DisplayDriver.h>

#if SUPPORT_12864_LCD

#include "Fonts/Fonts.h"
#include "Tasks.h"

extern const LcdFont font7x11;

const LcdFont * const defaultfonts[] = { &font7x11 };

// Set one default font to use
DisplayDriver::DisplayDriver(PixelNumber width, PixelNumber height) noexcept
	: displayWidth(width), displayHeight(height), fonts(defaultfonts), fontCount(ARRAY_SIZE(defaultfonts)),
	  currentFontNumber(0), numContinuationBytesLeft(0), textInverted(false)
{
	// sizeof(imageBuffer) was used in some places, which doesn't work with a pointer type
	displayBufferSize = ((displayHeight * displayWidth) / 8);
	displayBuffer = new uint8_t[displayBufferSize];
}

DisplayDriver::~DisplayDriver()
{
	delete[] displayBuffer;
}

void DisplayDriver::Init() noexcept
{
	numContinuationBytesLeft = 0;
	currentFontNumber = 0;

	SetFlushDone();

	OnInitialize();

	Clear();
	Flush();

	OnEnable();
}

// Flush all of the dirty part of the image to the display. Only called during startup and shutdown.
void DisplayDriver::Flush() noexcept
{
	while (FlushRow())
	{
		delayMicroseconds(GetFlushRowDelayMs());
	}
}

// Flush some of the dirty part of the image to the LCD (in this case a (partial) row)
// Returns true if there is more to do. The dirty area is shrunk by the height of the flushed row until the area has been cleared
bool DisplayDriver::FlushRow() noexcept
{
	// See if there is anything to flush
	// Right and bottom need to be at least one pixel larger than left and top with this method of marking a dirty rectangle.
	// TODO: perhaps a dirtyRectTop, dirtyRectLeft and dirtyRectWidth, dirtyRectHeight would work more robustly?
	//       if either dirtyRectWidth or dirtyRectHeight are 0, then nothing to flush, and you don't risk off by one errors.
	if (dirtyRectRight > dirtyRectLeft && dirtyRectBottom > dirtyRectTop)
	{
/*
		debugPrintf("flush dl=%u dr=%u dt=%u db=%u  cs=%u ce=%u rs=%u re=%u  nr=%u\n",
				dirtyRectLeft, dirtyRectRight, dirtyRectTop, dirtyRectBottom,
				startColNum, endColNum, startRowNum, endRowNum,
				nextFlushRow);
*/

		// The rows are quantized to whole multiples of the tile height.
		const uint8_t qr = GetTileHeight() - 1;
		uint8_t startRow = dirtyRectTop & ~qr;
		const uint8_t endRow = (dirtyRectBottom + qr) & ~qr;

		// The columns are quantized to whole multiples of the tile width.
		const uint8_t qc = GetTileWidth() - 1;
		uint8_t startColumn = dirtyRectLeft & ~qc;
		const uint8_t endColumn = (dirtyRectRight + qc) & ~qc;

		// Decide which row to flush next
		// It can occur that the dirty area has expanded between subsequent FlushRow() calls
		// To assess if the dirty area has grown, we need to keep track of the previously flushed row and column start and end
		if (flushedRow < startRow || flushedRow >= endRow /*|| startColumn < flushedStartColumn || endColumn >= flushedEndColumn*/)
		{
			// Start over and keep track of what is about to be flushed
			flushedRow = startRow;
		}

		// If we are flushing from the beginning
		if (flushedRow == startRow)
		{
			// Shrink the dirty rectangle pre-emptively (flag as flushed)
			dirtyRectTop = startRow + GetTileHeight();
		}

		// Invoke the driver callback
		OnFlushRow(startRow, startColumn, endRow, endColumn);

		// Check if there is still area to flush
		if (dirtyRectTop < dirtyRectBottom)
		{
			flushedRow += GetTileHeight();
			return true;
		}

		// Reset dirty rectangle and row tracking
		SetFlushDone();
	}

	return false;
}

void DisplayDriver::SetFlushDone() noexcept
{
	dirtyRectTop = displayHeight;
	dirtyRectLeft = displayWidth;
	dirtyRectBottom = dirtyRectRight = flushedRow = 0;
}

void DisplayDriver::SetFonts(const LcdFont * const fnts[], size_t nFonts) noexcept
{
	fonts = fnts;
	fontCount = nFonts;
}

void DisplayDriver::SelectFont(size_t newFont) noexcept
{
	if (newFont < fontCount)
	{
		currentFontNumber = newFont;
	}
}

// Get the current font height
PixelNumber DisplayDriver::GetFontHeight() const noexcept
{
	return GetFontHeight(currentFontNumber);
}

// Get the height of a specified font
PixelNumber DisplayDriver::GetFontHeight(size_t fontNumber) const noexcept
{
	if(fonts == nullptr) return 0;

	if (fontNumber >= fontCount)
	{
		fontNumber = currentFontNumber;
	}

	return fonts[fontNumber]->height;
}

// Flag a pixel as dirty.
// Depending on the display controller, this may lead to refreshing multiple bits at a time.
// A dirty pixel is set by making the right and bottom of the dirty rectangle one pixel larger than its left and top.
// The dirty rectangle area is extended automatically to include other dirty pixels when they are set.
//TODO: check if this is intended behavior, the c + 1 seems counter-intuitive (but consistent, since ClearRect() has that too.
void DisplayDriver::SetPixelDirty(PixelNumber r, PixelNumber c) noexcept
{
	// The r and c parameters must be no greater than displayHeight-1 and displayWidth-1 respectively.
	if (r >= displayHeight || c >= displayWidth) return;
	SetRectDirty(r, c, r + 1, c + 1);
}

// Write a space
void DisplayDriver::WriteSpaces(PixelNumber numPixels) noexcept
{
	const LcdFont * const currentFont = fonts[currentFontNumber];
	uint8_t fontHeight = currentFont->height;

	// Check if we're writing off-screen
	if (cursorRow >= displayHeight)
	{
		// We still execute the code, so that the caller can tell how many columns the text
		// will occupy by writing it off-screen. This is to support center/right alignment of text.
		fontHeight = 0;
	}
	// Check if the operation is partially off-screen
	else if (cursorRow + fontHeight > displayHeight)
	{
		// Determine the height that still fits the screen
		fontHeight = displayHeight - cursorRow;
	}

	while (numPixels != 0 && cursorColumn < displayWidth)
	{
		if (fontHeight != 0)
		{
			const uint8_t mask = 0x80 >> (cursorColumn & 7);
			uint8_t *p = displayBuffer + ((cursorRow * (displayWidth >> 3)) + (cursorColumn >> 3));
			
			for (uint8_t i = 0; i < fontHeight && p < (displayBuffer + displayBufferSize); ++i)
			{
				const uint8_t oldVal = *p;
				const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;
				
				if (newVal != oldVal)
				{
					*p = newVal;
					// Flag the affected area dirty, with a width of 8 bits, since a byte has been written
					SetPixelDirty(cursorRow + i, cursorColumn);
				}
				
				p += (displayWidth/8);
			}
		}
		
		--numPixels;
		++cursorColumn;
	}

	lastCharColData = 0;
	justSetCursor = false;
}

// Set the left margin. This is where the cursor goes to when we print newline.
void DisplayDriver::SetLeftMargin(PixelNumber c) noexcept
{
	leftMargin = min<PixelNumber>(c, displayWidth);
}

// Set the right margin. In graphics mode, anything written will be truncated at the right margin. Defaults to the right hand edge of the display.
void DisplayDriver::SetRightMargin(PixelNumber c) noexcept
{
	rightMargin = min<PixelNumber>(c, displayWidth);
}

// Clear a rectangle from the current position to the right margin. The height of the rectangle is the height of the current font.
void DisplayDriver::ClearToMargin() noexcept
{
	const uint8_t fontHeight = fonts[currentFontNumber]->height;

	while (cursorColumn < rightMargin)
	{
		//TODO: this statement to determine the relevant position in the buffer repeats a lot in the code, make a function out of it?
		uint8_t *p = displayBuffer + ((cursorRow * (displayWidth >> 3)) + (cursorColumn >> 3));
		uint8_t mask = 0xFF >> (cursorColumn & 7);
		PixelNumber nextColumn;

		if ((cursorColumn & (~7)) < (rightMargin & (~7)))
		{
			nextColumn = (cursorColumn & (~7)) + 8;
		}
		else
		{
			mask ^= 0xFF >> (rightMargin & 7);
			nextColumn = rightMargin;
		}

		for (uint8_t i = 0; i < fontHeight && p < (displayBuffer + displayBufferSize); ++i)
		{
			const uint8_t oldVal = *p;
			const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;

			if (newVal != oldVal)
			{
				*p = newVal;
				// Flag the affected area dirty
				SetRectDirty(cursorRow + i, cursorColumn, cursorRow + i + 1, nextColumn);
			}
			
			p += (displayWidth >> 3);
		}
		
		cursorColumn = nextColumn;
	}
}

// Select normal or inverted text
void DisplayDriver::TextInvert(bool b) noexcept
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

// Clear the entire display buffer
void DisplayDriver::Clear() noexcept
{
	ClearRect(0, 0, displayHeight, displayWidth);
}

// Clear a rectangular block of pixels starting at rows, startCol ending just before endRow, endCol
//TODO: seems counter-intuitive that the endRow and endCol are not inclusive and need to be at least start + 1!
//      perhaps top, left, width, height would be better and simpler to process.
void DisplayDriver::ClearRect(PixelNumber startRow, PixelNumber startColumn, PixelNumber endRow, PixelNumber endColumn) noexcept
{
	if (endColumn > displayWidth) { endColumn = displayWidth; }
	if (endRow > displayHeight) { endRow = displayHeight; }

	if (startColumn < endColumn && startRow < endRow)
	{
		// Mask of bits we want to keep in the first byte of each row that we modify
		uint8_t startMask = ~(0xFF >> (startColumn & 7));
		// Mask of bits we want to keep in the last byte of each row that we modify
		const uint8_t endMask = 0xFF >> (endColumn & 7);

		if ((startColumn & ~7) == (endColumn & ~7))
		{
			// Special case of clearing some middle bits
			startMask |= endMask;
		}

		for (PixelNumber row = startRow; row < endRow; ++row)
		{
			uint8_t * p = displayBuffer + ((row * (displayWidth/8)) + (startColumn/8));
			uint8_t * const endp = displayBuffer + ((row * (displayWidth/8)) + (endColumn/8));
			*p &= startMask;

			if (p != endp)
			{
				while (++p < endp)
				{
					*p = 0;
				}
				if ((endColumn & 7) != 0)
				{
					*p &= endMask;
				}
			}
		}

		// Flag cleared part as dirty
		SetRectDirty(startRow, startColumn, endRow, endColumn);

		SetCursor(startRow, startColumn);
		
		textInverted = false;
		leftMargin = startColumn;
		rightMargin = endColumn;
	}
}

// Draw a line using the Bresenham Algorithm (thanks Wikipedia)
void DisplayDriver::DrawLine(PixelNumber y0, PixelNumber x0, PixelNumber y1, PixelNumber x1, PixelMode mode) noexcept
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
void DisplayDriver::DrawCircle(PixelNumber x0, PixelNumber y0, PixelNumber radius, PixelMode mode) noexcept
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

// Draw a bitmap. x0 and displayWidth must be divisible by 8.
//TODO: really fix parameter order consistency (x, y) vs. (top, left) and naming here (row, column, x, y, top, left)
void DisplayDriver::DrawBitmap(PixelNumber x0, PixelNumber y0, PixelNumber width, PixelNumber height, const uint8_t data[]) noexcept
{
	for (PixelNumber r = 0; r < height && r + y0 < displayHeight; ++r)
	{
		uint8_t *p = displayBuffer + (((r + y0) * (displayWidth >> 3)) + (x0 >> 3));
		uint16_t bitMapOffset = r * (width/8);
		
		for (PixelNumber c = 0; c < (width/8) && c + (x0/8) < displayWidth/8; ++c)
		{
			*p++ = data[bitMapOffset++];
		}
	}

	// Assume the whole area has changed
	SetRectDirty(y0, x0, y0 + height, x0 + width);
}

void DisplayDriver::SetRectDirty(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept
{
	//TODO: Cap the coordinates to their maximum values?
	//if (top < dirtyRectTop) dirtyRectTop = min<PixelNumber>(top, displayHeight);
	//etc.

	if (top < dirtyRectTop) dirtyRectTop = top;
	if (left < dirtyRectLeft) dirtyRectLeft = left;
	if (bottom > dirtyRectBottom) dirtyRectBottom = bottom;
	if (right > dirtyRectRight) dirtyRectRight = right;
}

// Draw a single bitmap row. 'left' and 'width' do not need to be divisible by 8.
void DisplayDriver::DrawBitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept
{
	if (width != 0)														// avoid possible arithmetic underflow
	{
		const uint8_t inv = (invert) ? 0xFF : 0;
		uint8_t firstColIndex = left >> 3;								// column index of the first byte to write
		const uint8_t lastColIndex = ((left + width - 1) >> 3);			// column index of the last byte to write
		const unsigned int firstDataShift = left & 7;					// number of bits in the first byte that we leave alone
		uint8_t *p = displayBuffer + (top * displayWidth >> 3) + firstColIndex;

		// Do all bytes except the last one
		uint8_t accumulator = *p & (0xFF << (8 - firstDataShift));		// prime the accumulator
		
		while (firstColIndex < lastColIndex)
		{
			const uint8_t invData = *data ^ inv;
			const uint8_t newVal = accumulator | (invData >> firstDataShift);

			if (newVal != *p)
			{
				*p = newVal;
				SetPixelDirty(top, 8 * firstColIndex);
			}

			accumulator = invData << (8 - firstDataShift);
			++p;
			++data;
			++firstColIndex;
		}

		// Do the last byte. 'accumulator' contains up to 'firstDataShift' of the most significant bits.
		const unsigned int lastDataShift = 7 - ((left + width - 1) & 7);	// number of trailing bits in the last byte that we leave alone, 0 to 7
		const uint8_t lastMask = (1u << lastDataShift) - 1;					// mask for bits we want to keep;
		accumulator |= (*data ^ inv) >> firstDataShift;
		accumulator &= ~lastMask;
		accumulator |= *p & lastMask;

		if (accumulator != *p)
		{
			*p = accumulator;
			SetPixelDirty(top, firstColIndex << 3);
		}
	}
}

// Set the cursor position
void DisplayDriver::SetCursor(PixelNumber r, PixelNumber c) noexcept
{
	cursorRow = r;
	cursorColumn = c;
	lastCharColData = 0u;    // flag that we just set the cursor position, so no space before next character
	justSetCursor = true;
}

void DisplayDriver::SetPixel(PixelNumber y, PixelNumber x, PixelMode mode) noexcept
{
	if (y < displayHeight && x < rightMargin)
	{
		uint8_t * const p = displayBuffer + ((y * (displayWidth >> 3)) + (x >> 3));
		const uint8_t mask = 0x80u >> (x & 7);
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
			SetPixelDirty(y, x);
		}
	}
}

bool DisplayDriver::ReadPixel(PixelNumber x, PixelNumber y) const noexcept
{
	if (x >= displayWidth || y >= displayHeight) return false;
	const uint8_t* const p = displayBuffer + ((y * (displayWidth >> 3)) + (x >> 3));
	return (*p & (0x80u >> (x & 7))) != 0;
}

// Write a UTF8 byte.
// If textYpos is off the end of the display, then don't write anything, just update textXpos and lastCharColData
size_t DisplayDriver::write(uint8_t c) noexcept
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

size_t DisplayDriver::writeNative(uint16_t ch) noexcept
{
	const LcdFont * const currentFont = fonts[currentFontNumber];
	
	if (ch == '\n')
	{
		SetCursor(cursorRow + currentFont->height + 1, leftMargin);
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
		const uint8_t bytesPerColumn = (ySize + 7) >> 3;
		
		if (cursorRow >= displayHeight)
		{
			ySize = 0;				// we still execute the code, so that the caller can tell how many columns the text will occupy by writing it off-screen
		}
		else if (cursorRow + ySize > displayHeight)
		{
			ySize = displayHeight - cursorRow;
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
			
			if (numSpaces != 0 && cursorColumn < rightMargin)
			{
				// Add a single space column after the character
				if (ySize != 0)
				{
					const uint8_t mask = 0x80 >> (cursorColumn & 7);
					uint8_t *p = displayBuffer + ((cursorRow * (displayWidth >> 3)) + (cursorColumn >> 3));

					for (uint8_t i = 0; i < ySize && p < (displayBuffer + displayBufferSize); ++i)
					{
						const uint8_t oldVal = *p;
						const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;
						
						if (newVal != oldVal)
						{
							*p = newVal;
							SetPixelDirty(cursorRow + i, cursorColumn);
						}
						
						p += (displayWidth/8);
					}
				}
				
				++cursorColumn;
			}
		}

		while (nCols != 0 && cursorColumn < rightMargin)
		{
			uint16_t colData = *reinterpret_cast<const uint16_t*>(fontPtr);
			fontPtr += bytesPerColumn;
			
			if (colData != 0)
			{
				lastCharColData = colData & cmask;
			}
			
			if (ySize != 0)
			{
				const uint8_t mask1 = 0x80 >> (cursorColumn & 7);
				const uint8_t mask2 = ~mask1;
				uint8_t *p = displayBuffer + ((cursorRow * (displayWidth >> 3)) + (cursorColumn >> 3));
				const uint16_t setPixelVal = (textInverted) ? 0 : 1;
				
				for (uint8_t i = 0; i < ySize; ++i)
				{
					const uint8_t oldVal = *p;
					const uint8_t newVal = ((colData & 1u) == setPixelVal) ? oldVal | mask1 : oldVal & mask2;
					
					if (newVal != oldVal)
					{
						*p = newVal;
						SetPixelDirty(cursorRow + i, cursorColumn);
					}
					
					colData >>= 1;
					p += (displayWidth >> 3);
				}
			}

			--nCols;
			++cursorColumn;
		}

		justSetCursor = false;
	}
	
	return 1;
}

#endif

// End
