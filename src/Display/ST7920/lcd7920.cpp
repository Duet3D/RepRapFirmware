// Driver for 128x64 graphical LCD with ST7920 controller
// D Crocker, Escher Technologies Ltd.

#include "lcd7920.h"

#if SUPPORT_12864_LCD

#include "Pins.h"
#include "Tasks.h"

// The LCD SPI clock frequency is now defined in the Pins.h file for the configuration being built

// LCD basic instructions. These all take 72us to execute except LcdDisplayClear, which takes 1.6ms
constexpr uint8_t LcdDisplayClear = 0x01;
constexpr uint8_t LcdHome = 0x02;
constexpr uint8_t LcdEntryModeSet = 0x06;				// move cursor right and increment address when writing data
constexpr uint8_t LcdDisplayOff = 0x08;
constexpr uint8_t LcdDisplayOn = 0x0C;					// add 0x02 for cursor on and/or 0x01 for cursor blink on
constexpr uint8_t LcdFunctionSetBasicAlpha = 0x20;
constexpr uint8_t LcdFunctionSetBasicGraphic = 0x22;
constexpr uint8_t LcdFunctionSetExtendedAlpha = 0x24;
constexpr uint8_t LcdFunctionSetExtendedGraphic = 0x26;
constexpr uint8_t LcdSetDdramAddress = 0x80;			// add the address we want to set

// LCD extended instructions
constexpr uint8_t LcdSetGdramAddress = 0x80;

constexpr unsigned int LcdCommandDelayMicros = 72 - 8;	// 72us required, less 7us time to send the command @ 2.0MHz
constexpr unsigned int LcdDataDelayMicros = 4;			// delay between sending data bytes
constexpr unsigned int LcdDisplayClearDelayMillis = 3;	// 1.6ms should be enough

inline void Lcd7920::CommandDelay()
{
	delayMicroseconds(LcdCommandDelayMicros);
}

inline void Lcd7920::DataDelay()
{
	delayMicroseconds(LcdDataDelayMicros);
}

Lcd7920::Lcd7920(Pin csPin, const LcdFont * const fnts[], size_t nFonts)
	: fonts(fnts), numFonts(nFonts), currentFontNumber(0), numContinuationBytesLeft(0), textInverted(false)
{
	device.csPin = csPin;
	device.csPolarity = true;						// active high chip select
	device.spiMode = 0;
	device.clockFrequency = LcdSpiClockFrequency;
#if __LPC17xx__
    device.sspChannel = LcdSpiChannel;
#endif
}

// Set the SPI clock frequency
void Lcd7920::SetSpiClockFrequency(uint32_t freq)
{
	device.clockFrequency = freq;
}

void Lcd7920::Init()
{
	sspi_master_init(&device, 8);
	numContinuationBytesLeft = 0;
	startRow = NumRows;
	startCol = NumCols;
	endRow = endCol = nextFlushRow = 0;

	{
		MutexLocker lock(Tasks::GetSpiMutex());
		sspi_master_setup_device(&device);
		delayMicroseconds(1);
		sspi_select_device(&device);
		delayMicroseconds(1);

		sendLcdCommand(LcdFunctionSetBasicAlpha);
		delay(2);
		sendLcdCommand(LcdFunctionSetBasicAlpha);
		CommandDelay();
		sendLcdCommand(LcdEntryModeSet);
		CommandDelay();
		sendLcdCommand(LcdDisplayClear);					// need this on some displays to ensure that the alpha RAM is clear (M3D Kanji problem)
		delay(LcdDisplayClearDelayMillis);
		sendLcdCommand(LcdFunctionSetExtendedGraphic);
		CommandDelay();

		sspi_deselect_device(&device);
	}

	Clear();
	FlushAll();

	{
		MutexLocker lock(Tasks::GetSpiMutex());
		sspi_master_setup_device(&device);
		delayMicroseconds(1);
		sspi_select_device(&device);
		delayMicroseconds(1);
		sendLcdCommand(LcdDisplayOn);
		CommandDelay();
		sspi_deselect_device(&device);
	}
	currentFontNumber = 0;
}

void Lcd7920::SetFont(size_t newFont)
{
	if (newFont < numFonts)
	{
		currentFontNumber = newFont;
	}
}

// Get the current font height
PixelNumber Lcd7920::GetFontHeight() const
{
	return fonts[currentFontNumber]->height;
}

// Get the height of a specified font
PixelNumber Lcd7920::GetFontHeight(size_t fontNumber) const
{
	if (fontNumber >= numFonts)
	{
		fontNumber = currentFontNumber;
	}
	return fonts[fontNumber]->height;
}

// Flag a pixel as dirty. The r and c parameters must be no greater than NumRows-1 and NumCols-1 respectively.
// Only one pixel in each 16-bit word needs to be flagged dirty for the whole word to get refreshed.
void Lcd7920::SetDirty(PixelNumber r, PixelNumber c)
{
//	if (r >= NumRows) { debugPrintf("r=%u\n", r); return; }
//	if (c >= NumCols) { debugPrintf("c=%u\n", c); return; }

	if (c < startCol) { startCol = c; }
	if (c >= endCol) { endCol = c + 1; }
	if (r < startRow) { startRow = r; }
	if (r >= endRow) { endRow = r + 1; }
}

// Write a UTF8 byte.
// If textYpos is off the end of the display, then don't write anything, just update textXpos and lastCharColData
size_t Lcd7920::write(uint8_t c)
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

size_t Lcd7920::writeNative(uint16_t ch)
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
		if (row >= NumRows)
		{
			ySize = 0;				// we still execute the code, so that the caller can tell how many columns the text will occupy by writing it off-screen
		}
		else if (row + ySize > NumRows)
		{
			ySize = NumRows - row;
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
					uint8_t *p = image + ((row * (NumCols/8)) + (column/8));
					for (uint8_t i = 0; i < ySize && p < (image + sizeof(image)); ++i)
					{
						const uint8_t oldVal = *p;
						const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;
						if (newVal != oldVal)
						{
							*p = newVal;
							SetDirty(row + i, column);
						}
						p += (NumCols/8);
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
				uint8_t *p = image + ((row * (NumCols/8)) + (column/8));
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
					p += (NumCols/8);
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
void Lcd7920::WriteSpaces(PixelNumber numPixels)
{
	const LcdFont * const currentFont = fonts[currentFontNumber];
	uint8_t ySize = currentFont->height;
	if (row >= NumRows)
	{
		ySize = 0;				// we still execute the code, so that the caller can tell how many columns the text will occupy by writing it off-screen
	}
	else if (row + ySize > NumRows)
	{
		ySize = NumRows - row;
	}

	while (numPixels != 0 && column < NumCols)
	{
		if (ySize != 0)
		{
			const uint8_t mask = 0x80 >> (column & 7);
			uint8_t *p = image + ((row * (NumCols/8)) + (column/8));
			for (uint8_t i = 0; i < ySize && p < (image + sizeof(image)); ++i)
			{
				const uint8_t oldVal = *p;
				const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;
				if (newVal != oldVal)
				{
					*p = newVal;
					SetDirty(row + i, column);
				}
				p += (NumCols/8);
			}
		}
		--numPixels;
		++column;
	}

	lastCharColData = 0;
	justSetCursor = false;
}

// Set the left margin. This is where the cursor goes to when we print newline.
void Lcd7920::SetLeftMargin(PixelNumber c)
{
	leftMargin = min<uint8_t>(c, NumCols);
}

// Set the right margin. In graphics mode, anything written will be truncated at the right margin. Defaults to the right hand edge of the display.
void Lcd7920::SetRightMargin(PixelNumber r)
{
	rightMargin = min<uint8_t>(r, NumCols);
}

// Clear a rectangle from the current position to the right margin. The height of the rectangle is the height of the current font.
void Lcd7920::ClearToMargin()
{
	const uint8_t fontHeight = fonts[currentFontNumber]->height;
	while (column < rightMargin)
	{
		uint8_t *p = image + ((row * (NumCols/8)) + (column/8));
		uint8_t mask = 0xFF >> (column & 7);
		PixelNumber nextColumn;
		if ((column & (~7)) < (rightMargin & (~7)))
		{
			nextColumn = (column & (~7)) + 8;
		}
		else
		{
			mask ^= 0xFF >> (rightMargin & 7);
			nextColumn = rightMargin;;
		}

		for (uint8_t i = 0; i < fontHeight && p < (image + sizeof(image)); ++i)
		{
			const uint8_t oldVal = *p;
			const uint8_t newVal = (textInverted) ? oldVal | mask : oldVal & ~mask;
			if (newVal != oldVal)
			{
				*p = newVal;
				SetDirty(row + i, column);			// we refresh 16-bit words, so setting 1 pixel dirty in byte will suffice
			}
			p += (NumCols/8);
		}
		column = nextColumn;
	}
}

// Select normal or inverted text
void Lcd7920::TextInvert(bool b)
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
void Lcd7920::Clear(PixelNumber sRow, PixelNumber sCol, PixelNumber eRow, PixelNumber eCol)
{
	if (eCol > NumCols) { eCol = NumCols; }
	if (eRow > NumRows) { eRow = NumRows; }
	if (sCol < eCol && sRow < eRow)
	{
		uint8_t sMask = ~(0xFF >> (sCol & 7));		// mask of bits we want to keep in the first byte of each row that we modify
		const uint8_t eMask = 0xFF >> (eCol & 7);	// mask of bits we want to keep in the last byte of each row that we modify
		if ((sCol & ~7) == (eCol & ~7))
		{
			sMask |= eMask;							// special case of just clearing some middle bits
		}
		for (PixelNumber row = sRow; row < eRow; ++row)
		{
			uint8_t * p = image + ((row * (NumCols/8)) + (sCol/8));
			uint8_t * const endp = image + ((row * (NumCols/8)) + (eCol/8));
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
void Lcd7920::Line(PixelNumber y0, PixelNumber x0, PixelNumber y1, PixelNumber x1, PixelMode mode)
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
void Lcd7920::Circle(PixelNumber x0, PixelNumber y0, PixelNumber radius, PixelMode mode)
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
void Lcd7920::Bitmap(PixelNumber x0, PixelNumber y0, PixelNumber width, PixelNumber height, const uint8_t data[])
{
	for (PixelNumber r = 0; r < height && r + y0 < NumRows; ++r)
	{
		uint8_t *p = image + (((r + y0) * (NumCols/8)) + (x0/8));
		uint16_t bitMapOffset = r * (width/8);
		for (PixelNumber c = 0; c < (width/8) && c + (x0/8) < NumCols/8; ++c)
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
void Lcd7920::BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert)
{
	if (width != 0)														// avoid possible arithmetic underflow
	{
		const uint8_t inv = (invert) ? 0xFF : 0;
		uint8_t firstColIndex = left/8;									// column index of the first byte to write
		const uint8_t lastColIndex = (left + width - 1)/8;				// column index of the last byte to write
		const unsigned int firstDataShift = left % 8;					// number of bits in the first byte that we leave alone
		uint8_t *p = image + (top * NumCols/8) + firstColIndex;

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
void Lcd7920::FlushAll()
{
	while (FlushSome())
	{
		delayMicroseconds(20);			// at 2MHz clock speed we need a delay here, at 1MHz we don't
	}
}

// Flush some of the dirty part of the image to the LCD, returning true if there is more to do
bool Lcd7920::FlushSome()
{
	// See if there is anything to flush
	if (endCol > startCol && endRow > startRow)
	{
		// Decide which row to flush next
		if (nextFlushRow < startRow || nextFlushRow >= endRow)
		{
			nextFlushRow = startRow;	// start from the beginning
		}

		if (nextFlushRow == startRow)	// if we are starting form the beginning
		{
			++startRow;					// flag this row as flushed because it will be soon
		}

		// Flush that row
		{
			uint8_t startColNum = startCol/16;
			const uint8_t endColNum = (endCol + 15)/16;
//			debugPrintf("flush %u %u %u\n", nextFlushRow, startColNum, endColNum);

			MutexLocker lock(Tasks::GetSpiMutex());
			sspi_master_setup_device(&device);
			sspi_select_device(&device);
			delayMicroseconds(1);

			setGraphicsAddress(nextFlushRow, startColNum);
			uint8_t *ptr = image + (((NumCols/8) * nextFlushRow) + (2 * startColNum));
			while (startColNum < endColNum)
			{
				sendLcdData(*ptr++);
				sendLcdData(*ptr++);
				++startColNum;
				DataDelay();
			}
			sspi_deselect_device(&device);
		}

		if (startRow != endRow)
		{
			++nextFlushRow;
			return true;
		}

		startRow = NumRows;
		startCol = NumCols;
		endCol = endRow = nextFlushRow = 0;
	}
	return false;
}

// Set the cursor position
void Lcd7920::SetCursor(PixelNumber r, PixelNumber c)
{
	row = r;
	column = c;
	lastCharColData = 0u;    // flag that we just set the cursor position, so no space before next character
	justSetCursor = true;
}

void Lcd7920::SetPixel(PixelNumber y, PixelNumber x, PixelMode mode)
{
	if (y < NumRows && x < rightMargin)
	{
		uint8_t * const p = image + ((y * (NumCols/8)) + (x/8));
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

bool Lcd7920::ReadPixel(PixelNumber x, PixelNumber y) const
{
	if (y < NumRows && x < NumCols)
	{
		const uint8_t * const p = image + ((y * (NumCols/8)) + (x/8));
		return (*p & (0x80u >> (x%8))) != 0;
	}
	return false;
}

// Set the address to write to. The column address is in 16-bit words, so it ranges from 0 to 7.
void Lcd7920::setGraphicsAddress(unsigned int r, unsigned int c)
{
	sendLcdCommand(LcdSetGdramAddress | (r & 31));
	//commandDelay();  // don't seem to need this one
	sendLcdCommand(LcdSetGdramAddress | c | ((r & 32) >> 2));
	CommandDelay();    // we definitely need this one
}

// Send a command to the LCD. The SPI mutex is already owned
void Lcd7920::sendLcdCommand(uint8_t command)
{
	sendLcd(0xF8, command);
}

// Send a data byte to the LCD. The SPI mutex is already owned
void Lcd7920::sendLcdData(uint8_t data)
{
	sendLcd(0xFA, data);
}

// Send a command to the lcd. Data1 is sent as-is, data2 is split into 2 bytes, high nibble first.
// The SPI mutex is already owned
void Lcd7920::sendLcd(uint8_t data1, uint8_t data2)
{
	uint8_t data[3];
	data[0] = data1;
	data[1] = data2 & 0xF0;
	data[2] = data2 << 4;
	sspi_transceive_packet(data, nullptr, 3);
}

#endif

// End
