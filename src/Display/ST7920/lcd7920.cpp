// Driver for 128x64 graphical LCD with ST7920 controller
// D Crocker, Escher Technologies Ltd.

#include "RepRapFirmware.h"
#include "Pins.h"

#if SUPPORT_12864_LCD

#include "lcd7920.h"

const uint32_t SpiClockFrequency = 1000000;			// try 1MHz for now

// LCD basic instructions. These all take 72us to execute except LcdDisplayClear, which takes 1.6ms
const uint8_t LcdDisplayClear = 0x01;
const uint8_t LcdHome = 0x02;
const uint8_t LcdEntryModeSet = 0x06;				// move cursor right and increment address when writing data
const uint8_t LcdDisplayOff = 0x08;
const uint8_t LcdDisplayOn = 0x0C;					// add 0x02 for cursor on and/or 0x01 for cursor blink on
const uint8_t LcdFunctionSetBasicAlpha = 0x20;
const uint8_t LcdFunctionSetBasicGraphic = 0x22;
const uint8_t LcdFunctionSetExtendedAlpha = 0x24;
const uint8_t LcdFunctionSetExtendedGraphic = 0x26;
const uint8_t LcdSetDdramAddress = 0x80;			// add the address we want to set

// LCD extended instructions
const uint8_t LcdSetGdramAddress = 0x80;

const unsigned int LcdCommandDelayMicros = 72 - 24; // 72us required, less 24us time to send the command @ 1MHz
const unsigned int LcdDataDelayMicros = 10;			// delay between sending data bytes
const unsigned int LcdDisplayClearDelayMillis = 3;	// 1.6ms should be enough

const unsigned int numRows = 64;
const unsigned int numCols = 128;

Lcd7920::Lcd7920(uint8_t csPin)
	: currentFont(nullptr), translateFrom(nullptr), translateTo(nullptr), textInverted(false)
{
	device.csPin = csPin;
	device.spiMode = 0;
	device.clockFrequency = SpiClockFrequency;
}

size_t Lcd7920::write(uint8_t ch)
{
	if (translateFrom != 0)
	{
		const char* p = strchr(translateFrom, ch);
		if (p != 0)
		{
			ch = translateTo[p - translateFrom];
		}
	}

	if (currentFont == 0)
	{
		return 0;
	}

	const uint8_t startChar = currentFont->startCharacter;
	const uint8_t endChar = currentFont->endCharacter;

	if (ch < startChar || ch > endChar)
	{
		return 0;
	}

	const uint8_t fontWidth = currentFont->width;
	const uint8_t fontHeight = currentFont->height;
	const uint8_t bytesPerColumn = (fontHeight + 7)/8;
	const uint8_t bytesPerChar = (bytesPerColumn * fontWidth) + 1;
	const uint8_t *fontPtr = currentFont->ptr + (bytesPerChar * (ch - startChar));
	uint16_t cmask = (1 << fontHeight) - 1;

	uint8_t nCols = *fontPtr++;

	// Update dirty rectangle coordinates, except for endCol which we do at the end
	{
		if (startRow > row) { startRow = row; }
		if (startCol > column) { startCol = column; }
		uint8_t nextRow = row + fontHeight;
		if (nextRow > numRows) { nextRow = numRows; }
		if (endRow < nextRow) { endRow = nextRow; }
	}

	// Decide whether to add a space column first (auto-kerning)
	// We don't add a space column before a space character.
	// We add a space column after a space character if we would have added one between the preceding and following characters.
	if (column < rightMargin)
	{
		uint16_t thisCharColData = *fontPtr & cmask;
		if (thisCharColData == 0)  // for characters with deliberate space row at the start, e.g. decimal point
		{
			thisCharColData = *(fontPtr + 2) & cmask;
		}
		const bool wantSpace = ((thisCharColData | (thisCharColData << 1)) & (lastCharColData | (lastCharColData << 1))) != 0;
		if (wantSpace)
		{
			// Add space after character
			uint8_t mask = 0x80 >> (column & 7);
			uint8_t *p = image + ((row * (numCols/8)) + (column/8));
			for (uint8_t i = 0; i < fontHeight && p < (image + sizeof(image)); ++i)
			{
				if (textInverted)
				{
					*p |= mask;
				}
				else
				{
					*p &= ~mask;
				}
				p += (numCols/8);
			}
			++column;
		}
	}

	while (nCols != 0 && column < rightMargin)
	{
		uint16_t colData = *fontPtr;
		fontPtr += bytesPerColumn;
		if (colData != 0)
		{
			lastCharColData = colData & cmask;
		}
		const uint8_t mask1 = 0x80 >> (column & 7);
		const uint8_t mask2 = ~mask1;
		uint8_t *p = image + ((row * (numCols/8)) + (column/8));
		const uint16_t setPixelVal = (textInverted) ? 0 : 1;
		for (uint8_t i = 0; i < fontHeight && p < (image + sizeof(image)); ++i)
		{
			if ((colData & 1) == setPixelVal)
			{
				*p |= mask1;      // set pixel
			}
			else
			{
				*p &= mask2;     // clear pixel
			}
			colData >>= 1;
			p += (numCols/8);
		}
		--nCols;
		++column;
	}
	
	if (column > endCol)
	{
		endCol = column;
	}
	return 1;
}

// Set the right margin. In graphics mode, anything written will be truncated at the right margin. Defaults to the right hand edge of the display.
void Lcd7920::setRightMargin(uint8_t r)
{
	rightMargin = (r > numCols) ? numCols : r;
}

// Clear a rectangle from the current position to the right margin (graphics mode only). The height of the rectangle is the height of the current font.
void Lcd7920::clearToMargin()
{
	if (currentFont != 0)
	{
		if (column < rightMargin)
		{
			const uint8_t fontHeight = currentFont->height;
			// Update dirty rectangle coordinates
			{
			  if (startRow > row) { startRow = row; }
			  if (startCol > column) { startCol = column; }
			  uint8_t nextRow = row + fontHeight;
			  if (nextRow > numRows) { nextRow = numRows; }
			  if (endRow < nextRow) { endRow = nextRow; }
			  if (endCol < rightMargin) { endCol = rightMargin; }
			}
			while (column < rightMargin)
			{
				// Add space after character
				uint8_t mask = 0x80 >> (column & 7);
				uint8_t *p = image + ((row * (numCols/8)) + (column/8));
				for (uint8_t i = 0; i < fontHeight && p < (image + sizeof(image)); ++i)
				{
					if (textInverted)
					{
						*p |= mask;
					}
					else
					{
						*p &= ~mask;
					}
					p += (numCols/8);
				}
				++column;
			}
		}
	}
}

// Select normal or inverted text
void Lcd7920::textInvert(bool b)
{
	if (b != textInverted)
	{
		textInverted = b;
		lastCharColData = 0xFFFF;    // always need space between inverted and non-inverted text
	}
}

// NB - if using SPI then the SS pin must be set to be an output before calling this!
void Lcd7920::begin()
{
	sendLcdCommand(LcdFunctionSetBasicAlpha);
	delay(1);
	sendLcdCommand(LcdFunctionSetBasicAlpha);
	commandDelay();
	sendLcdCommand(LcdEntryModeSet);
	commandDelay();
	extendedMode = false;

	clear();
	setCursor(0, 0);
	sendLcdCommand(LcdDisplayOn);
	commandDelay();
	currentFont = 0;
	textInverted = false;
	rightMargin = numCols;
}

void Lcd7920::setFont(const LcdFont *newFont)
{
  currentFont = newFont;
}

void Lcd7920::clear()
{
	memset(image, 0, sizeof(image));

	// Flag whole image as dirty and update the display
	startRow = 0;
	endRow = numRows;
	startCol = 0;
	endCol = numCols;
	flush();
	setCursor(0, 0);
	textInverted = false;
	rightMargin = numCols;
}

// Draw a line using the Bresenham Algorithm (thanks Wikipedia)
void Lcd7920::line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, PixelMode mode)
{
	int dx = (x1 >= x0) ? x1 - x0 : x0 - x1;
	int dy = (y1 >= y0) ? y1 - y0 : y0 - y1;
	int sx = (x0 < x1) ? 1 : -1;
	int sy = (y0 < y1) ? 1 : -1;
	int err = dx - dy;

	for (;;)
	{
		setPixel(x0, y0, mode);
		if (x0 == x1 && y0 == y1) break;
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
void Lcd7920::circle(uint8_t x0, uint8_t y0, uint8_t radius, PixelMode mode)
{
	int f = 1 - (int)radius;
	int ddF_x = 1;
	int ddF_y = -2 * (int)radius;
	int x = 0;
	int y = radius;

	setPixel(x0, y0 + radius, mode);
	setPixel(x0, y0 - radius, mode);
	setPixel(x0 + radius, y0, mode);
	setPixel(x0 - radius, y0, mode);

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
		setPixel(x0 + x, y0 + y, mode);
		setPixel(x0 - x, y0 + y, mode);
		setPixel(x0 + x, y0 - y, mode);
		setPixel(x0 - x, y0 - y, mode);
		setPixel(x0 + y, y0 + x, mode);
		setPixel(x0 - y, y0 + x, mode);
		setPixel(x0 + y, y0 - x, mode);
		setPixel(x0 - y, y0 - x, mode);
	}
}

// Draw a bitmap
void Lcd7920::bitmap(uint8_t x0, uint8_t y0, uint8_t width, uint8_t height, const uint8_t data[])
{
	for (uint8_t r = 0; r < height && r + y0 < numRows; ++r)
	{
		uint8_t *p = image + (((r + y0) * (numCols/8)) + (x0/8));
		uint16_t bitMapOffset = r * (width/8);
		for (uint8_t c = 0; c < (width/8) && c + (x0/8) < numCols/8; ++c)
		{
			*p++ = data[bitMapOffset++];
		}
	}
	if (x0 < startCol) startCol = x0;
	if (x0 + width > endCol) endCol = x0 + width;
	if (y0 < startRow) startRow = y0;
	if (y0 + height > endRow) endRow = y0 + height;
}

// Flush the dirty part of the image to the lcd
void Lcd7920::flush()
{
	if (endCol > startCol && endRow > startRow)
	{
		const uint8_t startColNum = startCol/16;
		const uint8_t endColNum = (endCol + 15)/16;
		for (uint8_t r = startRow; r < endRow; ++r)
		{
			setGraphicsAddress(r, startColNum);
			uint8_t *ptr = image + ((16 * r) + (2 * startColNum));
			for (uint8_t i = startColNum; i < endColNum; ++i)
			{
				sendLcdData(*ptr++);
				//commandDelay();    // don't seem to need a delay here
				sendLcdData(*ptr++);
				//commandDelay();    // don't seem to need as long a delay as this
				delayMicroseconds(LcdDataDelayMicros);
			}
		}
		startRow = numRows;
		startCol = numCols;
		endCol = endRow = 0;
	}
}

void Lcd7920::Update()
{
	//TODO instead of using flush() above, send a limited number of data bytes each time this is called
}

// Set the cursor position. We can only set alternate columns. The row addressing is rather odd.
void Lcd7920::setCursor(uint8_t r, uint8_t c)
{
	row = r % numRows;
	column = c % numCols;
	lastCharColData = 0u;    // flag that we just set the cursor position, so no space before next character
}

void Lcd7920::setPixel(uint8_t x, uint8_t y, PixelMode mode)
{
	if (y < numRows && x < rightMargin)
	{
		uint8_t * const p = image + ((y * (numCols/8)) + (x/8));
		const uint8_t mask = 0x80u >> (x%8);
		switch(mode)
		{
		case PixelClear:
			*p &= ~mask;
			break;
		case PixelSet:
			*p |= mask;
			break;
		case PixelFlip:
			*p ^= mask;
			break;
		}

		// Change the dirty rectangle to account for a pixel being dirty (we assume it was changed)
		if (startRow > y) { startRow = y; }
		if (endRow <= y)  { endRow = y + 1; }
		if (startCol > x) { startCol = x; }
		if (endCol <= x)  { endCol = x + 1; }
	}
}

bool Lcd7920::readPixel(uint8_t x, uint8_t y) const
{
	if (y < numRows && x < numCols)
	{
		const uint8_t * const p = image + ((y * (numCols/8)) + (x/8));
		return (*p & (0x80u >> (x%8))) != 0;
	}
	return false;
}

// Set up translation for characters. Useful for translating fullstop into decimal point, or changing the width of spaces.
// Either the first string passed must be NULL, or the two strings must have equal lengths as returned by strlen().
void Lcd7920::setTranslation(const char *tFrom, const char *tTo)
{
	translateFrom = tFrom;
	translateTo = tTo;
}

void Lcd7920::setGraphicsAddress(unsigned int r, unsigned int c)
{
	ensureExtendedMode();
	sendLcdCommand(LcdSetGdramAddress | (r & 31));
	//commandDelay();  // don't seem to need this one
	sendLcdCommand(LcdSetGdramAddress | c | ((r & 32) >> 2));
	commandDelay();    // we definitely need this one
}

void Lcd7920::commandDelay()
{
	delayMicroseconds(LcdCommandDelayMicros);
}

// Send a command to the LCD
void Lcd7920::sendLcdCommand(uint8_t command)
{
	sendLcd(0xF8, command);
}

// Send a data byte to the LCD
void Lcd7920::sendLcdData(uint8_t data)
{
	sendLcd(0xFA, data);
}

// Send a command to the lcd. Data1 is sent as-is, data2 is split into 2 bytes, high nibble first.
void Lcd7920::sendLcd(uint8_t data1, uint8_t data2)
{
	sspi_acquire();			// TODO when using RTOS, wait for shared SPI to be available
	uint8_t data[3];
	data[0] = data1;
	data[1] = data2 & 0xF0;
	data[2] = data2 << 4;
	sspi_transceive_packet(data, nullptr, 3);
	sspi_release();
}

void Lcd7920::ensureBasicMode()
{
	if (extendedMode)
	{
		sendLcdCommand(LcdFunctionSetBasicGraphic);
		commandDelay();
		extendedMode = false;
	}
}

void Lcd7920::ensureExtendedMode()
{
	if (!extendedMode)
	{
		sendLcdCommand(LcdFunctionSetExtendedGraphic);
		commandDelay();
		extendedMode = true;
	}
}

#endif

// End
