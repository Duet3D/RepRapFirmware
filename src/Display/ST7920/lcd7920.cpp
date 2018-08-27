// Driver for 128x64 graphical LCD with ST7920 controller
// D Crocker, Escher Technologies Ltd.

#include "RepRapFirmware.h"
#include "Pins.h"
#include "Tasks.h"

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

Lcd7920::Lcd7920(uint8_t csPin)
	: m_oCurrentFont(nullptr), m_u8NumContinuationBytesLeft(0), m_bTextInverted(false)
{
	m_oDevice.csPin = csPin;
	m_oDevice.csPolarity = true;						// active high chip select
	m_oDevice.spiMode = 0;
	m_oDevice.clockFrequency = SpiClockFrequency;
}

void Lcd7920::Init()
{
	sspi_master_init(&m_oDevice, 8);
	m_pixnumStartRow = 0;
	m_pixnumStartCol = 0;
	m_pixnumEndRow = 0;
	m_pixnumEndCol = 0;

	sendLcdCommand(LcdFunctionSetBasicAlpha);
	delay(1);

	sendLcdCommand(LcdFunctionSetBasicAlpha);
	commandDelay();

	sendLcdCommand(LcdEntryModeSet);
	commandDelay();
	sendLcdCommand(LcdFunctionSetExtendedGraphic);
	commandDelay();

	Clear();
	FlushAll();

	sendLcdCommand(LcdDisplayOn);
	commandDelay();

	m_oCurrentFont = nullptr;
}

void Lcd7920::SetFont(const LcdFont *newFont)
{
	m_oCurrentFont = newFont;
}

// Write a UTF8 byte.
// If textYpos is off the end of the display, then don't write anything, just update textXpos and lastCharColData
size_t Lcd7920::write(uint8_t c)
{
	// TODO (long term) consider reverting these two to members or making this class a singleton:
	static uint32_t uChar;
	static uint8_t u8BytesLeft = 0;

	if (0 == u8BytesLeft)
	{
		if (c < 0x80)
		{
			return writeNative(c);
		}
		else if ((c & 0xE0) == 0xC0)
		{
			uChar = (uint32_t)(c & 0x1F);
			u8BytesLeft = 1;
			return 1;
		}
		else if ((c & 0xF0) == 0xE0)
		{
			uChar = (uint32_t)(c & 0x0F);
			u8BytesLeft = 2;
			return 1;
		}
		else if ((c & 0xF8) == 0xF0)
		{
			uChar = (uint32_t)(c & 0x07);
			u8BytesLeft = 3;
			return 1;
		}
		else if ((c & 0xFC) == 0xF8)
		{
			uChar = (uint32_t)(c & 0x03);
			u8BytesLeft = 4;
			return 1;
		}
		else if ((c & 0xFE) == 0xFC)
		{
			uChar = (uint32_t)(c & 0x01);
			u8BytesLeft = 5;
			return 1;
		}
		else
		{
			return writeNative(0x7F);
		}
	}
	else if ((c & 0xC0) == 0x80)
	{
		uChar = (uChar << 6) | (c & 0x3F);
		--u8BytesLeft;
		if (u8BytesLeft == 0)
		{
			return writeNative((uChar < 0x10000) ? (uint16_t)uChar : 0x007F);
		}
		else
		{
			return 1;
		}
	}
	else
	{
		// Bad UTF8 state
		u8BytesLeft = 0;
		return writeNative(0x7F);
	}
}

size_t Lcd7920::writeNative(uint16_t ch)
{
	if (nullptr == m_oCurrentFont)
	{
		return 0;
	}

	if (ch == '\n')
	{
		SetCursor(m_pixnumRow + m_oCurrentFont->height + 1, m_pixnumLeftMargin);
	}
	else
	{
		if (ch < m_oCurrentFont->startCharacter || ch > m_oCurrentFont->endCharacter)
		{
			ch = 0x007F;			// replace unsupported characters by square box
		}

		uint8_t ySize = m_oCurrentFont->height;
		const uint8_t bytesPerColumn = (ySize + 7) / 8;
		if (m_pixnumRow >= NumRows)
		{
			ySize = 0;
		}
		else if (m_pixnumRow + ySize > NumRows)
		{
			ySize = NumRows - m_pixnumRow;
		}

		const uint8_t bytesPerChar = (bytesPerColumn * m_oCurrentFont->width) + 1;
		const uint8_t *fontPtr = m_oCurrentFont->ptr + (bytesPerChar * (ch - m_oCurrentFont->startCharacter));
		const uint16_t cmask = (1u << m_oCurrentFont->height) - 1;

		uint8_t nCols = *fontPtr++;

		// Update dirty rectangle coordinates, except for endCol which we do at the end
		{
			if (m_pixnumStartRow > m_pixnumRow) { m_pixnumStartRow = m_pixnumRow; }
			if (m_pixnumStartCol > m_pixnumColumn) { m_pixnumStartCol = m_pixnumColumn; }
			uint8_t nextRow = m_pixnumRow + ySize;
			if (nextRow > NumRows) { nextRow = NumRows; }
			if (m_pixnumEndRow < nextRow) { m_pixnumEndRow = nextRow; }
		}

		if (m_u16LastCharColData != 0) // if we have written anything other than spaces
		{
			uint8_t numSpaces = m_oCurrentFont->numSpaces;

			// Decide whether to add a space column first (auto-kerning)
			// We don't add a space column before a space character.
			// We add a space column after a space character if we would have added one between the preceding and following characters.
			uint16_t thisCharColData = *reinterpret_cast<const uint16_t*>(fontPtr) & cmask;
			if (thisCharColData == 0)  // for characters with deliberate space column at the start, e.g. decimal point
			{
				thisCharColData = *reinterpret_cast<const uint16_t*>(fontPtr + 2) & cmask;
			}

			const bool kern = (numSpaces >= 2)
							? ((thisCharColData & m_u16LastCharColData) == 0)
							: (((thisCharColData | (thisCharColData << 1)) & (m_u16LastCharColData | (m_u16LastCharColData << 1))) == 0);
			if (kern)
			{
				--numSpaces;	// kern the character pair
			}

			if (numSpaces != 0 && m_pixnumColumn < m_pixnumRightMargin + 1)
			{
				// Add a single space column after the character
				if (ySize != 0)
				{
					const uint8_t mask = 0x80 >> (m_pixnumColumn & 7);
					uint8_t *p = m_au8Image + ((m_pixnumRow * (NumCols/8)) + (m_pixnumColumn/8));
					for (uint8_t i = 0; i < ySize && p < (m_au8Image + sizeof(m_au8Image)); ++i)
					{
						if (m_bTextInverted)
						{
							*p |= mask;
						}
						else
						{
							*p &= ~mask;
						}
						p += (NumCols/8);
					}
				}
				++m_pixnumColumn;
			}
		}

		while (nCols != 0 && m_pixnumColumn < m_pixnumRightMargin)
		{
			uint16_t colData = *reinterpret_cast<const uint16_t*>(fontPtr);
			fontPtr += bytesPerColumn;
			if (colData != 0)
			{
				m_u16LastCharColData = colData & cmask;
			}
			const uint8_t mask1 = 0x80 >> (m_pixnumColumn & 7);
			const uint8_t mask2 = ~mask1;
			uint8_t *p = m_au8Image + ((m_pixnumRow * (NumCols/8)) + (m_pixnumColumn/8));
			const uint16_t setPixelVal = (m_bTextInverted) ? 0 : 1;
			for (uint8_t i = 0; i < ySize && p < (m_au8Image + sizeof(m_au8Image)); ++i)
			{
				if ((colData & 1u) == setPixelVal)
				{
					*p |= mask1;      // set pixel
				}
				else
				{
					*p &= mask2;     // clear pixel
				}
				colData >>= 1;
				p += (NumCols/8);
			}
			--nCols;
			++m_pixnumColumn;
		}

		if (m_pixnumColumn > m_pixnumEndCol)
		{
			m_pixnumEndCol = min<uint8_t>(m_pixnumColumn, NumCols);
		}

		m_bJustSetCursor = false;
	}
	return 1;
}

// Set the left margin. This is where the cursor goes to when we print newline.
void Lcd7920::SetLeftMargin(PixelNumber c)
{
	m_pixnumLeftMargin = (c > NumCols) ? NumCols : c;
}

// Set the right margin. In graphics mode, anything written will be truncated at the right margin. Defaults to the right hand edge of the display.
void Lcd7920::SetRightMargin(PixelNumber r)
{
	m_pixnumRightMargin = (r > NumCols) ? NumCols : r;
}

// Clear a rectangle from the current position to the right margin. The height of the rectangle is the height of the current font.
void Lcd7920::ClearToMargin()
{
	if (m_oCurrentFont != nullptr)
	{
		if (m_pixnumColumn < m_pixnumRightMargin)
		{
			const uint8_t fontHeight = m_oCurrentFont->height;
			// Update dirty rectangle coordinates
			{
			  if (m_pixnumStartRow > m_pixnumRow) { m_pixnumStartRow = m_pixnumRow; }
			  if (m_pixnumStartCol > m_pixnumColumn) { m_pixnumStartCol = m_pixnumColumn; }
			  uint8_t nextRow = m_pixnumRow + fontHeight;
			  if (nextRow > NumRows) { nextRow = NumRows; }
			  if (m_pixnumEndRow < nextRow) { m_pixnumEndRow = nextRow; }
			  if (m_pixnumEndCol < m_pixnumRightMargin) { m_pixnumEndCol = m_pixnumRightMargin; }
			}

			while (m_pixnumColumn < m_pixnumRightMargin)
			{
				uint8_t *p = m_au8Image + ((m_pixnumRow * (NumCols / 8)) + (m_pixnumColumn / 8));
				uint8_t mask = 0xFF >> (m_pixnumColumn & 7);
				if ((m_pixnumColumn & (~7)) < (m_pixnumRightMargin & (~7)))
				{
					m_pixnumColumn = (m_pixnumColumn & (~7)) + 8;
				}
				else
				{
					mask ^= 0xFF >> (m_pixnumRightMargin & 7);
					m_pixnumColumn = m_pixnumRightMargin;
				}
				for (uint8_t i = 0; i < fontHeight && p < (m_au8Image + sizeof(m_au8Image)); ++i)
				{
					if (m_bTextInverted)
					{
						*p |= mask;
					}
					else
					{
						*p &= ~mask;
					}
					p += (NumCols/8);
				}
			}
		}
	}
}

// Select normal or inverted text
void Lcd7920::TextInvert(bool b)
{
	if (b != m_bTextInverted)
	{
		m_bTextInverted = b;
		m_u16LastCharColData = (m_bJustSetCursor && !m_bTextInverted) ? 0u : 0xFFFF;
	}
}

void Lcd7920::Clear(PixelNumber sRow, PixelNumber sCol, PixelNumber eRow, PixelNumber eCol)
{
	for (PixelNumber r = sRow; r < eRow; ++r)
	{
		PixelNumber c = sCol;
		if ((c & 7) != 0)
		{
			uint8_t * const p = m_au8Image + ((r * (NumCols/8)) + (c/8));
			*p &= ~(0xFF >> (c & 7));
			c = (c & ~7) + 1;
		}
		while (c < eCol)
		{
			m_au8Image[(r * (NumCols/8)) + (c/8)] = 0;
			c += 8;
		}
		if ((eCol & 7) != 0)
		{
			uint8_t * const p = m_au8Image + ((r * (NumCols/8)) + (c/8));
			*p &= (0xFF >> (c & 7));
		}
	}

	// Flag cleared part as dirty
	m_pixnumStartRow = sRow;
	m_pixnumEndRow = eRow;
	m_pixnumStartCol = sCol;
	m_pixnumEndCol = eCol;
	SetCursor(sRow, sCol);
	m_bTextInverted = false;
	m_pixnumLeftMargin = sCol;
	m_pixnumRightMargin = eCol;
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
		uint8_t *p = m_au8Image + (((r + y0) * (NumCols/8)) + (x0/8));
		uint16_t bitMapOffset = r * (width/8);
		for (PixelNumber c = 0; c < (width/8) && c + (x0/8) < NumCols/8; ++c)
		{
			*p++ = data[bitMapOffset++];
		}
	}
	if (x0 < m_pixnumStartCol) m_pixnumStartCol = x0;
	if (x0 + width > m_pixnumEndCol) m_pixnumEndCol = x0 + width;
	if (y0 < m_pixnumStartRow) m_pixnumStartRow = y0;
	if (y0 + height > m_pixnumEndRow) m_pixnumEndRow = y0 + height;
}

// Flush all of the dirty part of the image to the lcd
void Lcd7920::FlushAll()
{
	while (FlushSome()) { }
}

// Flush some of the dirty part of the image to the LCD, returning true if there is more to do
bool Lcd7920::FlushSome()
{
	static PixelNumber pixnumNextFlushRow = 0;

	// See if there is anything to flush
	if (m_pixnumEndCol > m_pixnumStartCol && m_pixnumEndRow > m_pixnumStartRow)
	{
		// Decide which row to flush next
		if (pixnumNextFlushRow < m_pixnumStartRow || pixnumNextFlushRow >= m_pixnumEndRow)
		{
			pixnumNextFlushRow = m_pixnumStartRow;
			++m_pixnumStartRow;				// flag this row as flushed because it will be soon
		}

		// Flush that row
		const uint8_t startColNum = m_pixnumStartCol/16;
		const uint8_t endColNum = (m_pixnumEndCol + 15)/16;
		setGraphicsAddress(pixnumNextFlushRow, startColNum);
		uint8_t *ptr = m_au8Image + ((16 * pixnumNextFlushRow) + (2 * startColNum));
		for (uint8_t i = startColNum; i < endColNum; ++i)
		{
			sendLcdData(*ptr++);
			//commandDelay();		// don't seem to need a delay here
			sendLcdData(*ptr++);
			//commandDelay();  	 	// don't seem to need as long a delay as this
			delayMicroseconds(LcdDataDelayMicros);
		}

		if (m_pixnumStartRow != m_pixnumEndRow)
		{
			++pixnumNextFlushRow;
			return true;
		}

		m_pixnumStartRow = NumRows;
		m_pixnumStartCol = NumCols;
		m_pixnumEndCol = m_pixnumEndRow = pixnumNextFlushRow = 0;
	}
	return false;
}

// Set the cursor position
void Lcd7920::SetCursor(PixelNumber r, PixelNumber c)
{
	m_pixnumRow = r;
	m_pixnumColumn = c;
	m_u16LastCharColData = (m_bTextInverted) ? 0xFFFF : 0u;    // flag that we just set the cursor position, so no space before next character
	m_bJustSetCursor = true;
}

void Lcd7920::SetPixel(PixelNumber y, PixelNumber x, PixelMode mode)
{
	if (y < NumRows && x < m_pixnumRightMargin)
	{
		uint8_t * const p = m_au8Image + ((y * (NumCols / 8)) + (x / 8));
		const uint8_t mask = 0x80u >> (x % 8);
		switch (mode)
		{
		case PixelMode::PixelClear:
			*p &= ~mask;
			break;
		case PixelMode::PixelSet:
			*p |= mask;
			break;
		case PixelMode::PixelFlip:
			*p ^= mask;
			break;
		}

		// Change the dirty rectangle to account for a pixel being dirty (we assume it was changed)
		if (m_pixnumStartRow > y) { m_pixnumStartRow = y; }
		if (m_pixnumEndRow <= y)  { m_pixnumEndRow = y + 1; }
		if (m_pixnumStartCol > x) { m_pixnumStartCol = x; }
		if (m_pixnumEndCol <= x)  { m_pixnumEndCol = x + 1; }
	}
}

bool Lcd7920::ReadPixel(PixelNumber x, PixelNumber y) const
{
	if (y < NumRows && x < NumCols)
	{
		const uint8_t *const p = m_au8Image + ((y * (NumCols / 8)) + (x / 8));
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
	MutexLocker lock(Tasks::GetSpiMutex());
	sspi_master_setup_device(&m_oDevice);
	delayMicroseconds(1);
	sspi_select_device(&m_oDevice);
	delayMicroseconds(1);
	uint8_t data[3];
	data[0] = data1;
	data[1] = data2 & 0xF0;
	data[2] = data2 << 4;
	sspi_transceive_packet(data, nullptr, 3);
	delayMicroseconds(1);
	sspi_deselect_device(&m_oDevice);
	delayMicroseconds(1);
}

#endif

// End

