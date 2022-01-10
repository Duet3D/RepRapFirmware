// Driver for 128x64 graphical LCD with ST7920 controller
// D Crocker, Escher Technologies Ltd.

#include "Lcd7920.h"

#if SUPPORT_12864_LCD

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

Lcd7920::Lcd7920(const LcdFont * const fnts[], size_t nFonts) noexcept
	: Lcd(64, 128, fnts, nFonts, SpiMode::mode0)
{
}

// Get the display type
const char *Lcd7920::GetDisplayTypeName() const noexcept
{
	return "128x64 mono graphics with ST7920 controller";
}

void Lcd7920::HardwareInit() noexcept
{
	device.Select();
	delayMicroseconds(1);
	SendLcdCommand(LcdFunctionSetBasicAlpha);
	delay(2);
	SendLcdCommand(LcdFunctionSetBasicAlpha);
	CommandDelay();
	SendLcdCommand(LcdEntryModeSet);
	CommandDelay();
	SendLcdCommand(LcdDisplayClear);					// need this on some displays to ensure that the alpha RAM is clear (M3D Kanji problem)
	delay(LcdDisplayClearDelayMillis);
	SendLcdCommand(LcdFunctionSetExtendedGraphic);
	CommandDelay();
	device.Deselect();

	Clear();
	FlushAll();

	device.Select();
	delayMicroseconds(1);
	SendLcdCommand(LcdDisplayOn);
	CommandDelay();
	device.Deselect();
}

void Lcd7920::CommandDelay() noexcept
{
	delayMicroseconds(LcdCommandDelayMicros);
}

void Lcd7920::DataDelay() noexcept
{
	delayMicroseconds(LcdDataDelayMicros);
}

// Flush some of the dirty part of the image to the LCD, returning true if there is more to do
bool Lcd7920::FlushSome() noexcept
{
	// See if there is anything to flush
	if (endCol > startCol && endRow > startRow)
	{
		// Decide which row to flush next
		if (nextFlushRow < startRow || nextFlushRow >= endRow)
		{
			nextFlushRow = startRow;	// start from the beginning
		}

		if (nextFlushRow == startRow)	// if we are starting from the beginning
		{
			++startRow;					// flag this row as flushed because it will be soon
		}

		// Flush that row
		{
			uint8_t startColNum = startCol/16;
			const uint8_t endColNum = (endCol + 15)/16;
//			debugPrintf("flush %u %u %u\n", nextFlushRow, startColNum, endColNum);

			device.Select();
			delayMicroseconds(1);

			SetGraphicsAddress(nextFlushRow, startColNum);
			uint8_t *ptr = image + (((numCols/8) * nextFlushRow) + (2 * startColNum));
			while (startColNum < endColNum)
			{
				SendLcdData(*ptr++);
				SendLcdData(*ptr++);
				++startColNum;
				DataDelay();
			}
			device.Deselect();
		}

		if (startRow != endRow)
		{
			++nextFlushRow;
			return true;
		}

		startRow = numRows;
		startCol = numCols;
		endCol = endRow = nextFlushRow = 0;
	}
	return false;
}

// Set the address to write to. The column address is in 16-bit words, so it ranges from 0 to 7.
void Lcd7920::SetGraphicsAddress(unsigned int r, unsigned int c) noexcept
{
	SendLcdCommand(LcdSetGdramAddress | (r & 31));
	//commandDelay();  // don't seem to need this one
	SendLcdCommand(LcdSetGdramAddress | c | ((r & 32) >> 2));
	CommandDelay();    // we definitely need this one
}

void Lcd7920::SendLcdCommand(uint8_t byteToSend) noexcept
{
	uint8_t data[3] = { (uint8_t)0xF8, (uint8_t)(byteToSend & 0xF0), (uint8_t)(byteToSend << 4) };
	device.TransceivePacket(data, nullptr, 3);
}

void Lcd7920::SendLcdData(uint8_t byteToSend) noexcept
{
	uint8_t data[3] = { (uint8_t)0xFA, (uint8_t)(byteToSend & 0xF0), (uint8_t)(byteToSend << 4) };
	device.TransceivePacket(data, nullptr, 3);
}

#endif

// End
