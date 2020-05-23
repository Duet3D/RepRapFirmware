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

inline void Lcd7920::CommandDelay() noexcept
{
	delayMicroseconds(LcdCommandDelayMicros);
}

inline void Lcd7920::DataDelay() noexcept
{
	delayMicroseconds(LcdDataDelayMicros);
}

Lcd7920::Lcd7920(PixelNumber width, PixelNumber height, Pin csPin) noexcept
	: Driver(width, height)
{
	device.csPin = csPin;
	device.csPolarity = true;						// active high chip select
	device.spiMode = 0;
	device.clockFrequency = LcdSpiClockFrequency;
#ifdef __LPC17xx__
    device.sspChannel = LcdSpiChannel;
#endif
}

// Set the SPI clock frequency
void Lcd7920::SetSpiClockFrequency(uint32_t freq) noexcept
{
	device.clockFrequency = freq;
}

void Lcd7920::Init() noexcept
{
	sspi_master_init(&device, 8);
	numContinuationBytesLeft = 0;
	startRow = displayHeight;
	startCol = displayWidth;
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

// Flush all of the dirty part of the image to the lcd. Only called during startup and shutdown.
void Lcd7920::FlushAll() noexcept
{
	while (FlushSome())
	{
		delayMicroseconds(20);			// at 2MHz clock speed we need a delay here, at 1MHz we don't
	}
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
			uint8_t *ptr = imageBuffer + (((displayWidth/8) * nextFlushRow) + (2 * startColNum));
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

		startRow = displayHeight;
		startCol = displayWidth;
		endCol = endRow = nextFlushRow = 0;
	}
	return false;
}

// Set the address to write to. The column address is in 16-bit words, so it ranges from 0 to 7.
void Lcd7920::setGraphicsAddress(unsigned int r, unsigned int c) noexcept
{
	sendLcdCommand(LcdSetGdramAddress | (r & 31));
	//commandDelay();  // don't seem to need this one
	sendLcdCommand(LcdSetGdramAddress | c | ((r & 32) >> 2));
	CommandDelay();    // we definitely need this one
}

// Send a command to the LCD. The SPI mutex is already owned
void Lcd7920::sendLcdCommand(uint8_t command) noexcept
{
	sendLcd(0xF8, command);
}

// Send a data byte to the LCD. The SPI mutex is already owned
void Lcd7920::sendLcdData(uint8_t data) noexcept
{
	sendLcd(0xFA, data);
}

// Send a command to the lcd. Data1 is sent as-is, data2 is split into 2 bytes, high nibble first.
// The SPI mutex is already owned
void Lcd7920::sendLcd(uint8_t data1, uint8_t data2) noexcept
{
	uint8_t data[3];
	data[0] = data1;
	data[1] = data2 & 0xF0;
	data[2] = data2 << 4;
	sspi_transceive_packet(data, nullptr, 3);
}

#endif

// End
