/*
 * ST7920.cpp
 *
 *  Created on  : 2018-01-22
 *      Author  : David Crocker, Escher Technologies Ltd.
 *  Modified on : 2020-05-14
 *      Author  : Martijn Schiedon
 */

#include <Display/ST7920/ST7920.h>

#if SUPPORT_12864_LCD

#include "Pins.h"
#include "Tasks.h"

// The LCD SPI clock frequency is now defined in the Pins.h file for the configuration being built

inline void ST7920::commandDelay() noexcept
{
	delayMicroseconds(CommandDelayMicros);
}

inline void ST7920::dataDelay() noexcept
{
	delayMicroseconds(DataDelayMicros);
}

ST7920::ST7920(PixelNumber width, PixelNumber height, Pin csPin) noexcept
	: DisplayDriver(width, height)
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
void ST7920::SetBusClockFrequency(uint32_t freq) noexcept
{
	device.clockFrequency = freq;
}

void ST7920::OnInitialize() noexcept
{
	sspi_master_init(&device, 8);

	{
		MutexLocker lock(Tasks::GetSpiMutex());
		sspi_master_setup_device(&device);
		delayMicroseconds(1);
		sspi_select_device(&device);
		delayMicroseconds(1);

		sendLcdCommand(FunctionSetBasicAlpha);
		delay(2);
		sendLcdCommand(FunctionSetBasicAlpha);
		commandDelay();
		sendLcdCommand(EntryModeSet);
		commandDelay();
		sendLcdCommand(DisplayClear);					// need this on some displays to ensure that the alpha RAM is clear (M3D Kanji problem)
		delay(DisplayClearDelayMillis);
		sendLcdCommand(FunctionSetExtendedGraphic);
		commandDelay();

		sspi_deselect_device(&device);
	}
}

void ST7920::OnEnable() noexcept
{
	{
		MutexLocker lock(Tasks::GetSpiMutex());
		sspi_master_setup_device(&device);
		delayMicroseconds(1);
		sspi_select_device(&device);
		delayMicroseconds(1);

		sendLcdCommand(DisplayOn);
		commandDelay();

		sspi_deselect_device(&device);
	}
}

// Flush all of the dirty part of the image to the lcd. Only called during startup and shutdown.
void ST7920::FlushAll() noexcept
{
	while (Flush())
	{
		delayMicroseconds(20);			// at 2MHz clock speed we need a delay here, at 1MHz we don't
	}
}

//TODO: make Flush(bool full = false)
// Flush some of the dirty part of the image to the LCD, returning true if there is more to do
// The dirty area is shrunk by increasing startRow until startRow equals endRow
bool ST7920::Flush() noexcept
{
	// See if there is anything to flush
	if (dirtyRectRight > dirtyRectLeft && dirtyRectBottom > dirtyRectTop)
	{
		// Decide which row to flush next
		if (nextFlushRow < dirtyRectTop || nextFlushRow >= dirtyRectBottom)
		{
			nextFlushRow = dirtyRectTop;	// start from the beginning
		}

		if (nextFlushRow == dirtyRectTop)	// if we are starting from the beginning
		{
			++dirtyRectTop;					// flag this row as flushed because this will happen in the next section
		}

		// Flush that row
		{
			uint8_t startColNum = dirtyRectLeft/16;
			const uint8_t endColNum = (dirtyRectRight + 15)/16;
//			debugPrintf("flush %u %u %u\n", nextFlushRow, startColNum, endColNum);

			MutexLocker lock(Tasks::GetSpiMutex());
			sspi_master_setup_device(&device);
			delayMicroseconds(1);
			sspi_select_device(&device);
			delayMicroseconds(1);

			setGraphicsAddress(nextFlushRow, startColNum);

			uint8_t *ptr = displayBuffer + (((displayWidth/8) * nextFlushRow) + (2 * startColNum));

			while (startColNum < endColNum)
			{
				sendLcdData(*ptr++);
				sendLcdData(*ptr++);
				++startColNum;
				dataDelay();
			}

			sspi_deselect_device(&device);
		}

		if (dirtyRectTop < dirtyRectBottom)
		{
			++nextFlushRow;
			return true;
		}

		// Reset dirty rectangle and row tracking
		dirtyRectTop = displayHeight;
		dirtyRectLeft = displayWidth;
		dirtyRectRight = dirtyRectBottom = nextFlushRow = 0;
	}
	return false;
}

// Set the address to write to. The column address is in 16-bit words, so it ranges from 0 to 7.
void ST7920::setGraphicsAddress(unsigned int r, unsigned int c) noexcept
{
	sendLcdCommand(SetGdramAddress | (r & 31));
	//commandDelay();  // don't seem to need this one
	sendLcdCommand(SetGdramAddress | c | ((r & 32) >> 2));
	commandDelay();    // we definitely need this one
}

// Send a command to the LCD. The SPI mutex is already owned
void ST7920::sendLcdCommand(uint8_t command) noexcept
{
	sendLcd(0xF8, command);
}

// Send a data byte to the LCD. The SPI mutex is already owned
void ST7920::sendLcdData(uint8_t data) noexcept
{
	sendLcd(0xFA, data);
}

// Send a command to the lcd. Data1 is sent as-is, data2 is split into 2 bytes, high nibble first.
// The SPI mutex is already owned
void ST7920::sendLcd(uint8_t data1, uint8_t data2) noexcept
{
	uint8_t data[3];
	data[0] = data1;
	data[1] = data2 & 0xF0;
	data[2] = data2 << 4;
	sspi_transceive_packet(data, nullptr, 3);
}

#endif

// End
