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

ST7920::ST7920(PixelNumber width, PixelNumber height, Pin csPin) noexcept
	: DisplayDriver(width, height)
{
	spiDevice.csPin = csPin;
	// Active high chip select
	spiDevice.csPolarity = true;
	spiDevice.spiMode = 0;
	spiDevice.clockFrequency = LcdSpiClockFrequency;
#ifdef __LPC17xx__
    spiDevice.sspChannel = LcdSpiChannel;
#endif
}

// Set the SPI clock frequency
void ST7920::SetBusClockFrequency(uint32_t freq) noexcept
{
	spiDevice.clockFrequency = freq;
}

void ST7920::OnInitialize() noexcept
{
	sspi_master_init(&spiDevice, 8);

	{
		MutexLocker lock(Tasks::GetSpiMutex());
		selectDevice();

		sendLcdCommand(FunctionSetBasicAlpha);
		delay(2);
		sendLcdCommand(FunctionSetBasicAlpha);
		commandDelay();
		sendLcdCommand(EntryModeSet);
		commandDelay();
		// Need this on some displays to ensure that the alpha RAM is clear (M3D Kanji problem)
		sendLcdCommand(DisplayClear);
		delay(DisplayClearDelayMillis);
		sendLcdCommand(FunctionSetExtendedGraphic);
		commandDelay();

		deselectDevice();
	}
}

void ST7920::OnEnable() noexcept
{
	{
		MutexLocker lock(Tasks::GetSpiMutex());
		selectDevice();

		sendLcdCommand(DisplayOn);
		commandDelay();

		deselectDevice();
	}
}

// Flush the specified row
void ST7920::OnFlushRow(PixelNumber startRow, PixelNumber startColumn, PixelNumber endRow, PixelNumber endColumn) noexcept
{
	{
		MutexLocker lock(Tasks::GetSpiMutex());
		selectDevice();

		setGraphicsAddress(startRow, startColumn);

		uint8_t *ptr = displayBuffer + ((startRow * (displayWidth / 8)) + (startColumn / 8));

		while (startColumn < endColumn)
		{
			sendLcdData(*ptr++);
			sendLcdData(*ptr++);
			startColumn += GetTileWidth();
			dataDelay();
		}

		deselectDevice();
	}
}

void ST7920::selectDevice() noexcept
{
	//TODO: can/should the "MutexLocker lock(Tasks::GetSpiMutex());" be moved here as well?
	sspi_master_setup_device(&spiDevice);
	delayMicroseconds(1);
	sspi_select_device(&spiDevice);
	delayMicroseconds(1);
}

void ST7920::deselectDevice() noexcept
{
	delayMicroseconds(1);
	sspi_deselect_device(&spiDevice);
}

// Set the address to write to. The column address is in true coordinates, not 16-bit words anymore.
void ST7920::setGraphicsAddress(unsigned int r, unsigned int c) noexcept
{
	// First set vertical address (bits 5-0)
	sendLcdCommand(SetGdramAddress | (r & 0b00111111));
	// Then set horizontal address (bits 3-0) without delay
	sendLcdCommand(SetGdramAddress | ((c >> 4) & 0b00001111));
	// Standard 72us delay
	commandDelay();
}

inline void ST7920::commandDelay() noexcept
{
	delayMicroseconds(CommandDelayMicros);
}

inline void ST7920::dataDelay() noexcept
{
	delayMicroseconds(DataDelayMicros);
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
