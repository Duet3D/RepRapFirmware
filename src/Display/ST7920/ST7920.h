/*
 * ST7920.h
 *
 *  Created on  : 2018-01-22
 *      Author  : David Crocker
 *  Modified on : 2020-05-14
 *      Author  : Martijn Schiedon
 */

// Driver for 128x64 graphical LCD with ST7920 controller

// This controller uses 16 bit wide x 1 bit high refresh tiles.

#ifndef LCD7920_H
#define LCD7920_H

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include <Display/DisplayDriver.h>
#include "SharedSpi.h"

class ST7920 : public DisplayDriver
{
public:
	// Construct a GLCD driver.
	ST7920(PixelNumber width, PixelNumber height, Pin csPin) noexcept;

	// Handler to setup and initialize the specific driver.
	void OnInitialize() noexcept override;
	// Handler to enable the specific driver.
	void OnEnable() noexcept override;
	// Set the SPI clock frequency
	void SetBusClockFrequency(uint32_t freq) noexcept override;
	// Flush the display buffer to the display. Data will not be committed to the display until this is called.
	void FlushAll() noexcept override;
	// Flush just some data, returning true if this needs to be called again
	bool Flush() noexcept override;

private:
	sspi_device device;
	void sendLcdCommand(uint8_t command) noexcept;
	void sendLcdData(uint8_t data) noexcept;
	void sendLcd(uint8_t data1, uint8_t data2) noexcept;
	void commandDelay() noexcept;
	void dataDelay() noexcept;
	void setGraphicsAddress(unsigned int r, unsigned int c) noexcept;

	// LCD basic instructions. These all take 72us to execute except DisplayClear, which takes 1.6ms
	constexpr static uint8_t DisplayClear = 0x01;
	constexpr static uint8_t Home = 0x02;
	constexpr static uint8_t EntryModeSet = 0x06;				// move cursor right and increment address when writing data
	constexpr static uint8_t DisplayOff = 0x08;
	constexpr static uint8_t DisplayOn = 0x0C;					// add 0x02 for cursor on and/or 0x01 for cursor blink on
	constexpr static uint8_t FunctionSetBasicAlpha = 0x20;
	constexpr static uint8_t FunctionSetBasicGraphic = 0x22;
	constexpr static uint8_t FunctionSetExtendedAlpha = 0x24;
	constexpr static uint8_t FunctionSetExtendedGraphic = 0x26;
	constexpr static uint8_t SetDdramAddress = 0x80;			// add the address we want to set
	// LCD extended instructions
	constexpr static uint8_t SetGdramAddress = 0x80;

	constexpr static unsigned int CommandDelayMicros = 72 - 8;	// 72us required, less 7us time to send the command @ 2.0MHz
	constexpr static unsigned int DataDelayMicros = 4;			// delay between sending data bytes
	constexpr static unsigned int DisplayClearDelayMillis = 3;	// 1.6ms should be enough
};

#endif

#endif
