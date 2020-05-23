/*
 * ST7920.h
 *
 *  Created on  : 2018-01-22
 *      Author  : David Crocker
 *  Modified on : 2020-05-14
 *      Author  : Martijn Schiedon
 */

// Driver for 128x64 graphical LCD with ST7920 controller

#ifndef LCD7920_H
#define LCD7920_H

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include <Display/ScreenDriver.h>
#include "SharedSpi.h"

class ST7920 : public ScreenDriver
{
public:
	// Construct a GLCD driver.
	ST7920(PixelNumber width, PixelNumber height, Pin csPin) noexcept;

	// Initialize the display. Call this in setup(). Also call setFont to select initial text font.
	void Init() noexcept;
	// Set the SPI clock frequency
	void SetBusClockFrequency(uint32_t freq) noexcept;
	// Flush the display buffer to the display. Data will not be committed to the display until this is called.
	void FlushAll() noexcept;
	// Flush just some data, returning true if this needs to be called again
	bool Flush() noexcept;

private:
	sspi_device device;
	void sendLcdCommand(uint8_t command) noexcept;
	void sendLcdData(uint8_t data) noexcept;
	void sendLcd(uint8_t data1, uint8_t data2) noexcept;
	void commandDelay() noexcept;
	void dataDelay() noexcept;
	void setGraphicsAddress(unsigned int r, unsigned int c) noexcept;
};

#endif

#endif
