/*
 * UC1701.h
 *
 *  Created on : 2020-05-14
 *      Author : Martijn Schiedon
 */

#ifndef UC1701_H
#define UC1701_H

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include "SharedSpi.h"
#include <Display/ScreenDriver.h>
#include <Display/Fonts/Fonts.h>

// Class for driving 128x64 graphical LCD fitted with UC1701 controller
class UC1701 : public ScreenDriver
{
public:
	// Construct the driver
	UC1701(PixelNumber width, PixelNumber height, Pin csPin, Pin dcPin) noexcept;

	// The keyword 'override' is optional but makes it explicitly visible
	void Init() noexcept override;
	void SetBusClockFrequency(uint32_t freq) noexcept override;
	void FlushAll() noexcept override;
	bool Flush() noexcept override;

private:
	sspi_device spiDevice;
	// Pin configured to drive DC/A0 line
	Pin dcPin;

	void sendLcdCommand(uint8_t command) noexcept;
	void sendLcdData(uint8_t data) noexcept;
	void sendLcd(uint8_t data) noexcept;
	void commandDelay() noexcept;
	void dataDelay() noexcept;
	void setGraphicsAddress(unsigned int r, unsigned int c) noexcept;
};

#endif

#endif
