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
#include <Display/DisplayDriver.h>
#include <Display/Fonts/Fonts.h>

// Class for driving 128x64 graphical LCD fitted with UC1701 controller

// This controller uses 1 bit wide x 8 bit high refresh tiles.


class UC1701 : public DisplayDriver
{
public:
	// Construct the driver
	UC1701(PixelNumber width, PixelNumber height, Pin csPin, Pin dcPin) noexcept;

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
	sspi_device spiDevice;
	// Pin configured to drive DC/A0 line
	Pin dcPin;

	void sendCommand(uint8_t command) noexcept;
	void sendArg(uint8_t data) noexcept;
	void startSendData() noexcept;
	void sendData(uint8_t data) noexcept;
	void endSendData() noexcept;
	//void sendLcd(uint8_t data) noexcept;
	void commandDelay() noexcept;
	void dataDelay() noexcept;
	void setGraphicsAddress(unsigned int r, unsigned int c) noexcept;
	uint8_t transformTile(uint8_t data[8], PixelNumber c) noexcept;

	// 11100010 System reset
	constexpr static uint8_t SystemReset = 0xE2;
	// 10101110 Set display enable to off
	constexpr static uint8_t DisplayOff = 0xAE;
	// 10101111 Set display enable to on
	constexpr static uint8_t DisplayOn = 0xAF;
	// 10100100 Set all pixel off
	constexpr static uint8_t PixelOff = 0xA4;
	// 10100101 Set all pixel on
	constexpr static uint8_t PixelOn = 0xA5;

	constexpr static unsigned int CommandDelayMicros = 72 - 8;	 // 72us required, less 7us time to send the command @ 2.0MHz
	constexpr static unsigned int DataDelayMicros = 4;			 // delay between sending data bytes
	constexpr static unsigned int FlushRowDelayMicros = 20;     // Delay between sending each rows when flushing all rows @ 2.0MHz (@ 1.0MHz this is not necessary)
};

#endif

#endif
