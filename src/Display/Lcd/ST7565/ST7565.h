/*
 * ST7565.h
 *
 *  Created on : 2020-05-14
 *      Author : Martijn Schiedon
 */

#ifndef ST7565_H
#define ST7565_H

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#define ALTERNATIVE_ST7565_FLUSHROW

#include <Display/Lcd/DisplayDriver.h>
#include <Hardware/SharedSpi/SharedSpiClient.h>

// Driver for 128x64 graphical LCD with ST7565, ST7567, UC1701, NT7534 or compatible controller
// This controller uses 1 bit wide x 8 bit high refresh tiles.

class ST7565 : public DisplayDriver
{
public:
	// Construct the driver.
	// The gate pin is the pin to set active high to let MOSI and SCK through.
	ST7565(PixelNumber width, PixelNumber height, Pin csPin, Pin dcPin, bool csPolarity = true, Pin gatePin = NoPin) noexcept;

	// Handler to setup and initialize the specific driver.
	void OnInitialize() noexcept override;
	// Handler to enable the specific driver.
	void OnEnable() noexcept override;
	// Callback to flush the specified data to the display
	void OnFlushRow(PixelNumber startRow, PixelNumber startColumn, PixelNumber endRow, PixelNumber endColumn) noexcept;
	// Let the driver return the delay in microseconds between flushing rows
	unsigned int GetFlushRowDelayMs() const noexcept { return 0; };
	// Set the SPI clock frequency
	void SetBusClockFrequency(uint32_t freq) noexcept override;
	// Functions to return driver-specific flush control details
	// NOTE: the tile width and height returned in pixels MUST be powers of two (1, 2, 4, 8, 16, etc.)
#ifdef ALTERNATIVE_ST7565_FLUSHROW
	constexpr PixelNumber GetTileWidth() const noexcept { return 1; };
	constexpr PixelNumber GetTileHeight() const noexcept { return 8; };
#else
	constexpr PixelNumber GetTileWidth() const noexcept { return 8; };
	constexpr PixelNumber GetTileHeight() const noexcept { return 8; };
#endif

private:
	SharedSpiClient spiDevice;
	// Pin configured to drive DC/A0 line
	Pin dcPin;
	// Optional pin to use as gate for SCK and MOSI
	Pin gatePin;

	void selectDevice() noexcept;
	void deselectDevice() noexcept;
	void sendCommand(uint8_t command) noexcept;
	void sendArg(uint8_t data) noexcept;
	void startDataTransaction() noexcept;
	void sendData(uint8_t data) noexcept;
	void endDataTransaction() noexcept;
	//void sendLcd(uint8_t data) noexcept;
	void commandDelay() noexcept;
	void dataDelay() noexcept;
	void setGraphicsAddress(unsigned int r, unsigned int c) noexcept;
	uint8_t transformTile(uint8_t data[8], PixelNumber c) noexcept;
	void flushEntireBuffer() noexcept;
	void setPinMode(Pin pin, PinMode mode) noexcept;
	void writeDigital(Pin pin, bool high) noexcept;

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
