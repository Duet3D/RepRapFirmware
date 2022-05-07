/*
 * Lcd7567.h
 *
 *  Created on: 15 Jul 2020
 *      Author : Martijn Schiedon and David Crocker
 */

#ifndef SRC_DISPLAY_LCD_ST7567_LCD7567_H_
#define SRC_DISPLAY_LCD_ST7567_LCD7567_H_

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#define ALTERNATIVE_ST7565_FLUSHROW

#include <Display/Lcd/MonoLcd.h>

class Lcd7567 : public MonoLcd
{
public:
	// Construct a GLCD driver.
	Lcd7567(const LcdFont * const fnts[], size_t nFonts) noexcept;

	// Flush just some data, returning true if this needs to be called again
	bool FlushSome() noexcept override;

	// Get the display type
	const char *_ecv_array GetDisplayTypeName() const noexcept override;

protected:
	void HardwareInit() noexcept override;

private:

#ifdef ALTERNATIVE_ST7565_FLUSHROW
	constexpr PixelNumber GetTileWidth() const noexcept { return 1; };
	constexpr PixelNumber GetTileHeight() const noexcept { return 8; };
#else
	// NOTE: these are remnants of a more efficient flush routine, not sure this will return
	constexpr PixelNumber GetTileWidth() const noexcept { return 8; };
	constexpr PixelNumber GetTileHeight() const noexcept { return 8; };
#endif

	void CommandDelay() noexcept;
	void DataDelay() noexcept;
	void SendByte(uint8_t byteToSend) noexcept;
	void SetGraphicsAddress(unsigned int r, unsigned int c) noexcept;
	uint8_t TransformTile(uint8_t data[8], PixelNumber c) noexcept;
	void StartDataTransaction() noexcept;
	void EndDataTransaction() noexcept;
	bool FlushRow() noexcept;
	void SelectDevice() noexcept;
	void DeselectDevice() noexcept;

	constexpr static uint8_t SystemReset = 0xE2;				// 11100010 System reset
	constexpr static uint8_t DisplayOff = 0xAE;					// 10101110 Set display enable to off
	constexpr static uint8_t DisplayOn = 0xAF;					// 10101111 Set display enable to on
	constexpr static uint8_t PixelOff = 0xA4;					// 10100100 Set all pixel off
	constexpr static uint8_t PixelOn = 0xA5;					// 10100101 Set all pixel on

	constexpr static uint8_t SetDisplayStartLine = 0x40;
	constexpr static uint8_t SetPage = 0xB0;

	constexpr static uint8_t SetColumnUpper = 0x10;
	constexpr static uint8_t SetColumnLower = 0x00;

	constexpr static uint8_t SetAdcNormal =  0xA0;
	constexpr static uint8_t SetAdcReverse = 0xA1;

	constexpr static uint8_t SetDisplayNormal = 0xA6;
	constexpr static uint8_t SetDisplayreverse = 0xA7;

	constexpr static uint8_t SetBias9 = 0xA2;
	constexpr static uint8_t SetBias7 = 0xA3;

	constexpr static uint8_t Rmw = 0xE0;
	constexpr static uint8_t RmwClear = 0xEE;
	constexpr static uint8_t SetComNormal = 0xC0;
	constexpr static uint8_t SetComReverse = 0xC8;
	constexpr static uint8_t SetPowerControl = 0x28;
	constexpr static uint8_t SetResistorRatio = 0x20;
	constexpr static uint8_t SetVolumeFirst = 0x81;
	constexpr static uint8_t SetStaticOff = 0xAC;
	constexpr static uint8_t SetStaticOn = 0xAD;
	constexpr static uint8_t SetBoosterFirst = 0xF8;
	constexpr static uint8_t SetBooster234 = 0;
	constexpr static uint8_t SetBooster5 = 1;
	constexpr static uint8_t SetBooster6 = 3;
	constexpr static uint8_t Nop = 0xE3;
	constexpr static uint8_t Test = 0xF0;

	constexpr static unsigned int CommandDelayMicros = 72 - 8;	// 72us required, less 7us time to send the command @ 2.0MHz
	constexpr static unsigned int DataDelayMicros = 4;			// delay between sending data bytes
	constexpr static unsigned int FlushRowDelayMicros = 20;		// Delay between sending each rows when flushing all rows @ 2.0MHz (@ 1.0MHz this is not necessary)
};

#endif

#endif /* SRC_DISPLAY_LCD_ST7567_LCD7567_H_ */
