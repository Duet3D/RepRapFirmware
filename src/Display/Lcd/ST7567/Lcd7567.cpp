/*
 * Lcd7567.cpp
 *
 *  Created on: 15 Jul 2020
 *      Author : Martijn Schiedon and David Crocker
 */

#include "Lcd7567.h"

#if SUPPORT_12864_LCD

constexpr unsigned int TILE_WIDTH = 1;
constexpr unsigned int TILE_HEIGHT = 8;

Lcd7567::Lcd7567(const LcdFont * const fnts[], size_t nFonts) noexcept
	: Lcd(64, 128, fnts, nFonts, SpiMode::mode3)
{
}

// Get the display type
const char *Lcd7567::GetDisplayTypeName() const noexcept
{
	return "128x64 mono graphics with ST7567 controller";
}

void Lcd7567::HardwareInit() noexcept
{
	pinMode(a0Pin, OUTPUT_LOW);				// set DC/A0 pin to be an output with initial LOW state so as to be in command mode (command: 0, data: 1)

	// Post-reset wait of 6ms
	delay(6);

	device.Select();
	delayMicroseconds(1);

#if 0

	SendByte(SetBias7);						// LCD bias select
	SendByte(SetAdcNormal);					// ADC select
	SendByte(SetComNormal);					// SHL select
	SendByte(SetDisplayStartLine);			// Initial display line
	SendByte(SetPowerControl | 0x4);		// turn on voltage converter (VC=1, VR=0, VF=0)
	delay(50);								// wait for 50% rising

	SendByte(SetPowerControl | 0x6);		// turn on voltage regulator (VC=1, VR=1, VF=0)
	delay(50);								// wait >=50ms

	SendByte(SetPowerControl | 0x7);		// turn on voltage follower (VC=1, VR=1, VF=1)
	delay(10);								// wait

	SendByte(SetResistorRatio | 0x6);		// set lcd operating voltage (regulator resistor, ref voltage resistor)

#elif 0

	SendByte(SetDisplayStartLine);			// set display start line
	SendByte(SetAdcReverse);				// ADC set to reverse
	SendByte(SetComNormal);					// common output mode: set scan direction normal operation
	SendByte(SetDisplayNormal);				// display normal (none reverse)
	SendByte(SetBias9);						// LCD bias 1/9
	SendByte(SetPowerControl | 0x7);		// all power control circuits on
	SendByte(SetBoosterFirst);				// set booster ratio to
	SendByte(0x00);							// 4x
	SendByte(SetResistorRatio | 0x07);		// set V0 voltage resistor ratio to large
	SendByte(SetVolumeFirst);				// set contrast
	SendByte(0x018);						// contrast value, EA default: 0x016
	SendByte(SetStaticOff);					// indicator
	SendByte(0x00);							// disable
	SendByte(PixelOff);						// normal display (not all on)
	SendByte(DisplayOn);					// display on
	delay(50);								// delay 50 ms

#else
	SendByte(SystemReset);					// 11100010 System reset
	SendByte(DisplayOff);
	SendByte(SetDisplayStartLine);			// 01000000 Set scroll line to 0 (6-bit value)
	SendByte(SetAdcNormal);					// 10100000 Set SEG (column) direction to MX (mirror = 0)
	SendByte(SetComReverse);				// 11001000 Set COM (row) direction not to MY (mirror = 0)
	SendByte(SetDisplayNormal);				// 10100110 Set inverse display to false
	SendByte(SetBias9);						// 10100010 Set LCD bias ratio BR=0 (1/9th at 1/65 duty)
	SendByte(SetPowerControl | 0x07);		// 00101111 Set power control to enable XV0, V0 and VG charge pumps
	SendByte(SetBoosterFirst);				// 11111000 Set booster ratio (2-byte command) to 4x
	SendByte(0x00);
	SendByte(SetResistorRatio | 0x06);		// 00100011 Set Vlcd resistor ratio 1+Rb/Ra to 6.5 for the voltage regulator (contrast)
	SendByte(SetVolumeFirst);				// 10000001 Set electronic volume (2-byte command) 6-bit contrast value
	SendByte(0x18);
	SendByte(SetStaticOff);					// 10101100 Set static indicator off

#if 0
	// Enter sleep mode
	SendByte(DisplayOff);					// 10101110 Set display enable to off
	SendByte(PixelOn);						// 10100101 Set all pixel on

	delayMicroseconds(1);
	device.Deselect();

	Clear();
	FlushAll();

	device.Select();
	delayMicroseconds(1);
#endif

	// Enable display
	SendByte(PixelOff);						// 10100100 Set all pixel off
	SendByte(DisplayOn);					// 10101111 Set display enable to on

#endif

	delayMicroseconds(1);
	device.Deselect();
}

// Flush just some data, returning true if this needs to be called again
bool Lcd7567::FlushSome() noexcept
{
	// See if there is anything to flush
	if (endCol > startCol && endRow > startRow)
	{
		// Decide which row to flush next
		if (nextFlushRow <= startRow || nextFlushRow >= endRow)
		{
			nextFlushRow = startRow & ~(TILE_HEIGHT - 1);	// start from the beginning
			startRow = nextFlushRow + TILE_HEIGHT;			// flag this row as flushed because it will be soon
		}

		// Flush that row (which is 8 pixels high)
		device.Select();
		delayMicroseconds(1);

		SetGraphicsAddress(nextFlushRow, startCol);
		StartDataTransaction();

		// Send tiles of 1x8 for the desired (quantized) width of the dirty rectangle
		for (int x = startCol; x < endCol; x += TILE_WIDTH)
		{
			uint8_t data = 0;

			// Gather the bits for a vertical line of 8 pixels (LSB is the top pixel)
			for (uint8_t i = 0; i < 8; i++)
			{
				if (ReadPixel(x, nextFlushRow + i))
				{
					data |= (1u << i);
				}
			}

			SendByte(data);
		}

		EndDataTransaction();
		delayMicroseconds(1);
		device.Deselect();

		// Check if there is still area to flush
		if (startRow < endRow)
		{
			nextFlushRow += TILE_HEIGHT;
			return true;
		}

		startRow = numRows;
		startCol = numCols;
		endCol = endRow = nextFlushRow = 0;
	}
	return false;
}

inline void Lcd7567::CommandDelay() noexcept
{
	delayMicroseconds(CommandDelayMicros);
}

inline void Lcd7567::DataDelay() noexcept
{
	delayMicroseconds(DataDelayMicros);
}

inline void Lcd7567::StartDataTransaction() noexcept
{
	digitalWrite(a0Pin, true);
}

inline void Lcd7567::EndDataTransaction() noexcept
{
	digitalWrite(a0Pin, false);
}

void Lcd7567::SendByte(uint8_t byteToSend) noexcept
{
	uint8_t data[1] = { byteToSend };
	device.TransceivePacket(data, nullptr, 1);
}

// Set the address to write to.
// The display memory is organized in 8+1 pages (of horizontal rows) and 0-131 columns
void Lcd7567::SetGraphicsAddress(unsigned int r, unsigned int c) noexcept
{
	// TODO can we send these in one transaction instead of 3?
	SendByte(0x10 | ((c >> 4) & 0b00001111));	// 0001#### Set Column Address MSB
	SendByte(0x00 | (c & 0b00001111));			// 0000#### Set Column Address LSB
	SendByte(0xB0 | ((r >> 3) & 0b00001111));	// 1011#### Set Page Address

	CommandDelay();
}

#endif
// End
