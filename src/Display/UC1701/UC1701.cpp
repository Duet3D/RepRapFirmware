/*
 * UC1701.cpp
 *
 *  Created on : 2020-05-14
 *      Author : Martijn Schiedon
 */

// Driver for 128x64 graphical LCD with UC1701, RT7565, NT7534 or compatible controller

// Note: these controllers work with 4-wire SPI:
//   SS = Slave/Chip Select
//   CK = SPI Clock
//   SI = SPI Slave In
//   CD = Command/Data
// For now, the ext_0 pin serves as the CD pin. Hardware SPI is used for the other signals.
// Electrically, this pin should be buffered if the LCD uses 5V logic.

// dc42 19 May 2020, 08:21:
// According to the data sheets and schematics, the ST7567 does not need the SCLK pin to be gated with CS.
// Also the ST7567 uses 3.3V signal levels, and displays using it normally have input level shifters to tolerate 5V signals.
// I believe these should work just fine with 3.3V signal levels. So it should not be necessary to use any buffers.

/*
chip_enable_level = 0,
chip_disable_level = 1,

post_chip_enable_wait_ns = 5,
pre_chip_disable_wait_ns = 5,
reset_pulse_width_ms = 1,
post_reset_wait_ms = 6,
sda_setup_time_ns = 12,
sck_pulse_width_ns = 75,	half of cycle time (100ns according to datasheet), AVR: below 70: 8 MHz, >= 70 --> 4MHz clock
sck_clock_hz = 4000000UL,	since Arduino 1.6.0, the SPI bus speed in Hz. Should be  1000000000/sck_pulse_width_ns
spi_mode = 0,		        active high, rising edge
i2c_bus_clock_100kHz = 4,
data_setup_time_ns = 30,
write_pulse_width_ns = 40,
tile_width = 16,		    width of 16*8=128 pixel
tile_height = 8,
default_x_offset = 0,
flipmode_x_offset = 4,
pixel_width = 128,
pixel_height = 64
*/

#include "UC1701.h"

#if SUPPORT_12864_LCD

#include "Pins.h"
#include "Tasks.h"
#include "Hardware/IoPorts.h"

// 10101110 Set display enable to off
constexpr uint8_t LcdDisplayOff = 0xAE;
// 11100010 System reset
constexpr uint8_t LcdSystemReset = 0xE2;

constexpr uint8_t LcdDisplayClear = 0x01;
constexpr uint8_t LcdHome = 0x02;
constexpr uint8_t LcdEntryModeSet = 0x06;
constexpr uint8_t LcdDisplayOn = 0x0C;
constexpr uint8_t LcdFunctionSetBasicAlpha = 0x20;
constexpr uint8_t LcdFunctionSetBasicGraphic = 0x22;
constexpr uint8_t LcdFunctionSetExtendedAlpha = 0x24;
constexpr uint8_t LcdFunctionSetExtendedGraphic = 0x26;
constexpr uint8_t LcdSetDdramAddress = 0x80;

// LCD extended instructions
constexpr uint8_t LcdSetGdramAddress = 0x80;

constexpr unsigned int LcdCommandDelayMicros = 72 - 8;	// 72us required, less 7us time to send the command @ 2.0MHz
constexpr unsigned int LcdDataDelayMicros = 4;			// delay between sending data bytes
constexpr unsigned int LcdDisplayClearDelayMillis = 3;	// 1.6ms should be enough
constexpr unsigned int LcdFlushRowDelayMicros = 20;     // Delay between sending each rows when flushing all rows @ 2.0MHz (@ 1.0MHz this is not necessary)

UC1701::UC1701(PixelNumber width, PixelNumber height, Pin csPin, Pin dcPin) noexcept
	: ScreenDriver(width, height), dcPin(dcPin)
{
	spiDevice.csPin = csPin;
	// Set CS (chip select) to be active low
	spiDevice.csPolarity = false;
	// Data is sampled on the rising edge of the clock pulse and shifted out on the falling edge of the clock pulse
	spiDevice.spiMode = 0;
	// The LcdSpiClockFrequency is now defined in the Pins_xxxxx.h file for the configuration being built
	spiDevice.clockFrequency = LcdSpiClockFrequency;
#ifdef __LPC17xx__
	spiDevice.sspChannel = LcdSpiChannel;
#endif
}

void UC1701::Init() noexcept
{
	// Set DC/A0 pin to be an output with initial LOW state (command: 0, data: 1)
	IoPort::SetPinMode(dcPin, OUTPUT_LOW);

	sspi_master_init(&spiDevice, 8);

	numContinuationBytesLeft = 0;
	startRow = displayHeight;
	startCol = displayWidth;
	endRow = endCol = nextFlushRow = 0;

	{
		MutexLocker lock(Tasks::GetSpiMutex());
		sspi_master_setup_device(&spiDevice);
		delayMicroseconds(1);
		sspi_select_device(&spiDevice);
		delayMicroseconds(1);

		//TODO: make these separate methods?
		// 11100010 System reset
		sendLcdCommand(LcdSystemReset);
		sendLcdCommand(LcdDisplayOff);
		// 01000000 Set scroll line to 0 (6-bit value)
		sendLcdCommand(0x40);
		// 10100000 Set SEG (column) direction to MX (mirror = 0)
		sendLcdCommand(0xA0);
		// 11001000 Set COM (row) direction not to MY (mirror = 0)
		sendLcdCommand(0xC8);
		// 10100110 Set inverse display to false
		sendLcdCommand(0xA6);
		// 10100010 Set LCD bias ratio BR=0 (1/9th at 1/65 duty)
		sendLcdCommand(0xA2);
		// 00101111 Set power control to enable XV0, V0 and VG charge pumps
		sendLcdCommand(0x2F);
		// 11111000 Set booster ratio (2-byte command) to 4x
		sendLcdCommand(0xF8);
		sendLcdCommand(0x00);
		// 00100011 Set Vlcd resistor ratio 1+Rb/Ra to 6.5 for the voltage regulator (contrast)
		sendLcdCommand(0x23);
		// 10000001 Set electronic volume (2-byte command) 6-bit contrast value
		sendLcdCommand(0x81);
		sendLcdCommand(0x27);
		// 10101100 Set static indicator off
		sendLcdCommand(0xAC);
		// 10101110 Set display enable to off
		sendLcdCommand(0xAE);
		// 10100101 Set all pixel on (enters power saving mode)
		sendLcdCommand(0xA5);

		delayMicroseconds(1);
		sspi_deselect_device(&spiDevice);
	}

	Clear();
	FlushAll();

	{
		MutexLocker lock(Tasks::GetSpiMutex());
		sspi_master_setup_device(&spiDevice);
		delayMicroseconds(1);
		sspi_select_device(&spiDevice);
		delayMicroseconds(1);

		// Set display enable to on
		sendLcdCommand(0xAF);

		delayMicroseconds(1);
		sspi_deselect_device(&spiDevice);
	}

	currentFontNumber = 0;
}

// Adjust the serial interface clock frequency
void UC1701::SetBusClockFrequency(uint32_t freq) noexcept
{
	spiDevice.clockFrequency = freq;
}

// Flush all of the dirty part of the image to the lcd. Only called during startup and shutdown.
//TODO: call this just Flush()?
void UC1701::FlushAll() noexcept
{
	while (Flush())
	{
		delayMicroseconds(LcdFlushRowDelayMicros);
	}
}

// Flush some of the dirty part of the image to the LCD, returning true if there is more to do
//TODO: call this FlushBlock() or FlushRow() or FlushTile()?
//TODO: move this method to the DriverBase class, and implement callback UpdateTile(c, r)?
bool UC1701::Flush() noexcept
{
	// See if there is anything to flush
	if (endCol > startCol && endRow > startRow)
	{
		// Decide which row to flush next
		if (nextFlushRow < startRow || nextFlushRow >= endRow)
		{
			nextFlushRow = startRow;	// start from the beginning
		}

		if (nextFlushRow == startRow)	// if we are starting from the beginning
		{
			++startRow;					// flag this row as flushed because it will be soon
		}

		// Flush that row
		{
			uint8_t startColNum = startCol/16;
			const uint8_t endColNum = (endCol + 15)/16;
//			debugPrintf("flush %u %u %u\n", nextFlushRow, startColNum, endColNum);

			MutexLocker lock(Tasks::GetSpiMutex());
			sspi_master_setup_device(&spiDevice);
			sspi_select_device(&spiDevice);
			delayMicroseconds(1);

			setGraphicsAddress(nextFlushRow, startColNum);
			uint8_t *ptr = imageBuffer + (((displayWidth/8) * nextFlushRow) + (2 * startColNum));
			while (startColNum < endColNum)
			{
				sendLcdData(*ptr++);
				sendLcdData(*ptr++);
				++startColNum;
				dataDelay();
			}
			sspi_deselect_device(&spiDevice);
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


inline void UC1701::commandDelay() noexcept
{
	delayMicroseconds(LcdCommandDelayMicros);
}

inline void UC1701::dataDelay() noexcept
{
	delayMicroseconds(LcdDataDelayMicros);
}

// Set the address to write to.
// The display memory is organized in 8+1 pages (of horizontal rows) and 0-131 columns
void UC1701::setGraphicsAddress(unsigned int r, unsigned int c) noexcept
{
	// 1011#### Set Page Address
	sendLcdCommand(0xB0 | ((r >> 3) & 0x0F));
	// 0000#### Set Column Address LSB
	sendLcdCommand(0x00 | (c & 0x0F));
	// 0001#### Set Column Address MSB
	sendLcdCommand(0x10 | ((c >> 4) & 0x0F));
	commandDelay();
}

// Send a command to the LCD. The SPI mutex is already owned
void UC1701::sendLcdCommand(uint8_t command) noexcept
{
	sendLcd(command);
}

// Send a data byte to the LCD. The SPI mutex is already owned
void UC1701::sendLcdData(uint8_t data) noexcept
{
	digitalWrite(dcPin, true);
	sendLcd(data);
	digitalWrite(dcPin, false);
}

// Send a command to the lcd.
// The SPI mutex is already owned
void UC1701::sendLcd(uint8_t data) noexcept
{
	uint8_t buffer[1];
	buffer[0] = data;
	sspi_transceive_packet(buffer, nullptr, 1);
}

#endif
