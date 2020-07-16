/*
 * ST7565.cpp
 *
 *  Created on : 2020-05-14
 *      Author : Martijn Schiedon
 */

// Driver for 128x64 graphical LCD with ST7565, ST7567, UC1701, NT7534 or compatible controller

//TODO: initially used the UC1701 datasheet, but it has poor information on timing,
//      use the ST7565 datasheet and validate if timings are officially correct going forward

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

#include <Display/Lcd/ST7565/ST7565.h>

#if SUPPORT_12864_LCD

#include "Pins.h"
#include <Hardware/SharedSpi/SharedSpiDevice.h>
#include "Hardware/IoPorts.h"

ST7565::ST7565(PixelNumber width, PixelNumber height, Pin csPin, Pin dcPin, bool csPolarity, Pin gatePin) noexcept
	: DisplayDriver(width, height),
	  spiDevice(SharedSpiDevice::GetMainSharedSpiDevice(), LcdSpiClockFrequency, SpiMode::mode0, NoPin, true),
	  dcPin(dcPin),
	  gatePin(gatePin)
{
	// The pin to use for CS
	spiDevice.SetCsPin(csPin);
	// CS (chip select) should be active low for the ST7565
	// But this causes the MOSI and SCK to be permanently low because they are AND-gated with CS, so therefore
	// an alternative CS pin can be used, and the regular CS pin (can be left disconnected) is set high just before chip select.
	spiDevice.SetCsPolarity(csPolarity);
	// Data is sampled on the rising edge of the clock pulse and shifted out on the falling edge of the clock pulse
	//spiDevice.SetSpiMode = 0;
	// The LcdSpiClockFrequency is now defined in the Pins_xxxxx.h file for the configuration being built
	spiDevice.SetClockFrequency(LcdSpiClockFrequency);
#ifdef __LPC17xx__
	spiDevice.sspChannel = LcdSpiChannel;
#endif
}

void ST7565::OnInitialize() noexcept
{
	// Set DC/A0 pin to be an output with initial LOW state (command: 0, data: 1)
	setPinMode(dcPin, OUTPUT_LOW);
	// If an extra gate pin is defined, set that to LOW state initially as well
	setPinMode(gatePin, OUTPUT_LOW);

	// Post-reset wait of 6ms
	delay(6);

	{
		selectDevice();

		//TODO: make these separate methods?
		// 11100010 System reset
		sendCommand(SystemReset);
		sendCommand(DisplayOff);
		// 01000000 Set scroll line to 0 (6-bit value)
		sendCommand(0x40);
		// 10100000 Set SEG (column) direction to MX (mirror = 0)
		sendCommand(0xA0);
		// 11001000 Set COM (row) direction not to MY (mirror = 0)
		sendCommand(0xC8);
		// 10100110 Set inverse display to false
		sendCommand(0xA6);
		// 10100010 Set LCD bias ratio BR=0 (1/9th at 1/65 duty)
		sendCommand(0xA2);
		// 00101111 Set power control to enable XV0, V0 and VG charge pumps
		sendCommand(0x2F);
		// 11111000 Set booster ratio (2-byte command) to 4x
		sendCommand(0xF8);
		sendCommand(0x00);
		// 00100011 Set Vlcd resistor ratio 1+Rb/Ra to 6.5 for the voltage regulator (contrast)
		sendCommand(0x23);
		// 10000001 Set electronic volume (2-byte command) 6-bit contrast value
		sendCommand(0x81);
		sendArg(0x27);
		// 10101100 Set static indicator off
		sendCommand(0xAC);

		// Enter sleep mode
		sendCommand(DisplayOff);
		sendCommand(PixelOn);

		deselectDevice();
	}
}

void ST7565::OnEnable() noexcept
{
	{
		selectDevice();

		// Exit sleep mode, display on
		sendCommand(PixelOff);
		sendCommand(DisplayOn);

		deselectDevice();
	}
}

// Adjust the serial interface clock frequency
void ST7565::SetBusClockFrequency(uint32_t freq) noexcept
{
	spiDevice.SetClockFrequency(freq);
}

// Flush the specified row
void ST7565::OnFlushRow(PixelNumber startRow, PixelNumber startColumn, PixelNumber endRow, PixelNumber endColumn) noexcept
{
	{
		selectDevice();

		// Set the GDRAM address to send the bytes to
		setGraphicsAddress(startRow, startColumn);

		startDataTransaction();

#ifdef ALTERNATIVE_ST7565_FLUSHROW
		// Send tiles of 1x8 for the desired (quantized) width of the dirty rectangle
		for(int x = startColumn; x < endColumn; x += GetTileWidth())
		{
			uint8_t data = 0;

			// Gather the bits for a vertical line of 8 pixels (LSB is the top pixel)
			for(uint8_t i = 0; i < 8; i++)
			{
				if(ReadPixel(x, startRow + i)) {
					data |= (1u << i);
				}
			}

			sendData(data);
		}
#else
		uint8_t tile[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

		// Send tiles of 8x8 for the desired (quantized) width of the dirty rectangle
		for(int x = startColumn; x < endColumn; x += GetTileWidth())
		{

			uint8_t* ptr = displayBuffer + ((startRow * (displayWidth >> 3)) + (x >> 3));

			// Fill the tile buffer, so we're efficient and don't retrieve the same bytes 8x
			for(int i = 0; i < 8; i++)
			{
				tile[i] = *ptr;
				ptr += (displayWidth / 8);
			}

			// Send the tile, as vertical rows made out of the bits 7, then a row of bits 6, and so on.
			for(int i = 7; i >= 0; i--)
			{
				sendData(transformTile(tile, i));
			}
		}
#endif

		endDataTransaction();

		// End update
		deselectDevice();
	}
}

#ifndef ALTERNATIVE_ST7565_FLUSHROW

// Array of bytes is assumed to be 8 in size (square tile of 8x8 bits)
//TODO: define Tile data type?
uint8_t ST7565::transformTile(uint8_t data[8], PixelNumber c) noexcept
{
	return ((data[0] >> c) & 1)
         | ((data[1] >> c) & 1) << 1
		 | ((data[2] >> c) & 1) << 2
		 | ((data[3] >> c) & 1) << 3
		 | ((data[4] >> c) & 1) << 4
		 | ((data[5] >> c) & 1) << 5
		 | ((data[6] >> c) & 1) << 6
		 | ((data[7] >> c) & 1) << 7;
}

#endif

// Test to flush entire display each time something is dirty, no matter how small
// If this function is not used, the linker ignores it and it will not contribute to the firmware size
void ST7565::flushEntireBuffer() noexcept
{
	{
		selectDevice();

		uint8_t tile[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

		// Send 8 rows of pixels in each transaction
		for(int y = 0; y < displayHeight; y += GetTileHeight())
		{
			// Set where the bytes go
			setGraphicsAddress(y, 0);

			startDataTransaction();

			// Send tiles of 8x8 for the entire display width
			for(int x = 0; x < displayWidth; x += GetTileWidth())
			{
				// Fill the tile buffer
				for(int i = 0; i < 8; i++)
				{
					tile[i] = *(displayBuffer + ((y + i) * (displayWidth / 8)) + (x / 8));
				}
				// Send the tile, as vertical rows made out of the bits 7, then a row of bits 6, and so on.
				for(int i = 7; i >= 0; i--)
				{
					sendData(transformTile(tile, i));
				}
			}

			endDataTransaction();
		}

		// End update
		deselectDevice();
	}

	// Reset dirty rectangle and row tracking
	SetFlushDone();
}

// Simple wrapper for IoPort::SetPinMode, that does nothing for a NoPin
// (since I'm not sure if the lower level functions also do this).
void ST7565::setPinMode(Pin pin, PinMode mode) noexcept
{
	if(pin != NoPin) { IoPort::SetPinMode(pin, mode); }
}

// Simple wrapper for IoPort::WriteDigital, that does nothing for a NoPin
// (since I'm not sure if the lower level functions also do this).
void ST7565::writeDigital(Pin pin, bool high) noexcept
{
	if(pin != NoPin) { IoPort::WriteDigital(pin, high); }
}

void ST7565::selectDevice() noexcept
{
	//TODO: can/should the "MutexLocker lock(Tasks::GetSpiMutex());" be moved here as well?
	writeDigital(gatePin, true);
	delayMicroseconds(1);
	spiDevice.Select();
	delayMicroseconds(1);
}

void ST7565::deselectDevice() noexcept
{
	delayMicroseconds(1);
	spiDevice.Deselect();
	writeDigital(gatePin, false);
}

// Set the address to write to.
// The display memory is organized in 8+1 pages (of horizontal rows) and 0-131 columns
void ST7565::setGraphicsAddress(unsigned int r, unsigned int c) noexcept
{
	// 0001#### Set Column Address MSB
	sendCommand(0x10 | ((c >> 4) & 0b00001111));
	// 0000#### Set Column Address LSB
	sendCommand(0x00 | (c & 0b00001111));
	// 1011#### Set Page Address
	sendCommand(0xB0 | ((r >> 3) & 0b00001111));

	commandDelay();
}

// Send a command to the LCD. The SPI mutex is already owned
void ST7565::sendCommand(uint8_t command) noexcept
{
	uint8_t buffer[1];
	buffer[0] = command;

	spiDevice.TransceivePacket(buffer, nullptr, 1);
}

// Send a data byte to the LCD. The SPI mutex is already owned
void ST7565::sendArg(uint8_t arg) noexcept
{
	uint8_t buffer[1];
	buffer[0] = arg;

	spiDevice.TransceivePacket(buffer, nullptr, 1);
}

void ST7565::startDataTransaction() noexcept
{
	digitalWrite(dcPin, true);
}

// Send a data byte to the LCD. The SPI mutex is already owned
void ST7565::sendData(uint8_t data) noexcept
{
	uint8_t buffer[1];
	buffer[0] = data;
	spiDevice.TransceivePacket(buffer, nullptr, 1);
}

void ST7565::endDataTransaction() noexcept
{
	digitalWrite(dcPin, false);
}

inline void ST7565::commandDelay() noexcept
{
	delayMicroseconds(CommandDelayMicros);
}

inline void ST7565::dataDelay() noexcept
{
	delayMicroseconds(DataDelayMicros);
}

#endif
