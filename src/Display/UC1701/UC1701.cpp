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

UC1701::UC1701(PixelNumber width, PixelNumber height, Pin csPin, Pin dcPin) noexcept
	: DisplayDriver(width, height), dcPin(dcPin)
{
	spiDevice.csPin = csPin;
	// Set CS (chip select) to be active low
	//spiDevice.csPolarity = false;
	// Needs to be true because CLK is gated with CS
	spiDevice.csPolarity = true;
	// Data is sampled on the rising edge of the clock pulse and shifted out on the falling edge of the clock pulse
	spiDevice.spiMode = 0;
	// The LcdSpiClockFrequency is now defined in the Pins_xxxxx.h file for the configuration being built
	spiDevice.clockFrequency = LcdSpiClockFrequency;
#ifdef __LPC17xx__
	spiDevice.sspChannel = LcdSpiChannel;
#endif
}

void UC1701::OnInitialize() noexcept
{
	// Set DC/A0 pin to be an output with initial LOW state (command: 0, data: 1)
	IoPort::SetPinMode(dcPin, OUTPUT_LOW);

	// Post-reset wait
	delay(6);

	sspi_master_init(&spiDevice, 8);

	//numContinuationBytesLeft = 0;
	//startRow = displayHeight;
	//startCol = displayWidth;
	//endRow = endCol = nextFlushRow = 0;

	{
		MutexLocker lock(Tasks::GetSpiMutex());
		sspi_master_setup_device(&spiDevice);
		delayMicroseconds(1);
		sspi_select_device(&spiDevice);
		delayMicroseconds(1);

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

		delayMicroseconds(1);
		sspi_deselect_device(&spiDevice);
	}
}

void UC1701::OnEnable() noexcept
{
	//Clear();
	//FlushAll();

	{
		MutexLocker lock(Tasks::GetSpiMutex());
		sspi_master_setup_device(&spiDevice);
		delayMicroseconds(1);
		sspi_select_device(&spiDevice);
		delayMicroseconds(1);

		// Exit sleep mode, display on
		sendCommand(PixelOff);
		sendCommand(DisplayOn);

		delayMicroseconds(1);
		sspi_deselect_device(&spiDevice);
	}

	//currentFontNumber = 0;
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
		delayMicroseconds(FlushRowDelayMicros);
	}
}

// Flush some of the dirty part of the image to the LCD, returning true if there is more to do
//TODO: call this FlushBlock() or FlushRow() or FlushTile()?
//TODO: move this method to the DriverBase class, and implement callback UpdateTile(c, r)?
//      perhaps, offer the method in the specific driver the dirty rectangle in OnStartFlush() to let it determine the start parameters
//      and OnFlush() to let it flush based on these parameters, and return false when done,
//      so that the Flush() in the DisplayDriver can submit a new dirty rectangle through OnStartFlush()
//      the dirty rectangle member variables can be made private then as well.
bool UC1701::Flush() noexcept
{
	// See if there is anything to flush, right and bottom need to be at least one pixel larger than left and top.
	if (dirtyRectRight > dirtyRectLeft && dirtyRectBottom > dirtyRectTop)
	{
		// Test to flush entire display each time something is dirty
		{
			MutexLocker lock(Tasks::GetSpiMutex());
			sspi_master_setup_device(&spiDevice);
			delayMicroseconds(1);
			sspi_select_device(&spiDevice);
			delayMicroseconds(1);

			uint8_t tile[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			// Send 8 rows of pixels in each transaction
			for(int y = 0; y < displayHeight; y += 8)
			{
				// Set where the bytes go
				setGraphicsAddress(y, 0);

				startSendData();

				// Send tiles of 8x8 for the entire display width
				for(int x = 0; x < displayWidth; x += 8)
				{
					// Fill the tile buffer
					for(int i = 0; i < 8; i++)
					{
						tile[i] = *(displayBuffer + ((y + i) * displayWidth / 8) + (x / 8));
					}
					// Send the tile, as vertical rows made out of the bits 7, then a row of bits 6, and so on.
					for(int i = 7; i >= 0; i--)
					{
						sendData(transformTile(tile, i));
					}
				}

				endSendData();
			}

			// End update
			delayMicroseconds(1);
			sspi_deselect_device(&spiDevice);
		}

		// Reset dirty rectangle and row tracking
		dirtyRectTop = displayHeight;
		dirtyRectLeft = displayWidth;
		dirtyRectRight = dirtyRectBottom = nextFlushRow = 0;

/*
		// The columns are set one pixel at a time
		uint8_t startColNum = dirtyRectLeft;
		const uint8_t endColNum = dirtyRectRight;
		// The rows are set 8 pixels at a time
		uint8_t startRowNum = dirtyRectTop/8;
		const uint8_t endRowNum = (dirtyRectBottom + 7)/8;

		// Decide which row to flush next
		if (nextFlushRow < startRowNum || nextFlushRow >= endRowNum)
		{
			nextFlushRow = startRowNum;       // start from the beginning
		}

		if (nextFlushRow == startRowNum)	  // if we are starting from the beginning
		{
			// Shrink the dirty rectangle pre-emptively (flag as flushed)
			dirtyRectTop = startRowNum + 8;
		}

		// Flush that row/tile
		{
//			debugPrintf("flush %u %u %u\n", nextFlushRow, startColNum, endColNum);

			MutexLocker lock(Tasks::GetSpiMutex());
			sspi_master_setup_device(&spiDevice);
			sspi_select_device(&spiDevice);
			delayMicroseconds(1);

			setGraphicsAddress(nextFlushRow, startColNum);
			uint8_t *ptr = displayBuffer + (((displayWidth/8) * nextFlushRow) + (2 * startColNum));
			while (startColNum < endColNum)
			{
				sendLcdData(*ptr++);
				sendLcdData(*ptr++);
				++startColNum;
				dataDelay();
			}
			sspi_deselect_device(&spiDevice);
		}

		if (dirtyRectTop < dirtyRectBottom)
		{
			nextFlushRow += 8;
			return true;
		}

		// Reset dirty rectangle and row tracking
		dirtyRectTop = displayHeight;
		dirtyRectLeft = displayWidth;
		dirtyRectRight = dirtyRectBottom = nextFlushRow = 0;
*/
	}
	return false;
}

// Array of bytes is assumed to be 8 in size (square tile of 8x8 bits)
//TODO: define Tile data type?
uint8_t UC1701::transformTile(uint8_t data[8], PixelNumber c) noexcept
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

// Set the address to write to.
// The display memory is organized in 8+1 pages (of horizontal rows) and 0-131 columns
void UC1701::setGraphicsAddress(unsigned int r, unsigned int c) noexcept
{
	// 0001#### Set Column Address MSB
	sendCommand(0x10 | ((c >> 4) & 0x0F));
	// 0000#### Set Column Address LSB
	sendCommand(0x00 | (c & 0x0F));
	// 1011#### Set Page Address
	sendCommand(0xB0 | ((r >> 3) & 0x0F));

	commandDelay();
}

// Send a command to the LCD. The SPI mutex is already owned
void UC1701::sendCommand(uint8_t command) noexcept
{
	uint8_t buffer[1];
	buffer[0] = command;

	sspi_transceive_packet(buffer, nullptr, 1);
}

// Send a data byte to the LCD. The SPI mutex is already owned
void UC1701::sendArg(uint8_t arg) noexcept
{
	uint8_t buffer[1];
	buffer[0] = arg;

	sspi_transceive_packet(buffer, nullptr, 1);
}

void UC1701::startSendData() noexcept
{
	digitalWrite(dcPin, true);
}

// Send a data byte to the LCD. The SPI mutex is already owned
void UC1701::sendData(uint8_t data) noexcept
{
	uint8_t buffer[1];
	buffer[0] = data;

	sspi_transceive_packet(buffer, nullptr, 1);
}

void UC1701::endSendData() noexcept
{
	digitalWrite(dcPin, false);
}

inline void UC1701::commandDelay() noexcept
{
	delayMicroseconds(CommandDelayMicros);
}

inline void UC1701::dataDelay() noexcept
{
	delayMicroseconds(DataDelayMicros);
}

#endif
