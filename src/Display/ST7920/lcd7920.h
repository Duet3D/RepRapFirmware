#ifndef LCD7920_H
#define LCD7920_H

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include <Display/Driver.h>
#include "SharedSpi.h"

class Lcd7920 : public Driver
{
public:
	// Construct a GLCD driver.
	Lcd7920(Pin csPin, const LcdFont * const fnts[], size_t nFonts) noexcept;

	// Initialize the display. Call this in setup(). Also call setFont to select initial text font.
	void Init() noexcept;
	// Set the SPI clock frequency
	void SetSpiClockFrequency(uint32_t freq) noexcept;
	// Flush the display buffer to the display. Data will not be committed to the display until this is called.
	void FlushAll() noexcept;
	// Flush just some data, returning true if this needs to be called again
	bool FlushSome() noexcept;

private:
	sspi_device device;
	void sendLcdCommand(uint8_t command) noexcept;
	void sendLcdData(uint8_t data) noexcept;
	void sendLcd(uint8_t data1, uint8_t data2) noexcept;
	void CommandDelay() noexcept;
	void DataDelay() noexcept;
	void setGraphicsAddress(unsigned int r, unsigned int c) noexcept;
};

#endif

#endif
