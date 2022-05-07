#ifndef LCD7920_H
#define LCD7920_H

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include <Display/Lcd/MonoLcd.h>

// Class for driving 128x64 graphical LCD fitted with ST7920 controller
// This drives the GLCD in serial mode so that it needs just 2 pins.

// Derive the LCD class from the Print class so that we can print stuff to it in alpha mode
class Lcd7920 : public MonoLcd
{
public:
	// Construct a GLCD driver.
	Lcd7920(const LcdFont * const fnts[], size_t nFonts) noexcept;

	// Flush just some data, returning true if this needs to be called again
	bool FlushSome() noexcept override;

	// Get the display type
	const char *_ecv_array GetDisplayTypeName() const noexcept override;

protected:
	void HardwareInit() noexcept override;

private:
	void CommandDelay() noexcept;
	void DataDelay() noexcept;
	void SendLcdCommand(uint8_t byteToSend) noexcept;
	void SendLcdData(uint8_t byteToSend) noexcept;
	void SetGraphicsAddress(unsigned int r, unsigned int c) noexcept;
};

#endif

#endif
