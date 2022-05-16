/*
 * TFTLcd.h
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_LCD_TFTLCD_H_
#define SRC_DISPLAY_LCD_TFTLCD_H_

#include "Lcd.h"
#include <Hardware/Spi/SpiDevice.h>

#if SUPPORT_ILI9488_LCD

// This class represents a TFT LCD that uses an exclusive SPI channel to send data to the screen
class TFTLcd : public Lcd
{
public:
	TFTLcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts, SpiMode mode, uint8_t sercomNum) noexcept;
	virtual ~TFTLcd();

	// Get the SPI frequency
	uint32_t GetSpiFrequency() const noexcept override;

	// Initialize the display
	void Init(Pin p_csPin, Pin p_a0Pin, bool csPolarity, uint32_t freq, uint8_t p_contrastRatio, uint8_t p_resistorRatio) noexcept override;

	// Flush just some data, returning true if this needs to be called again
	bool FlushSome() noexcept override { return false; }

	// Set the foreground colour. Does nothing on monochrome displays.
	void SetForegroundColour(Colour col) noexcept override final { fgColour = col; }

	// Set the background colour. Does nothing on monochrome displays.
	void SetBackgroundColour(Colour col) noexcept override final { bgColour = col; }

protected:
	virtual void HardwareInit() noexcept = 0;

	SpiDevice spiDev;
	Colour fgColour, bgColour;
	Pin csPin = NoPin;
	bool csPol;

private:
	uint32_t spiFrequency = 0;
	SpiMode spiMode;
};

#endif

#endif /* SRC_DISPLAY_LCD_TFTLCD_H_ */
