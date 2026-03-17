/*
 * TFTLcd.cpp
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#include "TFTLcd.h"

#if SUPPORT_ILI9488_LCD

#if USE_FONT_CHIP
TFTLcd::TFTLcd(PixelNumber nr, PixelNumber nc, Pin fontCsPin, SpiMode mode, uint8_t sercomNum, uint32_t dataInPad, uint32_t dataOutPad) noexcept
	: Lcd(nr, nc, fontCsPin),
#else
TFTLcd::TFTLcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts, SpiMode mode, uint8_t sercomNum, uint32_t dataInPad, uint32_t dataOutPad) noexcept
	: Lcd(nr, nc, fnts, nFonts),
#endif
	  spiDev(sercomNum, dataInPad, dataOutPad),
	  fgColour(Colours::White), bgColour(Colours::Blue),
	  spiMode(mode)
{
}

TFTLcd::~TFTLcd()
{
	SetPinMode(csPin, INPUT_PULLUP);
}

// Get the SPI frequency
uint32_t TFTLcd::GetSpiFrequency() const noexcept
{
	return spiFrequency;
}

// Initialize the display
void TFTLcd::Init(Pin p_csPin, Pin p_a0Pin, bool csPolarity, uint32_t freq, uint8_t p_contrastRatio, uint8_t p_resistorRatio) noexcept
{
	csPin = p_csPin;
	csPol = csPolarity;
	spiFrequency = freq;
	spiDev.SetClockFrequencyAndMode(freq, spiMode, true);				// note we currently always use 9-bit SPI
	SetPinMode(csPin, (csPolarity) ? OUTPUT_LOW : OUTPUT_HIGH);
	HardwareInit();
}

#endif

// End
