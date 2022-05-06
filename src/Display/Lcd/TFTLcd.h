/*
 * TFTLcd.h
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_LCD_TFTLCD_H_
#define SRC_DISPLAY_LCD_TFTLCD_H_

#include "Lcd.h"

#if SUPPORT_ILI9488_LCD

class TFTLcd : public Lcd
{
public:
	TFTLcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts) noexcept;
	virtual ~TFTLcd();

	// Get the SPI frequency
	uint32_t GetSpiFrequency() const noexcept override;

	// Initialize the display
	void Init(Pin p_csPin, Pin p_a0Pin, bool csPolarity, uint32_t freq, uint8_t p_contrastRatio, uint8_t p_resistorRatio) noexcept override;

	// Clear part of the display and select non-inverted text.
	void Clear(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept override;

	// Set, clear or invert a pixel
	//  x = x-coordinate of the pixel, measured from left hand edge of the display
	//  y = y-coordinate of the pixel, measured down from the top of the display
	//  mode = whether we want to set or clear the pixel
	void SetPixel(PixelNumber y, PixelNumber x, bool mode) noexcept override;

	// Draw a bitmap
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display. Currently, must be a multiple of 8.
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels. Currently, must be a multiple of 8.
	//  rows = height of bitmap in pixels
	//  data = bitmap image, must be ((width/8) * rows) bytes long
	void BitmapImage(PixelNumber top, PixelNumber left, PixelNumber height, PixelNumber width, const uint8_t data[]) noexcept override;

	// Draw a bitmap row
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels
	//  data = bitmap image, must be ((width + 7)/8) bytes long
	void BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept override;

protected:
};

#endif

#endif /* SRC_DISPLAY_LCD_TFTLCD_H_ */
