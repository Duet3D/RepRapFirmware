#ifndef SRC_DISPLAY_LCD_LCD_H
#define SRC_DISPLAY_LCD_LCD_H

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include <Print.h>
#include "Fonts/LcdFont.h"
#include <Hardware/SharedSpi/SharedSpiClient.h>
#include <General/SafeVsnprintf.h>

// Enumeration for specifying drawing modes
enum class PixelMode : uint8_t
{
	PixelClear = 0,    // clear the pixel(s)
	PixelSet = 1,      // set the pixel(s)
	PixelFlip = 2      // invert the pixel(s)
};

typedef uint8_t PixelNumber;

// Class for driving 128x64 graphical LCD fitted with ST7920 controller
// This drives the GLCD in serial mode so that it needs just 2 pins.

// Derive the LCD class from the Print class so that we can print stuff to it in alpha mode
class Lcd
{
public:
	// Construct a GLCD driver.
	Lcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts, SpiMode mode) noexcept;
	virtual ~Lcd();

	// Flush just some data, returning true if this needs to be called again
	virtual bool FlushSome() noexcept = 0;

	// Get the display type
	virtual const char *GetDisplayTypeName() const noexcept = 0;

	// Get the SPI frequency
	uint32_t GetSpiFrequency() const noexcept { return device.GetFrequency(); }

	// Initialize the display
	void Init(Pin p_csPin, Pin p_a0Pin, bool csPolarity, uint32_t freq, uint8_t p_contrastRatio, uint8_t p_resistorRatio) noexcept;

	// Select the font to use for subsequent calls to write() in graphics mode
	void SetFont(size_t newFont) noexcept;

	const PixelNumber GetNumRows() const noexcept { return numRows; }
	const PixelNumber GetNumCols() const noexcept { return numCols; }

	// Write a single character in the current font. Called by the 'print' functions.
	//  c = character to write
	// Returns the number of characters written (1 if we wrote it, 0 otherwise)
	size_t write(uint8_t c) noexcept;		// write a character

	// Write a space
	void WriteSpaces(PixelNumber numPixels) noexcept;

	// Return the number of fonts
	size_t GetNumFonts() const noexcept { return numFonts; }

	// Get the current font height
	PixelNumber GetFontHeight() const noexcept;

	// Get the height of a specified font
	PixelNumber GetFontHeight(size_t fontNumber) const noexcept;

	// Select normal or inverted text (only works in graphics mode)
	void TextInvert(bool b) noexcept;

	// Clear part of the display and select non-inverted text.
	void Clear(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept;

	// Clear the whole display and select non-inverted text.
	void Clear() noexcept
	{
		Clear(0, 0, numRows, numCols);
	}

	// Set the cursor position
	//  r = row, the number of pixels from the top of the display to the top of the character.
	//  c = column, is the number of pixels from the left hand edge of the display and the left hand edge of the character.
	void SetCursor(PixelNumber r, PixelNumber c) noexcept;

	// Get the cursor row. Useful after we have written some text.
	PixelNumber GetRow() const noexcept { return row; }

	// Get the cursor column. Useful after we have written some text.
	PixelNumber GetColumn() const noexcept { return column; }

	// Set the left margin. This is where the cursor goes to when we print newline.
	void SetLeftMargin(PixelNumber c) noexcept;

	// Set the right margin. In graphics mode, anything written will be truncated at the right margin. Defaults to the right hand edge of the display.
	void SetRightMargin(PixelNumber r) noexcept;

	// Clear a rectangle from the current position to the right margin (graphics mode only). The height of the rectangle is the height of the current font.
	void ClearToMargin() noexcept;

	// Flush the display buffer to the display. Data will not be committed to the display until this is called.
	void FlushAll() noexcept;

	// Set, clear or invert a pixel
	//  x = x-coordinate of the pixel, measured from left hand edge of the display
	//  y = y-coordinate of the pixel, measured down from the top of the display
	//  mode = whether we want to set, clear or invert the pixel
	void SetPixel(PixelNumber y, PixelNumber x, PixelMode mode) noexcept;

	// Read a pixel. Returns true if the pixel is set, false if it is clear.
	//  x = x-coordinate of the pixel, measured from left hand edge of the display
	//  y = y-coordinate of the pixel, measured down from the top of the display
	bool ReadPixel(PixelNumber y, PixelNumber x) const noexcept;

	// Draw a line.
	//  x0 = x-coordinate of one end of the line, measured from left hand edge of the display
	//  y0 = y-coordinate of one end of the line, measured down from the top of the display
	//  x1, y1 = coordinates of the other end od the line
	//  mode = whether we want to set, clear or invert each pixel
	void Line(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right, PixelMode mode) noexcept;

	// Draw a circle
	//  x0 = x-coordinate of the centre, measured from left hand edge of the display
	//  y0 = y-coordinate of the centre, measured down from the top of the display
	//  radius = radius of the circle in pixels
	//  mode = whether we want to set, clear or invert each pixel
	void Circle(PixelNumber p_row, PixelNumber col, PixelNumber radius, PixelMode mode) noexcept;

	// Draw a bitmap
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display. Currently, must be a multiple of 8.
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels. Currently, must be a multiple of 8.
	//  rows = height of bitmap in pixels
	//  data = bitmap image, must be ((width/8) * rows) bytes long
	void BitmapImage(PixelNumber top, PixelNumber left, PixelNumber height, PixelNumber width, const uint8_t data[]) noexcept;

	// Draw a bitmap row
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels
	//  data = bitmap image, must be ((width + 7)/8) bytes long
	void BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept;

	// printf to LCD
	int printf(const char* fmt, ...) noexcept;

protected:
	virtual void HardwareInit() noexcept = 0;

	size_t writeNative(uint16_t c) noexcept;		// write a decoded character
	void SetDirty(PixelNumber r, PixelNumber c) noexcept;
	void SetRectDirty(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept;

	size_t imageSize;
	uint8_t *image;									// image buffer
	SharedSpiClient device;
	const PixelNumber numRows, numCols;
	Pin csPin;
	Pin a0Pin;
	uint8_t contrastRatio;
	uint8_t resistorRatio;
	PixelNumber startRow, startCol, endRow, endCol;	// coordinates of the dirty rectangle
	PixelNumber nextFlushRow;						// which row we need to flush next

private:
	const LcdFont * const *fonts;
	const size_t numFonts;
	size_t currentFontNumber;						// index of the current font
	uint32_t charVal;
	uint16_t lastCharColData;						// data for the last non-space column, used for kerning
	uint8_t numContinuationBytesLeft;
	PixelNumber row, column;
	PixelNumber leftMargin, rightMargin;
	bool textInverted;
	bool justSetCursor;
};

#endif

#endif	// SRC_DISPLAY_LCD_LCD_H
