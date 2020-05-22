/*
 * Driver.h
 *
 *  Created on: 22 mei 2020
 *      Author: Martijn Schiedon
 */

#ifndef SRC_DISPLAY_DRIVER_H_
#define SRC_DISPLAY_DRIVER_H_

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include "Print.h"
#include "Fonts/Fonts.h"

// Enumeration for specifying drawing modes
enum class PixelMode : uint8_t
{
	PixelClear = 0,    // clear the pixel(s)
	PixelSet = 1,      // set the pixel(s)
	PixelFlip = 2      // invert the pixel(s)
};

typedef uint8_t PixelNumber;
//const PixelNumber NumRows = 64, NumCols = 128;

// Derive this class from the Print class so that we can print stuff to it in alpha mode
class Driver : public Print
{
public:
	// Construct a GLCD driver.
	Driver(PixelNumber width, PixelNumber height) noexcept;

	constexpr PixelNumber GetNumRows() const noexcept { return displayHeight; }
	constexpr PixelNumber GetNumCols() const noexcept { return displayWidth; }

	// Initialize the display. Call this in setup(). Also call setFont to select initial text font.
	virtual void Init() noexcept = 0;
	// Set the SPI clock frequency
	virtual void SetSpiClockFrequency(uint32_t freq) noexcept = 0;
	// Flush the display buffer to the display. Data will not be committed to the display until this is called.
	virtual void FlushAll() noexcept = 0;
	// Flush just some data, returning true if this needs to be called again
	virtual bool FlushSome() noexcept = 0;

	// Write a single character in the current font. Called by the 'print' functions.
	//  c = character to write
	// Returns the number of characters written (1 if we wrote it, 0 otherwise)
	virtual size_t write(uint8_t c) noexcept;		// write a character

	// Write a space
	void WriteSpaces(PixelNumber numPixels) noexcept;

	// Maybe we just want to draw on the screen without text, so this is removed from the constructor
	void SetFonts(const LcdFont * const fnts[], size_t nFonts) noexcept;

	// Select the font to use for subsequent calls to write() in graphics mode
	void SelectFont(size_t newFont) noexcept;

	// Return the number of fonts
	size_t GetNumFonts() const noexcept { return numFonts; }

	// Get the current font height
	PixelNumber GetFontHeight() const noexcept;

	// Get the height of a specified font
	PixelNumber GetFontHeight(size_t fontNumber) const noexcept;

	// Select normal or inverted text (only works in graphics mode)
	void TextInvert(bool b) noexcept;

	// Clear the display and select non-inverted text.
	void Clear() noexcept;
	void Clear(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept;

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

	// Set, clear or invert a pixel
	//  x = x-coordinate of the pixel, measured from left hand edge of the display
	//  y = y-coordinate of the pixel, measured down from the top of the display
	//  mode = whether we want to set, clear or invert the pixel
	void SetPixel(PixelNumber y, PixelNumber x, PixelMode mode) noexcept;

	void SetDirty(PixelNumber r, PixelNumber c) noexcept;

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
	void Circle(PixelNumber row, PixelNumber col, PixelNumber radius, PixelMode mode) noexcept;

	// Draw a bitmap
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display. Currently, must be a multiple of 8.
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels. Currently, must be a multiple of 8.
	//  rows = height of bitmap in pixels
	//  data = bitmap image, must be ((width/8) * rows) bytes long
	void Bitmap(PixelNumber top, PixelNumber left, PixelNumber height, PixelNumber width, const uint8_t data[]) noexcept;

	// Draw a bitmap row
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels
	//  data = bitmap image, must be ((width + 7)/8) bytes long
	void BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept;

protected:
	PixelNumber displayWidth, displayHeight;
	const LcdFont * const *fonts;
	size_t numFonts;
	size_t currentFontNumber;						// index of the current font
	uint32_t charVal;
	uint16_t lastCharColData;						// data for the last non-space column, used for kerning
	uint8_t numContinuationBytesLeft;
	PixelNumber row, column;
	PixelNumber startRow, startCol, endRow, endCol;	// coordinates of the dirty rectangle
	PixelNumber nextFlushRow;						// which row we need to flush next
	PixelNumber leftMargin, rightMargin;
	//uint8_t image[(NumRows * NumCols)/8];			// image buffer, 1K in size
	uint8_t* image;			// image buffer
	bool textInverted;
	bool justSetCursor;

	size_t writeNative(uint16_t c) noexcept;		// write a decoded character
};

#endif

#endif /* SRC_DISPLAY_DRIVER_H_ */
