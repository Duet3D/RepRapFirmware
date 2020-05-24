/*
 * DisplayDriver.h
 *
 *  Created on  : 2018-01-22
 *      Author  : David Crocker
 *  Modified on : 2020-05-16
 *      Author  : Martijn Schiedon
 */

#ifndef SRC_DISPLAY_DISPLAYDRIVER_H_
#define SRC_DISPLAY_DISPLAYDRIVER_H_

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

//TODO: check if "east const" convention is something to use, check with David how he feels

// Derive this class from the Print class so that we can print stuff to it in alpha mode
class DisplayDriver : public Print
{
public:
	DisplayDriver(PixelNumber width, PixelNumber height) noexcept;
	~DisplayDriver();

	// TODO: rename these GetDisplayWidth/GetDisplayHeight?
	constexpr PixelNumber GetNumCols() const noexcept { return displayWidth; }
	constexpr PixelNumber GetNumRows() const noexcept { return displayHeight; }

	// Initialize the display.
	virtual void Init() noexcept;

	// Callback handler to initialize driver
	virtual void OnInitialize() noexcept = 0;
	// Callback handler to enable driver
	virtual void OnEnable() noexcept = 0;
	// Set the clock frequency of the interface (serial bus)
	virtual void SetBusClockFrequency(uint32_t freq) noexcept = 0;
	// Flush the entire display buffer to the display.
	virtual void FlushAll() noexcept = 0;
	// Flush units of dirty data to the display, returns true if there is more to flush
	virtual bool Flush() noexcept = 0;

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
	void ClearRect(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept;

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

	void SetPixelDirty(PixelNumber r, PixelNumber c) noexcept;

	// Read a pixel. Returns true if the pixel is set, false if it is clear.
	//  x = x-coordinate of the pixel, measured from left hand edge of the display
	//  y = y-coordinate of the pixel, measured down from the top of the display
	bool ReadPixel(PixelNumber y, PixelNumber x) const noexcept;

	// Draw a line.
	//  x0 = x-coordinate of one end of the line, measured from left hand edge of the display
	//  y0 = y-coordinate of one end of the line, measured down from the top of the display
	//  x1, y1 = coordinates of the other end od the line
	//  mode = whether we want to set, clear or invert each pixel
	void DrawLine(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right, PixelMode mode) noexcept;

	// Draw a circle
	//  x0 = x-coordinate of the centre, measured from left hand edge of the display
	//  y0 = y-coordinate of the centre, measured down from the top of the display
	//  radius = radius of the circle in pixels
	//  mode = whether we want to set, clear or invert each pixel
	void DrawCircle(PixelNumber row, PixelNumber col, PixelNumber radius, PixelMode mode) noexcept;

	// Draw a bitmap
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display. Currently, must be a multiple of 8.
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels. Currently, must be a multiple of 8.
	//  rows = height of bitmap in pixels
	//  data = bitmap image, must be ((width/8) * rows) bytes long
	void DrawBitmap(PixelNumber top, PixelNumber left, PixelNumber height, PixelNumber width, const uint8_t data[]) noexcept;

	// Draw a bitmap row
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels
	//  data = bitmap image, must be ((width + 7)/8) bytes long
	void DrawBitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept;

protected:
	const PixelNumber displayWidth, displayHeight;
	const LcdFont* const* fonts;
	size_t numFonts;
	size_t currentFontNumber;						                            // index of the current font
	uint32_t charVal;
	uint16_t lastCharColData;						                            // data for the last non-space column, used for kerning
	uint8_t numContinuationBytesLeft;
	PixelNumber row, column;                                                    // current cursor location
	PixelNumber dirtyRectTop, dirtyRectLeft, dirtyRectBottom, dirtyRectRight;	// coordinates of the dirty rectangle
	PixelNumber nextFlushRow;						                            // which row we need to flush next, owned by the Flush() method
	PixelNumber leftMargin, rightMargin;
	uint32_t displayBufferSize;
	uint8_t* displayBuffer;	        		                                    // screen/display buffer
	bool textInverted;
	bool justSetCursor;

	// Future idea
	//const PixelNumber flushTileWidth, flushTileHeight;

	size_t writeNative(uint16_t c) noexcept;		                            // write a decoded character
};

#endif

#endif /* SRC_DISPLAY_DISPLAYDRIVER_H_ */
