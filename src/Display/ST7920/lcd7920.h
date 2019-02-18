#ifndef LCD7920_H
#define LCD7920_H

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include "Print.h"
#include "SharedSpi.h"

// Enumeration for specifying drawing modes
enum class PixelMode : uint8_t
{
	PixelClear = 0,    // clear the pixel(s)
	PixelSet = 1,      // set the pixel(s)
	PixelFlip = 2      // invert the pixel(s)
};

// Struct for describing a font table, always held in PROGMEM
struct LcdFont
{
	const uint8_t *ptr;			// pointer to font table
	uint16_t startCharacter;	// Unicode code point of the first character in the font
	uint16_t endCharacter;		// Unicode code point of the last character in the font
	uint8_t height;				// row height in pixels - only this number of pixels will be fetched and drawn - maximum 16 in this version of the software
	uint8_t width;				// max character width in pixels (the font table contains this number of bytes or words per character, plus 1 for the active width)
	uint8_t numSpaces;			// number of space columns between characters before kerning
};

typedef uint8_t PixelNumber;
const PixelNumber NumRows = 64, NumCols = 128;

// Class for driving 128x64 graphical LCD fitted with ST7920 controller
// This drives the GLCD in serial mode so that it needs just 2 pins.

// Derive the LCD class from the Print class so that we can print stuff to it in alpha mode
class Lcd7920 : public Print
{
public:
	// Construct a GLCD driver.
	Lcd7920(Pin csPin, const LcdFont * const fnts[], size_t nFonts);

	constexpr PixelNumber GetNumRows() const { return NumRows; }
	constexpr PixelNumber GetNumCols() const { return NumCols; }

	// Set the SPI clock frequency
	void SetSpiClockFrequency(uint32_t freq);

	// Write a single character in the current font. Called by the 'print' functions.
	//  c = character to write
	// Returns the number of characters written (1 if we wrote it, 0 otherwise)
	virtual size_t write(uint8_t c);		// write a character

	// Write a space
	void WriteSpaces(PixelNumber numPixels);

	// Initialize the display. Call this in setup(). Also call setFont to select initial text font.
	void Init();

	// Return the number of fonts
	size_t GetNumFonts() const { return numFonts; }

	// Select the font to use for subsequent calls to write() in graphics mode
	void SetFont(size_t newFont);

	// Get the current font height
	PixelNumber GetFontHeight() const;

	// Get the height of a specified font
	PixelNumber GetFontHeight(size_t fontNumber) const;

	// Select normal or inverted text (only works in graphics mode)
	void TextInvert(bool b);

	// Clear the display and select non-inverted text.
	void Clear(PixelNumber top = 0, PixelNumber left = 0, PixelNumber bottom = NumRows, PixelNumber right = NumCols);

	// Set the cursor position
	//  r = row, the number of pixels from the top of the display to the top of the character.
	//  c = column, is the number of pixels from the left hand edge of the display and the left hand edge of the character.
	void SetCursor(PixelNumber r, PixelNumber c);

	// Get the cursor row. Useful after we have written some text.
	PixelNumber GetRow() const { return row; }

	// Get the cursor column. Useful after we have written some text.
	PixelNumber GetColumn() const { return column; }

	// Set the left margin. This is where the cursor goes to when we print newline.
	void SetLeftMargin(PixelNumber c);

	// Set the right margin. In graphics mode, anything written will be truncated at the right margin. Defaults to the right hand edge of the display.
	void SetRightMargin(PixelNumber r);

	// Clear a rectangle from the current position to the right margin (graphics mode only). The height of the rectangle is the height of the current font.
	void ClearToMargin();

	// Flush the display buffer to the display. Data will not be committed to the display until this is called.
	void FlushAll();

	// Flush just some data, returning true if this needs to be called again
	bool FlushSome();

	// Set, clear or invert a pixel
	//  x = x-coordinate of the pixel, measured from left hand edge of the display
	//  y = y-coordinate of the pixel, measured down from the top of the display
	//  mode = whether we want to set, clear or invert the pixel
	void SetPixel(PixelNumber y, PixelNumber x, PixelMode mode);

	// Read a pixel. Returns true if the pixel is set, false if it is clear.
	//  x = x-coordinate of the pixel, measured from left hand edge of the display
	//  y = y-coordinate of the pixel, measured down from the top of the display
	bool ReadPixel(PixelNumber y, PixelNumber x) const;

	// Draw a line.
	//  x0 = x-coordinate of one end of the line, measured from left hand edge of the display
	//  y0 = y-coordinate of one end of the line, measured down from the top of the display
	//  x1, y1 = coordinates of the other end od the line
	//  mode = whether we want to set, clear or invert each pixel
	void Line(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right, PixelMode mode);

	// Draw a circle
	//  x0 = x-coordinate of the centre, measured from left hand edge of the display
	//  y0 = y-coordinate of the centre, measured down from the top of the display
	//  radius = radius of the circle in pixels
	//  mode = whether we want to set, clear or invert each pixel
	void Circle(PixelNumber row, PixelNumber col, PixelNumber radius, PixelMode mode);

	// Draw a bitmap
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display. Currently, must be a multiple of 8.
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels. Currently, must be a multiple of 8.
	//  rows = height of bitmap in pixels
	//  data = bitmap image, must be ((width/8) * rows) bytes long
	void Bitmap(PixelNumber top, PixelNumber left, PixelNumber height, PixelNumber width, const uint8_t data[]);

	// Draw a bitmap row
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels
	//  data = bitmap image, must be ((width + 7)/8) bytes long
	void BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert);

private:
	const LcdFont * const *fonts;
	const size_t numFonts;
	size_t currentFontNumber;						// index of the current font
	uint32_t charVal;
	sspi_device device;
	uint16_t lastCharColData;						// data for the last non-space column, used for kerning
	uint8_t numContinuationBytesLeft;
	PixelNumber row, column;
	PixelNumber startRow, startCol, endRow, endCol;	// coordinates of the dirty rectangle
	PixelNumber nextFlushRow;						// which row we need to flush next
	PixelNumber leftMargin, rightMargin;
	uint8_t image[(NumRows * NumCols)/8];			// image buffer, 1K in size
	bool textInverted;
	bool justSetCursor;

	void sendLcdCommand(uint8_t command);
	void sendLcdData(uint8_t data);
	void sendLcd(uint8_t data1, uint8_t data2);
	void CommandDelay();
	void DataDelay();
	void setGraphicsAddress(unsigned int r, unsigned int c);
	size_t writeNative(uint16_t c);					// write a decoded character
	void SetDirty(PixelNumber r, PixelNumber c);
};

#endif

#endif
