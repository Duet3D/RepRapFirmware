/*
 * ILI9488.h
 *
 *  Created on: 5 May 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_LCD_ILI9488_ILI9488_H_
#define SRC_DISPLAY_LCD_ILI9488_ILI9488_H_

#include <Display/Lcd/TFTLcd.h>

#if SUPPORT_ILI9488_LCD

class LcdILI9488 : public TFTLcd
{
public:
	// Construct a GLCD driver.
	LcdILI9488(const LcdFont * const fnts[], size_t nFonts, uint8_t sercomNum) noexcept;

	~LcdILI9488();

	// Clear part of the display
	void ClearBlock(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right, bool foreground) noexcept override;

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

	// Get the display type
	const char *_ecv_array GetDisplayTypeName() const noexcept override;

protected:
	// Initialise the TFT screen
	void HardwareInit() noexcept override;

	// Start a character at the current row and column, clearing the specified number of space columns
	void StartCharacter(PixelNumber ySize, PixelNumber totalWidth, PixelNumber numSpaceColumns) noexcept override;

	// Write one column of character data at (row, column)
	void WriteColumnData(PixelNumber ySize, uint32_t columnData) noexcept override;

	// Finish writing a character
	void EndCharacter() noexcept override final;

private:
	void SendCommand(uint8_t cmd) noexcept;
	void SendBuffer(const uint16_t *_ecv_array buf, size_t numWords) noexcept;

	// Functions for setting up commands and data in the buffer. Each one takes the address to store it in the buffer and returns the next available buffer address.
	uint16_t *_ecv_array SetGraphicsAddress(uint16_t *_ecv_array buffer, PixelNumber rBegin, PixelNumber rEnd, PixelNumber cBegin, PixelNumber cEnd) noexcept;
	uint16_t *_ecv_array SetColumnMode(uint16_t *_ecv_array buffer, bool columnMode) noexcept;
	uint16_t *_ecv_array SetPixelData(uint16_t *_ecv_array buffer, Colour pixelColour, unsigned int numPixels) noexcept;

	static constexpr unsigned int MaxPixelsPerTransaction = 1024;				// enough for two entire rows (960) and for one 32x28 character plus 3 space columns (992)

	uint16_t spiBuffer[2 + 10 + 1 + (3 * MaxPixelsPerTransaction)];				// large enough to set row or column mode, set the address, and write MaxPixelsPerTransaction
	uint16_t *_ecv_array bufferPointer = spiBuffer;								// tracks the next free location when writing characters
	uint8_t currentRowColMode;

	static constexpr uint8_t CmdReset = 0x01;
	static constexpr uint8_t CmdDisplayOn = 0x29;
	static constexpr uint8_t CmdColumnAddressSet = 0x2A;
	static constexpr uint8_t CmdPageAddressSet = 0x2B;
	static constexpr uint8_t CmdMemoryWrite = 0x2C;
	static constexpr uint8_t CmdMemoryAccessControl = 0x36;
	static constexpr uint8_t CmdInterfacePixelFormat = 0x3A;
	static constexpr uint8_t CmdMemoryWriteContinue = 0x3C;
	static constexpr uint8_t CmdInterfaceModeControl = 0xB0;
	static constexpr uint8_t CmdFrameRateControlNormal = 0xB1;
	static constexpr uint8_t CmdDisplayInversionControl = 0xB4;
	static constexpr uint8_t CmdDisplayFunctionControl = 0xB6;
	static constexpr uint8_t CmdPowerControl1 = 0xC0;
	static constexpr uint8_t CmdPowerControl2 = 0xC1;
	static constexpr uint8_t CmdVComControl1 = 0xC5;
	static constexpr uint8_t CmdPositiveGammaControl = 0xE0;
	static constexpr uint8_t CmdNegativeGammaControl = 0xE1;
	static constexpr uint8_t CmdSetImageFunction = 0xE9;
	static constexpr uint8_t CmdAdjustControl3 = 0xF7;

	static constexpr uint32_t ResetDelayMillis = 5;

//	void CommandDelay() noexcept;
//	void DataDelay() noexcept;
//	void SendByte(uint8_t byteToSend) noexcept;
//	void SetGraphicsAddress(unsigned int r, unsigned int c) noexcept;
//	void StartDataTransaction() noexcept;
//	void EndDataTransaction() noexcept;
//	bool FlushRow() noexcept;
//	void SelectDevice() noexcept;
//	void DeselectDevice() noexcept;

//	constexpr static unsigned int CommandDelayMicros = 72 - 8;	// 72us required, less 7us time to send the command @ 2.0MHz
//	constexpr static unsigned int DataDelayMicros = 4;			// delay between sending data bytes
//	constexpr static unsigned int FlushRowDelayMicros = 20;		// Delay between sending each rows when flushing all rows @ 2.0MHz (@ 1.0MHz this is not necessary)
};

#endif

#endif /* SRC_DISPLAY_LCD_ILI9488_ILI9488_H_ */
