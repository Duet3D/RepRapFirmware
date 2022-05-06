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
	LcdILI9488(const LcdFont * const fnts[], size_t nFonts) noexcept;

	// Flush just some data, returning true if this needs to be called again
	bool FlushSome() noexcept override;

	// Get the display type
	const char *GetDisplayTypeName() const noexcept override;

protected:
	// Write one column of character data at (row, column)
	void WriteColumnData(uint16_t columData, uint8_t ySize) noexcept override;

private:
	void CommandDelay() noexcept;
	void DataDelay() noexcept;
	void SendByte(uint8_t byteToSend) noexcept;
	void SetGraphicsAddress(unsigned int r, unsigned int c) noexcept;
	uint8_t TransformTile(uint8_t data[8], PixelNumber c) noexcept;
	void StartDataTransaction() noexcept;
	void EndDataTransaction() noexcept;
	bool FlushRow() noexcept;
	void SelectDevice() noexcept;
	void DeselectDevice() noexcept;

//	constexpr static unsigned int CommandDelayMicros = 72 - 8;	// 72us required, less 7us time to send the command @ 2.0MHz
//	constexpr static unsigned int DataDelayMicros = 4;			// delay between sending data bytes
//	constexpr static unsigned int FlushRowDelayMicros = 20;		// Delay between sending each rows when flushing all rows @ 2.0MHz (@ 1.0MHz this is not necessary)
};

#endif

#endif /* SRC_DISPLAY_LCD_ILI9488_ILI9488_H_ */
