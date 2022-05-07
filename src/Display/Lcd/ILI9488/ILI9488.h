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

	// Flush just some data, returning true if this needs to be called again
	bool FlushSome() noexcept override;

	// Get the display type
	const char *_ecv_array GetDisplayTypeName() const noexcept override;

protected:
	// Initialise the TFT screen
	void HardwareInit() noexcept override;

private:
	void SendCommand(uint8_t cmd) noexcept;
	void SendCommand(uint8_t cmd, size_t numData, uint8_t data[]) noexcept;
	void SetGraphicsAddress(PixelNumber r, PixelNumber cBegin, PixelNumber cEnd) noexcept;
	void SendBuffer(size_t numWords) const noexcept;

	uint16_t spiBuffer[3 * 480 + 1];							// large enough to write one whole row of pixels

	static constexpr uint8_t CmdReset = 0x01;
	static constexpr uint8_t CmdColumnAddressSet = 0x2A;
	static constexpr uint8_t CmdPageAddressSet = 0x2B;

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
