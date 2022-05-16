/*
  UTouch.cpp - library support for Color TFT LCD Touch screens on SAM3X
  Originally based on Utouch library by Henning Karlsen.
  Rewritten by D Crocker using the approach described in TI app note http://www.ti.com/lit/pdf/sbaa036.
*/

#ifndef SRC_DISPLAY_RESISTIVE_TOUCH_H
#define SRC_DISPLAY_RESISTIVE_TOUCH_H

#include <RepRapFirmware.h>

#if SUPPORT_RESISTIVE_TOUCH

#include "DisplayOrientation.h"
#include <Hardware/Spi/SharedSpiClient.h>

class ResistiveTouch
{
public:
	ResistiveTouch(Pin csp, Pin irqp) noexcept;

	void Init(uint16_t xp, uint16_t yp, DisplayOrientation orientationAdjust) noexcept;
	bool Read(uint16_t &x, uint16_t &y, bool &repeat, uint16_t* null rawX = nullptr, uint16_t *null rawY = nullptr) noexcept;
	void Calibrate(uint16_t xlow, uint16_t xhigh, uint16_t ylow, uint16_t yhigh, uint16_t margin) noexcept;

private:
	SharedSpiClient spiDev;
	Pin csPin, irqPin;
	uint16_t disp_x_size, disp_y_size;
	uint16_t scaleX, scaleY;
	int16_t offsetX, offsetY;

	DisplayOrientation orientAdjust;
	bool pressed;

	bool GetData(bool wantY, uint16_t &rslt) noexcept;
	void WriteCommand(uint8_t command) noexcept;
	uint16_t ReadData(uint8_t command) noexcept;
	static uint16_t Diff(uint16_t a, uint16_t b) noexcept { return (a < b) ? b - a : a - b; }

	static constexpr uint32_t SpiFrequency = 2000000;			// max supported by the XPT2046 is 2.5MHz
};

#endif

#endif	// SRC_DISPLAY_RESISTIVE_TOUCH_H
