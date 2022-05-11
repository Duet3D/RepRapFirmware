/*
 * Display.h
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_DISPLAY_H_
#define SRC_DISPLAY_DISPLAY_H_

#include "RepRapFirmware.h"

#if SUPPORT_DIRECT_LCD

#if SUPPORT_ROTARY_ENCODER
# include "RotaryEncoder.h"
#endif
#if SUPPORT_RESISTIVE_TOUCH
# include "ResistiveTouch.h"
#endif

#include "Lcd/Lcd.h"
#include "Menu.h"
#include <ObjectModel/ObjectModel.h>

class Display INHERIT_OBJECT_MODEL
{
public:
	Display() noexcept;

	void Init() noexcept { }
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void Spin() noexcept;
	void Exit() noexcept;
	void Beep(unsigned int frequency, unsigned int milliseconds) noexcept;
	void SuccessBeep() noexcept;
	void ErrorBeep() noexcept;
	bool IsPresent() const noexcept { return lcd != nullptr; }
	void UpdatingFirmware() noexcept;
	void Tick() noexcept;

	constexpr static uint8_t DefaultDisplayContrastRatio = 30;		// this works well for the Fysetc display
	constexpr static uint8_t DefaultDisplayResistorRatio = 6;		// the recommended Fysetc display uses 6, some other displays use 3

protected:
	DECLARE_OBJECT_MODEL

private:
	void InitDisplay(GCodeBuffer& gb, Lcd *newLcd, Pin csPin, Pin a0Pin, bool defaultCsPolarity) THROWS(GCodeException);
	void StopBeep() noexcept;

	Lcd *null lcd;
	Menu *menu;
#if SUPPORT_ROTARY_ENCODER
	RotaryEncoder *null encoder;
#endif
#if SUPPORT_RESISTIVE_TOUCH
	ResistiveTouch *null touchController;
#endif
	volatile uint32_t beepTicksToGo;
	uint32_t lastRefreshMillis;
	uint16_t mboxSeq;
	bool mboxActive;
	bool updatingFirmware;
};

#endif

#endif /* SRC_DISPLAY_DISPLAY_H_ */
