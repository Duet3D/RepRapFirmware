/*
 * Display.h
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_DISPLAY_H_
#define SRC_DISPLAY_DISPLAY_H_

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include "RotaryEncoder.h"
#include "Lcd/Lcd.h"
#include "Menu.h"
#include "GCodes/GCodeResult.h"

class Display
{
public:
	Display() noexcept;

	void Init() noexcept { }
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply);
	void Spin() noexcept;
	void Exit() noexcept;
	void Beep(unsigned int frequency, unsigned int milliseconds) noexcept;
	void SuccessBeep() noexcept;
	void ErrorBeep() noexcept;
	bool IsPresent() const noexcept { return lcd != nullptr; }
	void UpdatingFirmware() noexcept;

	constexpr static uint8_t DefaultDisplayContrastRatio = 50;

private:
	void InitDisplay(GCodeBuffer& gb, Lcd *newLcd, bool defaultCsPolarity);

	Lcd *lcd;
	Menu *menu;
	RotaryEncoder *encoder;
	uint32_t whenBeepStarted;
	uint32_t beepLength;
	uint32_t lastRefreshMillis;
	uint16_t mboxSeq;
	bool mboxActive;
	bool beepActive;
	bool updatingFirmware;
};

#endif

#endif /* SRC_DISPLAY_DISPLAY_H_ */
