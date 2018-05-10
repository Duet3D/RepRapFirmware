/*
 * Display.h
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_DISPLAY_H_
#define SRC_DISPLAY_DISPLAY_H_

#include "RotaryEncoder.h"
#include "ST7920/lcd7920.h"
#include "Menu.h"
#include "GCodes/GCodeResult.h"

class Display
{
public:
	Display();

	void Init();
	void Start();
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply);
	void Spin(bool full);
	void Exit();
	void Beep(unsigned int frequency, unsigned int milliseconds);
	void SuccessBeep();
	void ErrorBeep();
	bool IsPresent() const { return present; }
	void UpdatingFirmware();

private:
	Lcd7920 lcd;
	RotaryEncoder encoder;
	Menu menu;
	uint32_t whenBeepStarted;
	uint32_t beepLength;
	bool present;
	bool beepActive;
	bool updatingFirmware;
};

#endif /* SRC_DISPLAY_DISPLAY_H_ */
