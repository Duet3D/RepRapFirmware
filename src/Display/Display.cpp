/*
 * Display.cpp
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#include "Display.h"
#include "GCodes/GCodes.h"

extern const LcdFont font16x16;
//extern const LcdFont font10x10;

static int val = 0;

Display::Display()
	: lcd(LcdCSPin), encoder(EncoderPinA, EncoderPinB, EncoderPinSw, 4)
{
	//TODO init menus here
}

void Display::Init()
{
	lcd.Init();
	encoder.Init();

	//TODO display top menu here
	// For now we just print some text to test the display
	lcd.SetFont(&font16x16);

	lcd.SetCursor(5, 5);
	lcd.SetRightMargin(128);
	lcd.print(reprap.GetPlatform().GetElectronicsString());

	lcd.SetCursor(20, 5);
	lcd.SetRightMargin(50);
	lcd.print(val);
}

void Display::Spin(bool full)
{
	encoder.Poll();
	if (full)
	{
		// Check encoder and update display here
		// For now we just test the encoder functionality
		const int ch = encoder.GetChange();
		const bool pressed = encoder.GetButtonPress();

		if (ch != 0)
		{
			val += ch;
		}
		if (pressed)
		{
			val += 100;
		}
		if (ch != 0 || pressed)
		{
			if (val < 0) val += 1000;
			if (val >= 1000) val -= 1000;
			lcd.SetCursor(20, 5);
			lcd.SetRightMargin(50);
			lcd.print(val);
			lcd.ClearToMargin();
		}

		lcd.FlushSome();
	}
}

void Display::Exit()
{
	// TODO display a "shutdown" message, or turn the display off?
}

// End
