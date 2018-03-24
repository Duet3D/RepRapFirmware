/*
 * Display.cpp
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#include "Display.h"

#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "IoPort.h"
#include "Pins.h"

constexpr int DefaultPulsesPerClick = -4;			// values that work with displays I have are 2 and -4

extern const LcdFont font11x14;
//extern const LcdFont font10x10;

static int val = 0;

Display::Display()
	: lcd(LcdCSPin), encoder(EncoderPinA, EncoderPinB, EncoderPinSw), present(false)
{
	//TODO init menus here
}

void Display::Init()
{
	lcd.Init();
	encoder.Init(DefaultPulsesPerClick);

	//TODO display top menu here
	// For now we just print some text to test the display
	lcd.SetFont(&font11x14);

	lcd.SetCursor(5, 5);
	lcd.SetRightMargin(128);
	lcd.print(reprap.GetPlatform().GetElectronicsString());

	lcd.SetCursor(20, 5);
	lcd.SetRightMargin(50);
	lcd.print(val);

	IoPort::SetPinMode(LcdBeepPin, OUTPUT_PWM_LOW);
	beepActive = false;
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

	if (beepActive && millis() - whenBeepStarted > beepLength)
	{
		IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);
	}
}

void Display::Exit()
{
	// TODO display a "shutdown" message, or turn the display off?
}

void Display::Beep(unsigned int frequency, unsigned int milliseconds)
{
	whenBeepStarted = millis();
	beepLength = milliseconds;
	beepActive = true;
	IoPort::WriteAnalog(LcdBeepPin, 0.5, (uint16_t)frequency);
}

GCodeResult Display::Configure(GCodeBuffer& gb, const StringRef& reply)
{
	if (gb.Seen('P') && gb.GetUIValue() == 1)
	{
		// 12864 display configuration
		present = true;
		if (gb.Seen('E'))
		{
			encoder.Init(gb.GetIValue());			// configure encoder pulses per click and direction
		}
	}
	return GCodeResult::ok;
}

// Suspend normal operation and display an "Updating firmware" message
void Display::UpdatingFirmware()
{
	IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);		// stop any beep
	lcd.TextInvert(false);
	lcd.Clear();
	lcd.SetFont(&font11x14);
	lcd.SetCursor(20, 0);
	lcd.print("Updating firmware...");
	lcd.FlushAll();
}

// End
