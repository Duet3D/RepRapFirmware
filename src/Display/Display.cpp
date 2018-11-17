/*
 * Display.cpp
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#include "Display.h"

#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "IoPorts.h"
#include "Pins.h"

constexpr int DefaultPulsesPerClick = -4;			// values that work with displays I have are 2 and -4

extern const LcdFont font11x14;
extern const LcdFont font7x11;

const LcdFont * const fonts[] = { &font7x11, &font11x14 };
const size_t SmallFontNumber = 0;
const size_t LargeFontNumber = 1;

Display::Display()
	: lcd(LcdCSPin, fonts, ARRAY_SIZE(fonts)), encoder(EncoderPinA, EncoderPinB, EncoderPinSw), menu(lcd), present(false), beepActive(false), updatingFirmware(false)
{
}

void Display::Init()
{
	encoder.Init(DefaultPulsesPerClick);
}

void Display::Spin()
{
	if (present)
	{
		encoder.Poll();

		if (!updatingFirmware)
		{
			// Check encoder and update display
			const int ch = encoder.GetChange();
			if (ch != 0)
			{
				menu.EncoderAction(ch);
			}
			else if (encoder.GetButtonPress())
			{
				menu.EncoderAction(0);
			}
			menu.Refresh();
		}
		lcd.FlushSome();

		if (beepActive && millis() - whenBeepStarted > beepLength)
		{
			IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);
			beepActive = false;
		}
	}
}

void Display::Exit()
{
	if (present)
	{
		IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);		// stop any beep
		if (!updatingFirmware)
		{
			lcd.TextInvert(false);
			lcd.Clear();
			lcd.SetFont(LargeFontNumber);
			lcd.SetCursor(20, 0);
			lcd.print("Shutting down...");
		}
		lcd.FlushAll();
	}
}

// NOTE: nothing enforces that this beep concludes before another is begun;
//   that is, in rapid succession of commands, only the last beep issued will be heard by the user
void Display::Beep(unsigned int frequency, unsigned int milliseconds)
{
	if (present)
	{
		whenBeepStarted = millis();
		beepLength = milliseconds;
		beepActive = true;
		IoPort::WriteAnalog(LcdBeepPin, 0.5, (uint16_t)frequency);
	}
}

void Display::SuccessBeep()
{
	Beep(2000, 100);
}

void Display::ErrorBeep()
{
	Beep(500, 1000);
}

GCodeResult Display::Configure(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;

	if (gb.Seen('P'))
	{
		seen = true;
		switch (gb.GetUIValue())
		{
		case 1:		// 12864 display
			present = true;
			lcd.Init();
			IoPort::SetPinMode(LcdBeepPin, OUTPUT_PWM_LOW);
			lcd.SetFont(SmallFontNumber);
			menu.Load("main");
			break;

		default:
			reply.copy("Unknown display type");
			return GCodeResult::error;
		}
	}

	if (gb.Seen('E'))
	{
		seen = true;
		encoder.Init(gb.GetIValue());			// configure encoder pulses per click and direction
	}

	if (!seen)
	{
		if (present)
		{
			reply.printf("12864 display is configured, pulses-per-click is %d", encoder.GetPulsesPerClick());
		}
		else
		{
			reply.copy("12864 display is not present or not configured");
		}
	}
	return GCodeResult::ok;
}

// Suspend normal operation and display an "Updating firmware" message
void Display::UpdatingFirmware()
{
	updatingFirmware = true;
	if (present)
	{
		IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);		// stop any beep
		lcd.TextInvert(false);
		lcd.Clear();
		lcd.SetFont(LargeFontNumber);
		lcd.SetCursor(20, 0);
		lcd.print("Updating firmware...");
		lcd.FlushAll();
	}
}

// End
