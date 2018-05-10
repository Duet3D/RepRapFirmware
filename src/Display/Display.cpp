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
extern const LcdFont font7x11;

const LcdFont& smallFont = font7x11;
const LcdFont& largeFont = font11x14;

const LcdFont * const fonts[] = { &font7x11, &font11x14 };

Display::Display()
	: lcd(LcdCSPin), encoder(EncoderPinA, EncoderPinB, EncoderPinSw), menu(lcd, fonts, ARRAY_SIZE(fonts)), present(false), beepActive(false), updatingFirmware(false)
{
}

void Display::Init()
{
	lcd.Init();
	encoder.Init(DefaultPulsesPerClick);
	IoPort::SetPinMode(LcdBeepPin, OUTPUT_PWM_LOW);
}

void Display::Start()
{
	lcd.SetFont(&smallFont);
	menu.Load("main");
}

void Display::Spin(bool full)
{
	encoder.Poll();

	if (full)
	{
		if (!updatingFirmware)
		{
			// Check encoder and update display here
			// For now we just test the encoder functionality
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
	}

	if (beepActive && millis() - whenBeepStarted > beepLength)
	{
		IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);
	}
}

void Display::Exit()
{
	IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);		// stop any beep
	if (!updatingFirmware)
	{
		lcd.TextInvert(false);
		lcd.Clear();
		lcd.SetFont(&largeFont);
		lcd.SetCursor(20, 0);
		lcd.print("Shutting down...");
	}
	lcd.FlushAll();
}

void Display::Beep(unsigned int frequency, unsigned int milliseconds)
{
	whenBeepStarted = millis();
	beepLength = milliseconds;
	beepActive = true;
	IoPort::WriteAnalog(LcdBeepPin, 0.5, (uint16_t)frequency);
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
	updatingFirmware = true;
	IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);		// stop any beep
	lcd.TextInvert(false);
	lcd.Clear();
	lcd.SetFont(&largeFont);
	lcd.SetCursor(20, 0);
	lcd.print("Updating firmware...");
	lcd.FlushAll();
}

// End
