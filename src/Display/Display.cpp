/*
 * Display.cpp
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#include "Display.h"

#if SUPPORT_12864_LCD

#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "Hardware/IoPorts.h"
#include "Pins.h"

constexpr int DefaultPulsesPerClick = -4;			// values that work with displays I have are 2 and -4

extern const LcdFont font11x14;
extern const LcdFont font7x11;

const LcdFont * const fonts[] = { &font7x11, &font11x14 };
constexpr size_t SmallFontNumber = 0;
constexpr size_t LargeFontNumber = 1;

constexpr uint32_t NormalRefreshMillis = 250;
constexpr uint32_t FastRefreshMillis = 50;

Display::Display()
	: lcd(nullptr), menu(nullptr), encoder(nullptr), lastRefreshMillis(0),
	  mboxSeq(0), mboxActive(false), beepActive(false), updatingFirmware(false)
{
}

void Display::Spin()
{
	if (lcd != nullptr)
	{
		encoder->Poll();

		if (!updatingFirmware)
		{
			// Check encoder and update display
			const int ch = encoder->GetChange();
			bool forceRefresh = false;
			if (ch != 0)
			{
				menu->EncoderAction(ch);
				forceRefresh = true;
			}
			else if (encoder->GetButtonPress())
			{
				menu->EncoderAction(0);
				forceRefresh = true;
			}

			const MessageBox& mbox = reprap.GetMessageBox();
			if (mbox.active)
			{
				if (!mboxActive || mboxSeq != mbox.seq)
				{
					// New message box to display
					if (!mboxActive)
					{
						menu->ClearHighlighting();					// cancel highlighting and adjustment
						menu->Refresh();
					}
					mboxActive = true;
					mboxSeq = mbox.seq;
					menu->DisplayMessageBox(mbox);
					forceRefresh = true;
				}
			}
			else if (mboxActive)
			{
				// Message box has been cancelled from this or another input channel
				menu->ClearMessageBox();
				mboxActive = false;
				forceRefresh = true;
			}

			const uint32_t now = millis();
			if (forceRefresh)
			{
				menu->Refresh();
				// To avoid a noticeable delay in updating the coordinates and babystepping offset when live adjusting them and we stop rotating the encoder,
				// we force another update 50ms after any encoder actions
				lastRefreshMillis = now - (NormalRefreshMillis - FastRefreshMillis);
			}
			else if (now - lastRefreshMillis >= NormalRefreshMillis)
			{
				menu->Refresh();
				// When the encoder is inactive, we update at most 5 times per second, to avoid rapidly-changing values flickering on the display
				lastRefreshMillis = now;
			}
		}
		lcd->FlushSome();

		if (beepActive && millis() - whenBeepStarted > beepLength)
		{
			IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);
			beepActive = false;
		}
	}
}

void Display::Exit()
{
	if (lcd != nullptr)
	{
		IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);		// stop any beep
		if (!updatingFirmware)
		{
			lcd->TextInvert(false);
			lcd->Clear();
			lcd->SetFont(LargeFontNumber);
			lcd->SetCursor(20, 0);
			lcd->print("Shutting down...");
		}
		lcd->FlushAll();
	}
}

// NOTE: nothing enforces that this beep concludes before another is begun;
//   that is, in rapid succession of commands, only the last beep issued will be heard by the user
void Display::Beep(unsigned int frequency, unsigned int milliseconds)
{
	if (lcd != nullptr)
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
			if (lcd == nullptr)
			{
				lcd = new Lcd7920(LcdCSPin, fonts, ARRAY_SIZE(fonts));
			}
			if (gb.Seen('F'))
			{
				lcd->SetSpiClockFrequency(gb.GetUIValue());
			}
			lcd->Init();
			IoPort::SetPinMode(LcdBeepPin, OUTPUT_PWM_LOW);
			lcd->SetFont(SmallFontNumber);

			if (encoder == nullptr)
			{
				encoder = new RotaryEncoder(EncoderPinA, EncoderPinB, EncoderPinSw);
				encoder->Init(DefaultPulsesPerClick);
			}
			if (menu == nullptr)
			{
				menu = new Menu(*lcd);
			}
			menu->Load("main");
			break;

		default:
			reply.copy("Unknown display type");
			return GCodeResult::error;
		}
	}

	if (gb.Seen('E') && encoder != nullptr)
	{
		seen = true;
		encoder->Init(gb.GetIValue());			// configure encoder pulses per click and direction
	}

	if (!seen)
	{
		if (lcd != nullptr)
		{
			reply.printf("12864 display is configured, pulses-per-click is %d", encoder->GetPulsesPerClick());
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
	if (lcd != nullptr)
	{
		IoPort::WriteAnalog(LcdBeepPin, 0.0, 0);		// stop any beep
		lcd->TextInvert(false);
		lcd->Clear();
		lcd->SetFont(LargeFontNumber);
		lcd->SetCursor(20, 0);
		lcd->print("Updating firmware...");
		lcd->FlushAll();
	}
}

#endif

// End
