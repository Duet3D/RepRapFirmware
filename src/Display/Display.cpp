/*
 * Display.cpp
 *
 *  Created on  : 2018-01-22
 *      Author  : David Crocker
 *  Modified on : 2020-05-16
 *      Author  : Martijn Schiedon
 */

// TODO: what function calls this class? Is M918 only allowed once?
// TODO: how is the driver class destructed?
// TODO: Isn't it better to put the graphics drawing stuff in this class so that menu calls Display instead of the driver?
// TODO: Why let beep functionality depend on whether a display is configured or not?
// TODO: Decide if the EXT_0 pin used as DC should be configurable from GCode.

#include "Display.h"

#if SUPPORT_12864_LCD

#include "Fonts/Fonts.h"
#include <Display/ST7920/ST7920.h>
#include <Display/UC1701/UC1701.h>
#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
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

Display::Display() noexcept
	: lcd(nullptr), menu(nullptr), encoder(nullptr), lastRefreshMillis(0),
	  mboxSeq(0), mboxActive(false), beepActive(false), updatingFirmware(false)
{
}

void Display::Spin() noexcept
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

		lcd->Flush();

		if (beepActive && millis() - whenBeepStarted > beepLength)
		{
			StopBeep();
			beepActive = false;
		}
	}
}

void Display::Exit() noexcept
{
	if (lcd != nullptr)
	{
		StopBeep();
		if (!updatingFirmware)
		{
			lcd->TextInvert(false);
			lcd->Clear();
			lcd->SelectFont(LargeFontNumber);
			lcd->SetCursor(20, 0);
			lcd->print("Shutting down...");
		}
		lcd->FlushAll();
	}
}

void Display::SetBeepPin(Pin pin) noexcept
{
	beepPin = pin;
	IoPort::SetPinMode(beepPin, OUTPUT_PWM_LOW);
}

// NOTE: nothing enforces that this beep concludes before another is begun;
//   that is, in rapid succession of commands, only the last beep issued will be heard by the user
void Display::Beep(unsigned int frequency, unsigned int milliseconds) noexcept
{
	if (lcd != nullptr)
	{
		whenBeepStarted = millis();
		beepLength = milliseconds;
		beepActive = true;
		IoPort::WriteAnalog(beepPin, 0.5, (uint16_t)frequency);
	}
}

void Display::StopBeep() noexcept
{
	IoPort::WriteAnalog(beepPin, 0.0, 0);		// stop any beep
}

void Display::SuccessBeep() noexcept
{
	Beep(2000, 100);
}

void Display::ErrorBeep() noexcept
{
	Beep(500, 1000);
}

GCodeResult Display::Configure(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	bool seen = false;

	if (gb.Seen('P'))
	{
		seen = true;
		uint32_t displayType = gb.GetUIValue();

		if (displayType == 1 || displayType == 2)
		{
			// TODO: should replace driver when displayType changes; the current code seems to only allow
			// configuring a driver with M918 once. Spin() is designed to handle a null pointer for the driver,
			// so we can replace this with another driver.
			if (lcd == nullptr)
			{
				switch(displayType)
				{
					case 1:  // ST7920 128x64 display
						lcd = new ST7920(128, 64, LcdCSPin);
						break;
					case 2:  // UC1701 128x64 display
						// NOTE: now using EXP_0, could be made configurable to use BEEP or EXT_1 pins as DC line
						lcd = new UC1701(128, 64, LcdCSPin, Exp0Pin);
						break;
				}
			}

			// The following statement does allow changing the SPI frequency of the driver multiple times
			// NOTE: theoretically, a driver does not necessarily have to be SPI-based, but since M918 assumes serial display drivers,
			// a method to set the serial bus clock frequency has been supported at the DisplayDriver base class level.
			if (gb.Seen('F'))
			{
				lcd->SetBusClockFrequency(gb.GetUIValue());
			}

			lcd->SetFonts(fonts, ARRAY_SIZE(fonts));
			lcd->SelectFont(SmallFontNumber);
			lcd->Init();

			// Future option to make this potentially configurable from a GCode parameter,
			// or to derive and set the pin from the display type
			SetBeepPin(LcdBeepPin);

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
		}
		else
		{
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
void Display::UpdatingFirmware() noexcept
{
	updatingFirmware = true;
	if (lcd != nullptr)
	{
		StopBeep();
		lcd->TextInvert(false);
		lcd->Clear();
		lcd->SelectFont(LargeFontNumber);
		lcd->SetCursor(20, 0);
		lcd->print("Updating firmware...");
		lcd->FlushAll();
	}
}

#endif

// End
