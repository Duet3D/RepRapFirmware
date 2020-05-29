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
// TODO: isn't it better to put the graphics drawing stuff in this class so that menu calls Display instead of the driver?
// TODO: why let beep functionality depend on whether a display is configured or not?
// TODO: decide if the EXT_0 pin used as DC should be configurable from GCode.
// TODO: discuss with David to maybe use exp_1 as csPin and set LcdCSPin (PortCPin(9)) high @ sspi_select_device so we can do without NAND inverter?
//       or to use a more logical arrangement of unbuffered pins on Drive 5 in the Maestro or the temp daughterboard header?
//       what about the pins for the Duet Wifi and Ethernet?

#include "Display.h"

#if SUPPORT_12864_LCD

#include "Fonts/Fonts.h"
#include <Display/ST7920/ST7920.h>
#include <Display/ST7565/ST7565.h>
#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Hardware/IoPorts.h"
#include "Pins.h"

// Value that seem to work fine with most displays
constexpr int DefaultPulsesPerClick = -4;

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
						// Cancel highlighting and adjustment
						menu->ClearHighlighting();
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

		lcd->FlushRow();

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
		lcd->Flush();
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

		if (displayType >= 1 && displayType <= 3)
		{
			// TODO: should replace driver when displayType changes; the current code seems to only allow
			// configuring a driver with M918 once. Spin() is designed to handle a null pointer for the driver,
			// so we can replace this with another driver.
			if (lcd == nullptr)
			{
				switch(displayType)
				{
					case 1:  // ST7920 128x64 display type
						lcd = new ST7920(128, 64, LcdCSPin);
						break;
					case 2:  // ST7565 128x64 display type
						// This configuration requires no additional components IF the display logic is +3.3v
						// It uses EXP_0 as A0/DC and EXP_1 as CS (active low).
						// LCD_CS is disconnected but still needs to be controlled to gate MOSI and SCK (active high).
						lcd = new ST7565(128, 64, Exp1Pin, Exp0Pin, false, LcdCSPin);
						break;
					case 3:  // ST7565 128x64 display type
						// The driver uses the default buffered LCD_CS pin.
						// This requires an additional external (74xxx00) component to invert the logic level of LCD_CS.
						// NOTE: now using EXP_0 as DC/A0 , but could be made configurable to other pins?
						//       David used 'A0Pin' so the pin could be redefined in the Pins_<Config>.h
						lcd = new ST7565(128, 64, LcdCSPin, Exp0Pin);
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
		lcd->Flush();
	}
}

#endif

// End
