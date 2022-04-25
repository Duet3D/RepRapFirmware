/*
 * ValueMenuItem.cpp
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#include "ValueMenuItem.h"
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>
#include <PrintMonitor/PrintMonitor.h>
#include <Heating/Heat.h>
#include <Movement/Move.h>
#include <Tools/Tool.h>
#include "Display.h"

#ifdef __LPC17xx__
# include "Network.h"
#else
# include <Networking/Network.h>
#endif

ValueMenuItem::ValueMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis, bool adj, unsigned int v, unsigned int d) noexcept
	: MenuItem(r, c, ((w != 0) ? w : DefaultWidth), a, fn, vis), valIndex(v), currentFormat(PrintFormat::undefined), decimals(d), adjusting(AdjustMode::displaying), adjustable(adj)
{
}

void ValueMenuItem::CorePrint(Lcd& lcd) noexcept
{
	if (adjustable)
	{
		lcd.WriteSpaces(1);
	}

	if (error)
	{
		lcd.printf("***");
	}
	else
	{
		switch (currentFormat)
		{
		case PrintFormat::asFloat:
			lcd.printf("%.*f", decimals, (double)currentValue.f);
			break;

		case PrintFormat::asPercent:
			lcd.printf("%.*f%%", decimals, (double)currentValue.f);
			break;

		case PrintFormat::asUnsigned:
			lcd.printf("%u", currentValue.u);
			break;

		case PrintFormat::asSigned:
			lcd.printf("%d", currentValue.i);
			break;

		case PrintFormat::asText:
			lcd.printf("%s", textValue);
			break;

		case PrintFormat::asIpAddress:
			lcd.printf("%u.%u.%u.%u", currentValue.u & 0x000000FF, (currentValue.u >> 8) & 0x0000000FF, (currentValue.u >> 16) & 0x0000000FF, (currentValue.u >> 24) & 0x0000000FF);
			break;

		case PrintFormat::asTime:
			{
				unsigned int hours = currentValue.u/3600,
					minutes = (currentValue.u / 60) % 60,
					seconds = currentValue.u % 60;
				lcd.printf("%u:%02u:%02u", hours, minutes, seconds);
			}
			break;

		case PrintFormat::undefined:
		default:
			lcd.printf("***");
			break;
		}
	}
}

void ValueMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) noexcept
{
	if (IsVisible())
	{
		error = false;
		textValue = nullptr;

		if (valIndex == 501)
		{
			// Item 501 is a special case because it is text, not a number. We store the current message sequence number in currentValue.
			uint16_t newSeq;
			textValue = reprap.GetLatestMessage(newSeq);
			if (newSeq != currentValue.u)
			{
				itemChanged = true;
				currentValue.u = newSeq;
				currentFormat = PrintFormat::asText;
			}
		}
		else if (adjusting != AdjustMode::adjusting)
		{
			const unsigned int itemNumber = valIndex % 100;
			const Value oldValue = currentValue;
			currentFormat = PrintFormat::asFloat;

			switch (valIndex/100)
			{
			case 0:		// heater current temperature
				currentValue.f = max<float>(reprap.GetGCodes().GetItemCurrentTemperature(itemNumber), 0.0);
				break;

			case 1:		// heater active temperature
				currentValue.f = max<float>(reprap.GetGCodes().GetItemActiveTemperature(itemNumber), 0.0);
				break;

			case 2:		// heater standby temperature
				currentValue.f = max<float>(reprap.GetGCodes().GetItemStandbyTemperature(itemNumber), 0.0);
				break;

			case 3:		// fan %
				currentValue.f = ((itemNumber == 99)
								? reprap.GetGCodes().GetPrimaryMovementState().virtualFanSpeed
								: reprap.GetFansManager().GetFanValue(itemNumber)
							   ) * 100.0;
				currentFormat = PrintFormat::asPercent;
				break;

			case 4:		// extruder %
				currentValue.f = reprap.GetGCodes().GetExtrusionFactor(itemNumber) * 100.0;
				currentFormat = PrintFormat::asPercent;
				break;

			case 5:		// misc
				switch (itemNumber)
				{
				case 0:
					currentValue.f = reprap.GetGCodes().GetPrimarySpeedFactor() * 100.0;
					currentFormat = PrintFormat::asPercent;
					break;

				// case 1 is the latest message sent by M117, but it handled at the start

				case 10: // X
				case 11: // Y
				case 12: // Z
				case 13: // U
				case 14: // V
				case 15: // W
					currentValue.f = reprap.GetGCodes().GetUserCoordinate(reprap.GetGCodes().GetPrimaryMovementState(), itemNumber - 10);
					break;

				case 20:
					currentValue.i = reprap.GetGCodes().GetPrimaryMovementState().GetCurrentToolNumber();
					currentFormat = PrintFormat::asSigned;
					break;

				case 21:	// Z baby-step
					currentValue.f = reprap.GetGCodes().GetTotalBabyStepOffset(Z_AXIS);
					break;

				// Platform's IP address is the "planned", Network's IP address is the "actual"
				case 30:
				case 31:
				case 32:
				case 33:
					currentValue.u = reprap.GetNetwork().GetIPAddress(0).GetQuad(itemNumber - 30);
					currentFormat = PrintFormat::asUnsigned;
					break;

				case 34:	// IP address in one go
					currentValue.u = reprap.GetNetwork().GetIPAddress(0).GetV4LittleEndian();
					currentFormat = PrintFormat::asIpAddress;
					break;

				case 35:	// Percentage of file that has been processed
					currentValue.f = (reprap.GetPrintMonitor().IsPrinting())
										? reprap.GetPrintMonitor().FractionOfFilePrinted() * 100.0
											: 0;
					currentFormat = PrintFormat::asPercent;
					break;

				case 36:	// Print time remaining, file-based
					currentValue.u = (reprap.GetPrintMonitor().IsPrinting())
										? static_cast<int>(reprap.GetPrintMonitor().EstimateTimeLeft(PrintEstimationMethod::fileBased))
											: 0;
					currentFormat = PrintFormat::asTime;
					break;

				case 37:	// Print time remaining, filament-based
					currentValue.u = (reprap.GetPrintMonitor().IsPrinting())
										? static_cast<int>(reprap.GetPrintMonitor().EstimateTimeLeft(PrintEstimationMethod::filamentBased))
											: 0;
					currentFormat = PrintFormat::asTime;
					break;

				case 38:	// requested speed
					currentValue.f = reprap.GetMove().GetRequestedSpeedMmPerSec();
					break;

				case 39:	// top speed
					currentValue.f = reprap.GetMove().GetTopSpeedMmPerSec();
					break;

				default:
					error = true;
				}
				break;

			default:
				error = true;
				break;
			}

			if (error)
			{
				itemChanged = true;
			}
			else
			{
				switch (currentFormat)
				{
				case PrintFormat::undefined:
					itemChanged = true;
					break;

				case PrintFormat::asFloat:
					if (currentValue.f != oldValue.f)
					{
						itemChanged = true;
					}
					break;

				case PrintFormat::asSigned:
					if (currentValue.i != oldValue.i)
					{
						itemChanged = true;
					}
					break;

				case PrintFormat::asUnsigned:
				case PrintFormat::asIpAddress:
				case PrintFormat::asText:
				case PrintFormat::asTime:
				default:
					if (currentValue.u != oldValue.u)
					{
						itemChanged = true;
					}
					break;
				}
			}
		}

		if (itemChanged || !drawn || (highlight != highlighted))
		{
			highlighted = highlight;
			PrintAligned(lcd, tOffset, rightMargin);
			itemChanged = false;
			drawn = true;
		}
	}
}

bool ValueMenuItem::Select(const StringRef& cmd) noexcept
{
	adjusting = AdjustMode::adjusting;
	return false;
}

void ValueMenuItem::UpdateWidthAndHeight(Lcd& lcd) noexcept
{
	// The width is always set for a ValueMenuItem so we just need to determine the height
	if (height == 0)
	{
		lcd.SetFont(fontNumber);
		height = lcd.GetFontHeight();
	}
}

PixelNumber ValueMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept
{
	// TODO
	return 0;
}

bool ValueMenuItem::Adjust_SelectHelper() noexcept
{
	if (adjusting == AdjustMode::adjusting)
	{
		const unsigned int itemNumber = GetReferencedToolNumber();

		bool err = false;
		switch (valIndex/100)
		{
		case 1: // heater active temperature
			reprap.GetGCodes().SetItemActiveTemperature(itemNumber, (currentValue.f < 1.0) ? ABS_ZERO : currentValue.f);
			break;

		case 2: // heater standby temperature
			reprap.GetGCodes().SetItemStandbyTemperature(itemNumber, (currentValue.f < 1.0) ? ABS_ZERO : currentValue.f);
			break;

		case 3: // fan %
			if (itemNumber == 99)
			{
				reprap.GetGCodes().SetMappedFanSpeed(nullptr, currentValue.f * 0.01);
			}
			else
			{
				reprap.GetFansManager().SetFanValue(itemNumber, currentValue.f * 0.01);
			}
			break;

		case 4: // extruder %
			reprap.GetGCodes().SetExtrusionFactor(itemNumber, currentValue.f * 0.01);
			break;

		case 5: // misc.
			switch (itemNumber)
			{
			case 0:
				reprap.GetGCodes().SetPrimarySpeedFactor(currentValue.f * 0.01);
				break;

			case 20:
				reprap.GetGCodes().SelectPrimaryTool(currentValue.i, false);
				break;

			case 21: // baby stepping
				break;

			default:
				err = true;
				break;
			}
			break;

		default:
			err = true;
			break;
		}

		if (err)
		{
			reprap.GetDisplay().ErrorBeep();
		}
	}

	adjusting = AdjustMode::displaying;
	return true;
}

unsigned int ValueMenuItem::GetReferencedToolNumber() const noexcept
{
	unsigned int uToolNumber = valIndex % 100;
	if (79 == uToolNumber)
	{
		uToolNumber = reprap.GetGCodes().GetPrimaryMovementState().GetCurrentToolNumber();
	}

	return uToolNumber;
}

// Adjust the value of this item by 'clicks' click of the encoder. 'clicks' is nonzero.
// Return true if we have finished adjusting it.
bool ValueMenuItem::Adjust_AlterHelper(int clicks) noexcept
{
	itemChanged = true;			// we will probably change the value, so it will need to be re-displayed
	const unsigned int itemNumber = GetReferencedToolNumber();

	switch (valIndex/100)
	{
	case 1:	// heater active temperature
	case 2:	// heater standby temperature
		if (itemNumber < 80) // Tool heaters
		{
			// If we're decreasing, make any value smaller than 95 go to 0
			// If we're increasing, make any value between 0 and 95 jump directly to 95
			// Also cap the maximum
			if (0 > clicks) // decrementing
			{
				currentValue.f += (float)clicks;
				if (95.0 > currentValue.f)
				{
					currentValue.f = 0.0;
				}
			}
			else // incrementing
			{
				if (0.0 == currentValue.f)
				{
					currentValue.f = 95.0 - 1.0;
				}
				currentValue.f = min<int>(currentValue.f + (float)clicks, reprap.GetHeat().GetHighestTemperatureLimit(Tool::GetLockedTool(itemNumber)->GetHeater(0)));
			}
		}
		else
		{
			currentValue.f += (float)clicks;
		}
		break;

	case 3: // fan %
		currentValue.f = constrain<float>(currentValue.f + (float)clicks, 0.0, 100.0);
		break;

	case 5: // misc.
		switch (itemNumber)
		{
		case 0: // 500 Feed Rate
			currentValue.f = constrain<float>(currentValue.f + (float)clicks, 10.0, 500.0);
			break;

		case 20: // 520 Tool Selection
			currentValue.i = constrain<int>((int)currentValue.f + clicks, -1, 255);
			break;

		case 21: // 521 baby stepping
			{
				String<ShortGCodeLength> cmd;
				cmd.printf("M290 Z%.2f", (double)(0.02 * clicks));
				(void) reprap.GetGCodes().ProcessCommandFromLcd(cmd.c_str());
				adjusting = AdjustMode::liveAdjusting;
			}
			break;

		default:
			if (itemNumber >= 10 && itemNumber < 10 + reprap.GetGCodes().GetVisibleAxes())	// 510-518 axis position adjustment
			{
				String<ShortGCodeLength> cmd;
				const float amount = ((itemNumber == 12) ? 0.02 : 0.1) * clicks;			// 0.02mm Z resolution, 0.1mm for other axes
				cmd.printf("M120 G91 G1 F3000 %c%.2f M121", 'X' + (itemNumber - 10), (double)amount);
				(void) reprap.GetGCodes().ProcessCommandFromLcd(cmd.c_str());
				adjusting = AdjustMode::liveAdjusting;
			}
			break;
		}
		break;

	default:
		currentValue.f += (float)clicks;
		break;
	}

	return false;
}

// Adjust this element, returning true if we have finished adjustment.
// 'clicks' is the number of encoder clicks to adjust by, or 0 if the button was pushed.
bool ValueMenuItem::Adjust(int clicks) noexcept
{
	return (clicks == 0)						// if button has been pressed
			?  Adjust_SelectHelper()
				: Adjust_AlterHelper(clicks);
}

// End

