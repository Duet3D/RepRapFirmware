/*
 * MenuItem.cpp
 *
 *  Created on: 7 May 2018
 *      Author: David
 */

#include "MenuItem.h"
#include "RepRap.h"
#include "Heating/Heat.h"
#include "Platform.h"
#include "GCodes/GCodes.h"
#include "Display.h"

MenuItem::MenuItem(PixelNumber r, PixelNumber c, FontNumber fn)
	: row(r), column(c), fontNumber(fn), next(nullptr)
{
}

/*static*/ void MenuItem::AppendToList(MenuItem **root, MenuItem *item)
{
	while (*root != nullptr)
	{
		root = &((*root)->next);
	}
	item->next = nullptr;
	*root = item;
}

ButtonMenuItem *ButtonMenuItem::freelist = nullptr;

ButtonMenuItem::ButtonMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, const char* t, const char* cmd)
	: MenuItem(r, c, fn), text(t), command(cmd)
{
}

void ButtonMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight)
{
	lcd.SetCursor(row, column);
	lcd.SetRightMargin(rightMargin);
	lcd.TextInvert(highlight);
	lcd.print(text);
}

ValueMenuItem::ValueMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, PixelNumber w, unsigned int v, unsigned int d)
	: MenuItem(r, c, fn), valIndex(v), currentValue(0.0), width(w), decimals(d), adjusting(false)
{
}

void ValueMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight)
{
	lcd.SetCursor(row, column);
	lcd.SetRightMargin(min<PixelNumber>(column + width, rightMargin));
	lcd.TextInvert(highlight);

	bool error = false;
	if (!adjusting)
	{
		const unsigned int itemNumber = valIndex % 100;
		switch (valIndex/100)
		{
		case 0:		// heater current temperature
			currentValue = reprap.GetGCodes().GetItemCurrentTemperature(itemNumber);
			break;

		case 1:		// heater active temperature
			currentValue = reprap.GetGCodes().GetItemActiveTemperature(itemNumber);
			break;

		case 2:		// heater standby temperature
			currentValue = reprap.GetGCodes().GetItemStandbyTemperature(itemNumber);
			break;

		case 3:		// fan %
			currentValue = ((itemNumber == 99)
							? reprap.GetGCodes().GetMappedFanSpeed()
							: reprap.GetPlatform().GetFanValue(itemNumber)
						   ) * 100.0;
			break;

		case 4:		// extruder %
			currentValue = reprap.GetGCodes().GetExtrusionFactor(itemNumber) * 100.0;
			break;

		case 5:		// misc
			switch (itemNumber)
			{
			case 0:
				currentValue = reprap.GetGCodes().GetSpeedFactor() * 100.0;
				break;

			default:
				error = true;
			}
			break;

		default:
			error = true;
			break;
		}
	}

	if (error)
	{
		lcd.print("***");
	}
	else
	{
		lcd.print(currentValue, decimals);
	}
	lcd.ClearToMargin();
}

const char* ValueMenuItem::Select()
{
	adjusting = true;
	return nullptr;
}

bool ValueMenuItem::Adjust(int clicks)
{
	if (clicks == 0)	// if button has been pressed
	{
		bool error = false;
		const unsigned int itemNumber = valIndex % 100;
		switch (valIndex/100)
		{
		case 1:		// heater active temperature
			reprap.GetGCodes().SetItemActiveTemperature(itemNumber, currentValue);
			break;

		case 2:		// heater standby temperature
			reprap.GetGCodes().SetItemStandbyTemperature(itemNumber, currentValue);
			break;

		case 3:		// fan %
			if (itemNumber == 99)
			{
				reprap.GetGCodes().SetMappedFanSpeed(currentValue * 0.01);
			}
			else
			{
				reprap.GetPlatform().SetFanValue(itemNumber, currentValue * 0.01);
			}
			break;

		case 4:		// extruder %
			reprap.GetGCodes().SetExtrusionFactor(itemNumber, currentValue * 0.01);
			break;

		case 5:		// misc
			switch (itemNumber)
			{
			case 0:
				reprap.GetGCodes().SetSpeedFactor(currentValue * 0.01);
				break;

			default:
				error = true;
				break;
			}
			break;

		default:
			error = true;
			break;
		}

		if (error)
		{
			reprap.GetDisplay().ErrorBeep();
		}
		adjusting = false;
		return true;
	}

	currentValue += (float)clicks;			// currently we always adjust by 1
	return false;
}

FilesMenuItem *FilesMenuItem::freelist = nullptr;

FilesMenuItem::FilesMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, const char *cmd, const char *dir, unsigned int nf)
	: MenuItem(r, c, fn), initialDirectory(dir), numFiles(nf)
{
}

void FilesMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight)
{
	lcd.SetCursor(row, column);
	lcd.SetRightMargin(rightMargin);
	lcd.print("File list here...");
	//TODO
}

// End

