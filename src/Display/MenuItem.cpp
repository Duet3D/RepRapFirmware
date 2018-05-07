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

MenuItem::MenuItem(PixelNumber r, PixelNumber c, FontNumber fn)
	: row(r), column(c), fontNumber(fn), next(nullptr)
{
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
	: MenuItem(r, c, fn), valIndex(v), width(w), decimals(d)
{
}

void ValueMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight)
{
	lcd.SetCursor(row, column);
	lcd.SetRightMargin(min<PixelNumber>(column + width, rightMargin));
	lcd.TextInvert(highlight);

	const unsigned int itemNumber = valIndex % 100;
	float valueToPrint;
	bool error = false;
	switch (valIndex/100)
	{
	case 0:		// heater current temperature
		valueToPrint = reprap.GetHeat().GetTemperature(itemNumber);
		break;

	case 1:		// heater active temperature
		valueToPrint = reprap.GetHeat().GetActiveTemperature(itemNumber);
		break;

	case 2:		// heater standby temperature
		valueToPrint = reprap.GetHeat().GetStandbyTemperature(itemNumber);
		break;

	case 3:		// fan %
		valueToPrint = reprap.GetPlatform().GetFanValue(itemNumber) * 100.0;
		break;

	case 4:		// extruder %
		valueToPrint = reprap.GetGCodes().GetExtrusionFactor(itemNumber) * 100.0;
		break;

	case 5:		// misc
		switch (itemNumber)
		{
		case 0:
			valueToPrint = reprap.GetGCodes().GetSpeedFactor() * 100.0;
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
		lcd.print("***");
	}
	else
	{
		lcd.print(valueToPrint, decimals);
	}
	lcd.ClearToMargin();
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

