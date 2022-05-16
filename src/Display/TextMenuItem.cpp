/*
 * TextMenuItem.cpp
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#include "TextMenuItem.h"

#if SUPPORT_DIRECT_LCD

TextMenuItem::TextMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, const char *_ecv_array t) noexcept
	: MenuItem(r, c, w, a, fn), text(t)
{
}

void TextMenuItem::CorePrint(Lcd& lcd) noexcept
{
	lcd.printf("%s", text);
}

void TextMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept
{
	// We ignore the 'highlight' parameter because text items are not selectable
	if (IsVisible() && (!drawn || itemChanged))
	{
		PrintAligned(lcd, rightMargin);
		itemChanged = false;
		drawn = true;
	}
}

void TextMenuItem::UpdateWidthAndHeight(Lcd& lcd) noexcept
{
	if (width == 0)
	{
		lcd.SetFont(fontNumber);
		lcd.SetCursor(lcd.GetNumRows(), 0);
		lcd.SetRightMargin(lcd.GetNumCols());
		lcd.TextInvert(false);
		lcd.printf("%s", text);
		width = lcd.GetColumn();
		if (align == LeftAlign)
		{
			++width;			// add a space column after left-aligned text with no explicit width, so that the next item can follow immediately
		}
	}
	if (height == 0)
	{
		lcd.SetFont(fontNumber);
		height = lcd.GetFontHeight();
	}
}

#endif

// End
