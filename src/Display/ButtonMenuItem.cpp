/*
 * ButtonMenuItem.cpp
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#include "ButtonMenuItem.h"

#if SUPPORT_DIRECT_LCD

ButtonMenuItem::ButtonMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, const char* t, const char* cmd, char const* acFile) noexcept
	: MenuItem(r, c, w, CentreAlign, fn), text(t), command(cmd), m_acFile(acFile)
{
}

void ButtonMenuItem::CorePrint(Lcd& lcd) noexcept
{
	lcd.WriteSpaces(1);				// space at start in case highlighted
	lcd.printf("%s", text);
	lcd.WriteSpaces(1);				// space at end to allow for highlighting
}

void ButtonMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept
{
	if (IsVisible() && (itemChanged || !drawn || highlight != highlighted) && column < lcd.GetNumCols())
	{
		highlighted = highlight;
		PrintAligned(lcd, rightMargin);
		itemChanged = false;
		drawn = true;
	}
}

void ButtonMenuItem::UpdateWidthAndHeight(Lcd& lcd) noexcept
{
	if (width == 0)
	{
		lcd.SetFont(fontNumber);
		lcd.SetCursor(lcd.GetNumRows(), 0);
		lcd.SetRightMargin(lcd.GetNumCols());
		lcd.TextInvert(false);
		CorePrint(lcd);
		width = lcd.GetColumn();
	}
	if (height == 0)
	{
		lcd.SetFont(fontNumber);
		height = lcd.GetFontHeight();
	}
}

// TODO WS1: if we overflow the command or directory string, we should probably offer a return value that tells the caller to do nothing...
bool ButtonMenuItem::Select(const StringRef& cmd) noexcept
{
	const int nReplacementIndex = StringContains(command, "#0");
	if (-1 != nReplacementIndex)
	{
		cmd.copy(command, nReplacementIndex);
		cmd.cat(m_acFile);
	}
	else
	{
		cmd.copy(command);
		if (StringEqualsIgnoreCase(command, "menu") && strlen(m_acFile) != 0)
		{
			// For backwards compatibility, if 'menu' is used without any parameters, use the L parameter as the name of the menu file
			cmd.cat(' ');
			cmd.cat(m_acFile);
		}
	}

	return true;
}

PixelNumber ButtonMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept
{
	PixelNumber tOffsetRequest = tCurrentOffset;

	// Are we off the bottom of the visible window?
	if (64 + tCurrentOffset <= row + fontHeight + 1)
	{
		tOffsetRequest = row - 3;
	}

	// Should we move back up?
	if (row < tCurrentOffset + 3)
	{
		tOffsetRequest = (row > 3) ? row - 3 : 0;
	}

	return tOffsetRequest;
}

#endif

// End
