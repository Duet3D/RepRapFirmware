/*
 * Menu.cpp
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 *
 *  Menus are read from files in the /menu folder of the SD card. the root menu is called 'main'.
 *  Each menu file holds a sequence of commands, one per line.
 *  The following commands are supported:
 *
 *  image [Rnn] [Cnn] [Fnn] L"filename"							; display the image from "filename" at position RC
 *  text [Rnn] [Cnn] [Fnn] T"text"								; display non-selectable "text" at position RC
 *  button [Rnn] [Cnn] [Fnn] T"text" A"action" [L"filename"]	; display selectable "text" at RC, perform action when clicked
 *  value [Rnn] [Cnn] [Fnn] [Dnn] Wnnn Nvvv						; display the specified value at RC to the specified number of decimal places in the specified width
 *  alter [Rnn] [Cnn] [Fnn] [Dnn] Wnnn Nvvv						; display the specified value at RC to the specified number of decimal places in the specified width and allow it to be altered
 *  files [Rnn] [Fnn] Nnn L"initial-directory" A"action"		; display a list of files N lines high and allow them to be selected. The list uses the full width of the display.
 *
 *  Rnn is the row number for the top of the element measured in pixels from the top of the display
 *  Cnn is the column number for the left of the element measured in pixels from the left hand edge of the display
 *  Fnn is the font to use, 0=small 1=large
 *  Wnn is the width in pixels for the element
 *
 *  "action" can be any of:
 *  - a Gcode command string (must begin with G, M or T). In such a string, #0 represents the full name of the current file, in double quotes, set when a file is selected
 *  - "menu" (chains to the menu file given in the L parameter)
 *  - "popup" (pops up the menu given in the L parameter)
 *  - "return" (returns to the parent menu)
 *  Multiple actions can be specified, separated by the vertical-bar character, e.g. "M32 #0|return|return|menu" but 'menu' may only be the last command
 *
 *  The N parameter in the "value" and "alter" commands specifies the value to display or change as follows:
 *  000-079		Tool N first heater current temperature e.g. 0 = tool 0 current temperature (display only)
 *  080-089		Bed heater (N-80) current temperature e.g. 80 = bed heater 0 current temperature (display only)
 *  090-099		Chamber heater (N-90) current temperature e.g. 90 = chamber heater 0 current temperature (display only)
 *  100-179		Tool (N-100) first heater active temperature e.g. 100 = tool 0 active temperature
 *  180-189		Bed heater (N-180) active temperature e.g. 180 = bed heater 0 active temperature
 *  190-199		Chamber heater (N-190) active temperature e.g. 190 = chamber heater 0 active temperature
 *  200-279		Tool (N-200) first heater standby temperature e.g. 200 = tool 0 standby temperature
 *  280-289		Bed heater (N-280) standby temperature e.g. 280 = bed heater 0 standby temperature
 *  290-299		Chamber heater (N-290) standby temperature e.g. 290 = chamber heater 0 standby temperature
 *  300-398		Fan (N-300) percent full PWM e.g. 302 = fan 2 percent
 *  399			Current tool fan percent full PWM
 *  400-499		Extruder (N-400) extrusion factor
 *  500			Speed factor
 */

#include "Menu.h"
#include "ST7920/lcd7920.h"
#include "RepRap.h"
#include "Platform.h"
#include "Storage/MassStorage.h"
#include "GCodes/GCodes.h"
#include "Display/Display.h"

Menu::Menu(Lcd7920& refLcd, const LcdFont * const fnts[], size_t nFonts)
	: lcd(refLcd), fonts(fnts), numFonts(nFonts),
	  selectableItems(nullptr), unSelectableItems(nullptr), numNestedMenus(0), numSelectableItems(0), highlightedItem(0), itemIsSelected(false)
{
}

void Menu::Load(const char* filename)
{
	if (numNestedMenus < MaxMenuNesting)
	{
		filenames[numNestedMenus].copy(filename);

		if (numNestedMenus == 0)
		{
			currentMargin = 0;
			lcd.Clear(0, 0, NumRows, NumCols);
		}
		else
		{
			currentMargin = numNestedMenus * (OuterMargin + InnerMargin) - InnerMargin;
			const PixelNumber right = NumCols - currentMargin;
			const PixelNumber bottom = NumRows - currentMargin;
			lcd.Clear(currentMargin, currentMargin, bottom, right);

			// Draw the outline
			lcd.Line(currentMargin, currentMargin, bottom, currentMargin, PixelMode::PixelSet);
			lcd.Line(currentMargin, currentMargin, currentMargin, right, PixelMode::PixelSet);
			lcd.Line(bottom, currentMargin, bottom, right, PixelMode::PixelSet);
			lcd.Line(currentMargin, right, bottom, right, PixelMode::PixelSet);

			currentMargin += InnerMargin;
		}

		++numNestedMenus;
		Reload();
	}
}

void Menu::Pop()
{
}

void Menu::LoadError(const char *msg, unsigned int line)
{
	lcd.Clear(currentMargin, currentMargin, NumRows - currentMargin, NumCols - currentMargin);
	lcd.SetFont(fonts[0]);
	lcd.print("Error loading menu\nFile ");
	lcd.print(filenames[numNestedMenus - 1].c_str());
	if (line != 0)
	{
		lcd.print("\nLine ");
		lcd.print(line);
	}
	lcd.write('\n');
	lcd.print(msg);

	if (numNestedMenus > 1)
	{
		// TODO add control to pop previous menu here, or revert to main menu after some time
	}
}

// Parse a command returning the error message, or nullptr if there was no error.
// If numCommandArguments is nonzero on entry, don't execute the command and leave numCommandArguments unchanged.
// if numCommandArguments is zero on entry, execute the command, and set numCommandArguments to the number of following argument lines.
// Leading whitespace has already been skipped.
const char *Menu::ParseCommand(char *commandWord)
{
	// Check for blank or comment line
	if (*commandWord == ';' || *commandWord == 0)
	{
		return nullptr;
	}

	// Find the first word
	char *args = commandWord;
	while (isalpha(*args))
	{
		++args;
	}
	if (args == commandWord || (*args != ' ' && *args != '\t' && *args != 0))
	{
		return "Bad command";
	}

	if (*args != 0)
	{
		*args = 0;		// null terminate command word
		++args;
	}

	// Parse the arguments
	unsigned int decimals = 0;
	unsigned int nparam = 0;
	unsigned int width = DefaultNumberWidth;
	const char *text = "*";
	const char *fname = "main";
	const char *action = nullptr;

	while (*args != 0 && *args != ';')
	{
		char ch;
		switch (ch = toupper(*args++))
		{
		case ' ':
		case '\t':
			break;

		case 'R':
			row = SafeStrtoul(args, &args);
			break;

		case 'C':
			column = SafeStrtoul(args, &args);
			break;

		case 'F':
			fontNumber = min<unsigned int>(SafeStrtoul(args, &args), numFonts - 1);
			break;

		case 'D':
			decimals = SafeStrtoul(args, &args);
			break;

		case 'N':
			nparam = SafeStrtoul(args, &args);
			break;

		case 'W':
			width = SafeStrtoul(args, &args);
			break;

		case 'T':
		case 'L':
		case 'A':
			if (*args != '"')
			{
				return "Missing string arg";
			}
			++args;
			((ch == 'T') ? text : (ch == 'A') ? action : fname) = args;
			while (*args != '"' && *args != 0)
			{
				++args;
			}
			if (*args == '"')
			{
				*args = 0;
				++args;
			}
			break;

		default:
			return "Bad arg letter";
		}
	}

	lcd.SetCursor(row + currentMargin, column + currentMargin);

	// Look up and execute the command
	if (StringEquals(commandWord, "text"))
	{
		lcd.SetFont(fonts[fontNumber]);
		lcd.print(text);
		row = lcd.GetRow() - currentMargin;
		column = lcd.GetColumn() - currentMargin;
	}
	else if (StringEquals(commandWord, "image") && fname != nullptr)
	{
		LoadImage(fname);
	}
	else if (StringEquals(commandWord, "button"))
	{
		const char * const textString = AppendString(text);
		const char * const actionString = AppendString(action);
		AddItem(new ButtonMenuItem(row, column, fontNumber, textString, actionString), true);
		// Print the button as well so that we can update the row and column
		lcd.SetFont(fonts[fontNumber]);
		lcd.print(text);
		row = lcd.GetRow() - currentMargin;
		column = lcd.GetColumn() - currentMargin;
	}
	else if (StringEquals(commandWord, "value"))
	{
		AddItem(new ValueMenuItem(row, column, fontNumber, width, nparam, decimals), false);
		column += width;
	}
	else if (StringEquals(commandWord, "alter"))
	{
		AddItem(new ValueMenuItem(row, column, fontNumber, width, nparam, decimals), true);
		column += width;
	}
	else if (StringEquals(commandWord, "files"))
	{
		const char * const actionString = AppendString(action);
		const char * const dir = AppendString(fname);
		AddItem(new FilesMenuItem(row, column, fontNumber, actionString, dir, nparam), true);
		//TODO update row by a sensible value e.g. nparam * text row height
		column = 0;
	}
	else
	{
		return "Unknown command";
	}

	return nullptr;
}

void Menu::Reload()
{
	// Delete the existing items
	while (selectableItems != nullptr)
	{
		MenuItem *current = selectableItems;
		selectableItems = selectableItems->GetNext();
		delete current;
	}
	while (unSelectableItems != nullptr)
	{
		MenuItem *current = unSelectableItems;
		unSelectableItems = unSelectableItems->GetNext();
		delete current;
	}
	numSelectableItems = highlightedItem = 0;

	lcd.SetRightMargin(NumCols - currentMargin);
	const char * const fname = filenames[numNestedMenus - 1].c_str();
	FileStore * const file = reprap.GetPlatform().OpenFile(MENU_DIR, fname, OpenMode::read);
	if (file == nullptr)
	{
		LoadError("Can't open menu file", 0);
	}
	else
	{
#if 0
		lcd.print("Menu");
		lcd.SetCursor(currentMargin + lcd.GetFontHeight() + 1, currentMargin);
		lcd.print(fname);
#else
		row = 0;
		column = 0;
		fontNumber = 0;
		commandBufferIndex = 0;
		for (unsigned int line = 1; ; ++line)
		{
			char buffer[MaxMenuLineLength];
			if (file->ReadLine(buffer, sizeof(buffer)) <= 0)
			{
				break;
			}
			char * const commandLine = SkipWhitespace(buffer);
			const char * const errMsg = ParseCommand(commandLine);
			if (errMsg != nullptr)
			{
				LoadError(errMsg, line);
				break;
			}

			// Check for string buffer full
			if (commandBufferIndex == sizeof(commandBuffer))
			{
				LoadError("|Menu buffer full", line);
				break;
			}
		}
#endif
		file->Close();
		Refresh();
	}
}

void Menu::AddItem(MenuItem *item, bool isSelectable)
{
	MenuItem::AppendToList((isSelectable) ? &selectableItems : &unSelectableItems, item);
	if (isSelectable)
	{
		++numSelectableItems;
	}
}

// Append a string to the string buffer and return its index
const char *Menu::AppendString(const char *s)
{
	const size_t oldIndex = commandBufferIndex;
	if (commandBufferIndex < sizeof(commandBuffer))
	{
		SafeStrncpy(commandBuffer + commandBufferIndex, s, sizeof(commandBuffer) - commandBufferIndex);
		commandBufferIndex += strlen(commandBuffer + commandBufferIndex) + 1;
	}
	return commandBuffer + oldIndex;
}

// Perform the specified encoder action
// If 'action' is zero then the button was pressed, else 'action' is the number of clicks (+ve for clockwise)
void Menu::EncoderAction(int action)
{
	if (numSelectableItems != 0)
	{
		if (itemIsSelected)
		{
			MenuItem * const item = FindHighlightedItem();
			if (item != nullptr)
			{
				const bool done = item->Adjust(action);
				if (done)
				{
					itemIsSelected = false;
				}
			}
			else
			{
				// Should not get here
				itemIsSelected = false;
			}
		}
		else if (action != 0)
		{
			highlightedItem += action;
			while (highlightedItem < 0)
			{
				highlightedItem += numSelectableItems;
			}
			while (highlightedItem >= numSelectableItems)
			{
				highlightedItem -= numSelectableItems;
			}
		}
		else
		{
			MenuItem * const item = FindHighlightedItem();
			if (item != nullptr)
			{
				const char * const cmd = item->Select();
				if (cmd != nullptr)
				{
					if (cmd[0] == 'G' || cmd[0] == 'M' || cmd[0] == 'T')
					{
						const bool success = reprap.GetGCodes().ProcessCommandFromLcd(cmd);
						if (success)
						{
							reprap.GetDisplay().SuccessBeep();
						}
						else
						{
							reprap.GetDisplay().ErrorBeep();			// long low beep
						}
					}
					else
					{
						//TODO run the command (popup, menu, return)
					}
				}
				else
				{
					itemIsSelected = true;
				}
			}
		}
	}
}

/*static*/ const char *Menu::SkipWhitespace(const char *s)
{
	while (*s == ' ' || *s == '\t')
	{
		++s;
	}
	return s;
}

/*static*/ char *Menu::SkipWhitespace(char *s)
{
	while (*s == ' ' || *s == '\t')
	{
		++s;
	}
	return s;
}

void Menu::LoadImage(const char *fname)
{
	//TODO
	lcd.print("<image>");
}

void Menu::Refresh()
{
	const PixelNumber rightMargin = NumCols - currentMargin;
	int currentItem = 0;
	for (MenuItem *item = selectableItems; item != nullptr; item = item->GetNext())
	{
		lcd.SetFont(fonts[item->GetFontNumber()]);
		item->Draw(lcd, rightMargin, currentItem == highlightedItem);
		++currentItem;
	}
	for (MenuItem *item = unSelectableItems; item != nullptr; item = item->GetNext())
	{
		lcd.SetFont(fonts[item->GetFontNumber()]);
		item->Draw(lcd, rightMargin, false);
		++currentItem;
	}
}

MenuItem *Menu::FindHighlightedItem() const
{
	MenuItem *p = selectableItems;
	for (int n = highlightedItem; n > 0 && p != nullptr; --n)
	{
		p = p->GetNext();
	}
	return p;
}

// End
