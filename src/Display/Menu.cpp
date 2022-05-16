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
 *  image [Rnn] [Cnn] [Fnn] L"filename" *    							; display the image from "filename" at position RC
 *  text [Rnn] [Cnn] [Fnn] T"text" *    								; display non-selectable "text" at position RC
 *  button [Rnn] [Cnn] [Fnn] [Vnn] T"text" A"action" [L"filename"] *    ; display selectable "text" at RC, perform action when clicked
 *  value [Rnn] [Cnn] [Fnn] [Dnn] Wnnn Nvvv *   						; display the specified value at RC to the specified number of decimal places in the specified width
 *  alter [Rnn] [Cnn] [Fnn] [Dnn] Wnnn Nvvv *    						; display the specified value at RC to the specified number of decimal places in the specified width and allow it to be altered
 *  files [Rnn] [Fnn] Nnn I"initial-directory" A"action" [L"filename"] * ; display a list of files N lines high and allow them to be selected. The list uses the full width of the display.
 *
 *  Rnn is the row number for the top of the element measured in pixels from the top of the display
 *  Cnn is the column number for the left of the element measured in pixels from the left hand edge of the display
 *  Fnn is the font to use, 0=small 1=large
 *  Wnn is the width in pixels for the element
 *  Dnn specifies the number of decimal places for numeric display
 *
 *  Vnn specifies the item's visibility (currently implemented for buttons only) with value:
 *   0  always visible (default if not specified)
 *   2  visible when the printer is actively printing (actively printing defined as not paused, pausing or resuming)
 *   3  visible when the printer is NOT actively printing
 *   4  visible when the printer is printing (includes paused, pausing and resuming states)
 *   5  visible when the printer is NOT printing
 *   6  visible when the printer is printing and in paused state (paused or pausing)
 *   7  visible when the printer is printing and NOT in paused state (actively printing or resuming)
 *   10 visible when SD card 0 is mounted
 *   11 visible when SD card 0 is NOT mounted
 *   20 visible when the current or default tool has a temperature fault
 *   28 visible when the bed heater has a temperature fault
 *
 *  "action" can be any of:
 *  - a Gcode command string (must begin with G, M or T). In such a string, #0 represents the full name of the current file, in double quotes, set when a file is selected
 *  - "menu" (chains to the menu file given in the L parameter)
 *  - "popup" (pops up the menu given in the L parameter)
 *    NOTE: not currently implemented
 *  - "return" (returns to the parent menu)
 *  Multiple actions can be specified, separated by the vertical-bar character, e.g. "M32 #0|return|return|menu" but 'menu' may only be the last command
 *
 *  The N parameter in the "value" and "alter" commands specifies the value to display or change as follows:
 *  000-078		Tool N first heater current temperature e.g. 0 = tool 0 current temperature (display only)
 *  079			Currently selected tool first heater current temperature (display only)
 *  080-089		Bed heater (N-80) current temperature e.g. 80 = bed heater 0 current temperature (display only)
 *  090-099		Chamber heater (N-90) current temperature e.g. 90 = chamber heater 0 current temperature (display only)
 *  100-178		Tool (N-100) first heater active temperature e.g. 100 = tool 0 active temperature
 *  179         Currently selected tool first heater active temperature
 *  180-189		Bed heater (N-180) active temperature e.g. 180 = bed heater 0 active temperature
 *  190-199		Chamber heater (N-190) active temperature e.g. 190 = chamber heater 0 active temperature
 *  200-278		Tool (N-200) first heater standby temperature e.g. 200 = tool 0 standby temperature
 *  279         Currently selected tool first heater standby temperature
 *  280-289		Bed heater (N-280) standby temperature e.g. 280 = bed heater 0 standby temperature
 *  290-299		Chamber heater (N-290) standby temperature e.g. 290 = chamber heater 0 standby temperature
 *  300-398		Fan (N-300) percent full PWM e.g. 302 = fan 2 percent
 *  399			Current tool fan percent full PWM
 *  400-499		Extruder (N-400) extrusion factor
 *  500			Speed factor
 *  510-516		Current axis location (X, Y, Z, E0, E1, E2, E3 respectively) (display only)
 *  519			Z baby-step offset (display only)
 *  520			Currently selected tool number
 *  530-533		Actual IP address, octets 1 through 4
 */

#include "Menu.h"

#if SUPPORT_DIRECT_LCD

#include "TextMenuItem.h"
#include "ButtonMenuItem.h"
#include "ValueMenuItem.h"
#include "FilesMenuItem.h"
#include "ImageMenuItem.h"
#include "Lcd/Lcd.h"
#include "Display.h"
#include "TextMenuItem.h"
#include "ButtonMenuItem.h"
#include "ValueMenuItem.h"
#include "FilesMenuItem.h"
#include "ImageMenuItem.h"

#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

const uint32_t InactivityTimeout = 20000;		// inactivity timeout
const uint32_t ErrorTimeout = 6000;				// how long we display an error message for

Menu::Menu(Lcd& refLcd) noexcept
	: lcd(refLcd),
	  timeoutValue(0), lastActionTime(0),
	  selectableItems(nullptr), unSelectableItems(nullptr), highlightedItem(nullptr), numNestedMenus(0),
	  itemIsSelected(false), displayingFixedMenu(false), displayingErrorMessage(false), displayingMessageBox(false),
	  errorColumn(0), rowOffset(0)
{
}

Menu::~Menu()
{
	ResetCache();				// this releases the memory used by the menu items
}

void Menu::Load(const char* filename) noexcept
{
	if (numNestedMenus < MaxMenuNesting)
	{
		filenames[numNestedMenus].copy(filename);
		++numNestedMenus;
		rowOffset = 0;
		Reload();
	}
}

void Menu::LoadFixedMenu() noexcept
{
	displayingFixedMenu = true;
	numNestedMenus = 0;
	commandBufferIndex = 0;
	rowOffset = 0;
	currentMargin = 0;
	lcd.ClearAll();

	// Instead of Reload():
	lcd.SetRightMargin(lcd.GetNumCols() - currentMargin);

	ResetCache();

#if HAS_MASS_STORAGE
	char acLine1[] = "text R3 C5 F0 T\"No SD Card Found\"";
	char acLine2[] = "button R15 C5 F0 T\"Mount SD\" A\"M21\"";

	(void)ParseMenuLine(acLine1);
	(void)ParseMenuLine(acLine2);
#endif
}

// Display a M291 message box
void Menu::DisplayMessageBox(const MessageBox& mbox) noexcept
{
	ResetCache();
	displayingMessageBox = true;
	timeoutValue = 0;

	const PixelNumber topBottomMargin = 4;
	const PixelNumber sideMargin = 4;

	// Draw and a box and clear the interior
	const PixelNumber nr = lcd.GetNumRows(), nc = lcd.GetNumCols();
	lcd.SetRightMargin(nc);
	lcd.Line(topBottomMargin, sideMargin, topBottomMargin, nc - sideMargin - 1, true);
	lcd.Line(topBottomMargin, nc - sideMargin - 1, nr - topBottomMargin - 1, nc - sideMargin - 1, true);
	lcd.Line(nr - topBottomMargin - 1, sideMargin, nr - topBottomMargin - 1, nc - sideMargin - 1, true);
	lcd.Line(topBottomMargin, sideMargin, nr - topBottomMargin - 1, sideMargin, true);
	lcd.Clear(topBottomMargin + 1, sideMargin + 1, nr - topBottomMargin - 1, nc - sideMargin - 1);

	// We could draw the static text directly, but it is easier to use the existing classes
	const uint8_t fontToUse = 0;
	const PixelNumber insideMargin = 2;
	const PixelNumber rowHeight = lcd.GetFontHeight(fontToUse) + 1;
	const PixelNumber top = topBottomMargin + 1 + insideMargin;
	const PixelNumber left = sideMargin + 1 + insideMargin;
	const PixelNumber right = nc - left;
	const PixelNumber availableWidth = right - left;
	AddItem(new TextMenuItem(top, left, availableWidth, MenuItem::CentreAlign, fontToUse, mbox.title.c_str()), false);
	AddItem(new TextMenuItem(top + rowHeight, left, availableWidth, MenuItem::CentreAlign, fontToUse, mbox.message.c_str()), false);	// only 1 row for now

	// Add whichever XYZ jog buttons we have been asked to display - assume only XYZ for now
	const PixelNumber axisButtonWidth = availableWidth/4;
	const PixelNumber axisButtonStep = (availableWidth - 3 *axisButtonWidth)/2 + axisButtonWidth;
	if (mbox.controls.IsBitSet(X_AXIS))
	{
		AddItem(new ValueMenuItem(top + 2 * rowHeight, left, axisButtonWidth, MenuItem::CentreAlign, fontToUse, true, nullptr, 510, 1), true);
	}
	if (mbox.controls.IsBitSet(Y_AXIS))
	{
		AddItem(new ValueMenuItem(top + 2 * rowHeight, left + axisButtonStep, axisButtonWidth, MenuItem::CentreAlign, fontToUse, true, nullptr, 511, 1), true);
	}
	if (mbox.controls.IsBitSet(Z_AXIS))
	{
		AddItem(new ValueMenuItem(top + 2 * rowHeight, left + 2 * axisButtonStep, axisButtonWidth, MenuItem::CentreAlign, fontToUse, true, nullptr, 512, 2), true);
	}

	const PixelNumber okCancelButtonWidth = 30;
	if (mbox.mode & 2)
	{
		AddItem(new ButtonMenuItem(top + 3 * rowHeight, left, okCancelButtonWidth, fontToUse, "OK", "M292 P0", nullptr), true);
	}
	if (mbox.mode & 1)
	{
		AddItem(new ButtonMenuItem(top + 3 * rowHeight, right - okCancelButtonWidth, okCancelButtonWidth, fontToUse, "Cancel", "M292 P1", nullptr), true);
	}
}

// Clear the message box and display the menu underneath it
void Menu::ClearMessageBox() noexcept
{
	displayingMessageBox = false;
	Reload();
}

void Menu::Pop() noexcept
{
	--numNestedMenus;
	Reload();
}

void Menu::LoadError(const char *msg, unsigned int line) noexcept
{
	// Remove selectable items that may obscure view of the error message
	ResetCache();

	lcd.ClearAll();
	lcd.SetFont(0);
	lcd.printf("Error loading menu\nFile: %s", (numNestedMenus > 0) ? filenames[numNestedMenus - 1].c_str() : "(none)");
	if (line != 0)
	{
		lcd.printf("\nLine %u", line);
		if (errorColumn != 0)
		{
			lcd.printf(" column %u", errorColumn);
		}
	}
	lcd.printf("\n%s", msg);

	lastActionTime = millis();
	timeoutValue = ErrorTimeout;
	displayingErrorMessage = true;
}

// Parse a line in a menu layout file returning any error message, or nullptr if there was no error.
// Leading whitespace has already been skipped.
const char *Menu::ParseMenuLine(char * const commandWord) noexcept
{
	errorColumn = 0;

	// Check for blank or comment line
	if (*commandWord == ';' || *commandWord == 0)
	{
		return nullptr;
	}

	// Find the first word
	char *_ecv_array args = commandWord;
	while (isalpha(*args))
	{
		++args;
	}
	if (args == commandWord || (*args != ' ' && *args != '\t' && *args != 0))
	{
		errorColumn = (args - commandWord) + 1;
		return "Bad command";
	}

	if (*args != 0)
	{
		*args = 0;		// null terminate the command word
		++args;
	}

	// Parse the arguments
	const char *_ecv_array _ecv_null strVis = nullptr;
	MenuItem::Visibility xVis = MenuItem::AlwaysVisible;
	unsigned int decimals = 0;
	unsigned int nparam = 0;
	const char *_ecv_array _ecv_null strNparam = nullptr;
	unsigned int width = 0;
	unsigned int alignment = 0;
	const char *text = "*";
	const char *fname = "main";
	const char *dirpath = "";
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
			row = StrToU32(args, &args) + rowOffset;
			break;

		case 'C':
			column = StrToU32(args, &args) + currentMargin;
			break;

		case 'F':
			fontNumber = min<unsigned int>(StrToU32(args, &args), lcd.GetNumFonts() - 1);
			break;

		case 'V':
			if (*args == '{')
			{
				++args;
				strVis = args;
				while (*args != '}' && *args != 0)
				{
					++args;
				}
				if (*args == '}')
				{
					*args = 0;
					++args;
				}
			}
			else
			{
				xVis = StrToU32(args, &args);
			}
			break;

		case 'D':
			decimals = StrToU32(args, &args);
			break;

		case 'N':
			// 'value' command allows the N parameter to be an object mode string
			if (*args == '{' && StringEqualsIgnoreCase(commandWord, "value"))
			{
				strNparam = args;
				while (*args != '}' && *args != 0)
				{
					++args;
				}
				if (*args == '}')
				{
					*args = 0;
					++args;
				}
			}
			else
			{
				nparam = StrToU32(args, &args);
			}
			break;

		case 'W':
			width = StrToU32(args, &args);
			break;

		case 'H':
			alignment = StrToU32(args, &args);
			break;

		case '"':			// a string with no letter is a T argument
			ch = 'T';
			--args;
			// no break
		case 'T':
		case 'L':
		case 'A':
		case 'I':
			if (*args != '"')
			{
				errorColumn = (args - commandWord) + 1;
				return "Missing string arg";
			}
			++args;
			((ch == 'T') ? text : (ch == 'A') ? action : (ch == 'I') ? dirpath : fname) = args;
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
			errorColumn = (args - commandWord);
			return "Bad arg letter";
		}
	}

	MenuItem *_ecv_null newItem = nullptr;

	// Create an object resident in memory corresponding to the menu layout file's description
	if (StringEqualsIgnoreCase(commandWord, "text"))
	{
		const char *_ecv_array const acText = AppendString(text);
		newItem = new TextMenuItem(row, column, width, alignment, fontNumber, acText);
		AddItem(newItem, false);
		column += newItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "image") && fname != nullptr)
	{
		newItem = new ImageMenuItem(row, column,fname);
		AddItem(newItem, false);
		column += newItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "button"))
	{
		const char *_ecv_array const textString = AppendString(text);
		const char *_ecv_array const actionString = AppendString(action);
		const char *_ecv_array const c_acFileString = AppendString(fname);
		newItem = new ButtonMenuItem(row, column, width, fontNumber, textString, actionString, c_acFileString);
		AddItem(newItem, true);
		column += newItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "value"))
	{
		newItem = new ValueMenuItem(row, column, width, alignment, fontNumber, false, strNparam, nparam, decimals);
		AddItem(newItem, false);
		column += newItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "alter"))
	{
		newItem = new ValueMenuItem(row, column, width, alignment, fontNumber, true, nullptr, nparam, decimals);
		AddItem(newItem, true);
		column += newItem->GetWidth();
	}
#if HAS_MASS_STORAGE
	else if (StringEqualsIgnoreCase(commandWord, "files"))
	{
		const char *_ecv_array const actionString = AppendString(action);
		const char *_ecv_array const dir = AppendString(dirpath);
		const char *_ecv_array const acFileString = AppendString(fname);
		newItem = new FilesMenuItem(row, 0, lcd.GetNumCols(), fontNumber, actionString, dir, acFileString, nparam);
		AddItem(newItem, true);
		row += nparam * lcd.GetFontHeight(fontNumber);
		column = currentMargin;
	}
#endif
	else
	{
		errorColumn = 1;
		return "Unknown command";
	}

	// Deal with the visibility of the newly-added menu item
	if (strVis != nullptr)
	{
		newItem->SetVisibility(strVis);
	}
	else
	{
		newItem->SetVisibility(xVis);
	}
	return nullptr;
}

void Menu::ResetCache() noexcept
{
	highlightedItem = nullptr;

	// Delete the existing items
	while (selectableItems != nullptr)
	{
		MenuItem * const current = selectableItems;
		selectableItems = selectableItems->GetNext();
		delete current;
	}
	while (unSelectableItems != nullptr)
	{
		MenuItem * const current = unSelectableItems;
		unSelectableItems = unSelectableItems->GetNext();
		delete current;
	}
}

void Menu::Reload() noexcept
{
	displayingFixedMenu = false;

#if 0	// if all menus use the whole screen (no visual nesting)
	currentMargin = rowOffset = 0;
	lcd.ClearAll();
#else
	if (numNestedMenus == 1)
	{
		currentMargin = 0;
		lcd.ClearAll();
	}
	else
	{
		constexpr PixelNumber indentPerLevel = 10;		//TODO make this depend on the screen resolution
		currentMargin = rowOffset = indentPerLevel * (numNestedMenus - 1);
		const PixelNumber borderMargin = currentMargin - 2;
		const PixelNumber right = lcd.GetNumCols() - borderMargin - 1;
		const PixelNumber bottom = lcd.GetNumRows() - borderMargin - 1;
		lcd.Clear(borderMargin, borderMargin, bottom, right);

		// Draw the outline
		lcd.Line(borderMargin, borderMargin, bottom, borderMargin, true);
		lcd.Line(borderMargin, borderMargin, borderMargin, right, true);
		lcd.Line(bottom, borderMargin, bottom, right, true);
		lcd.Line(borderMargin, right, bottom, right, true);
	}
#endif

	ResetCache();
	displayingErrorMessage = false;

	lcd.SetRightMargin(lcd.GetNumCols() - currentMargin);
	const char *_ecv_array const fname = filenames[numNestedMenus - 1].c_str();
	FileStore *_ecv_null const file = reprap.GetPlatform().OpenFile(MENU_DIR, fname, OpenMode::read);
	if (file == nullptr)
	{
		LoadError("File not found", 0);
	}
	else
	{
		row = rowOffset;
		column = currentMargin;
		fontNumber = 0;
		commandBufferIndex = 0;						// Free the string buffer, which contains layout elements from an old menu
		for (unsigned int line = 1; ; ++line)
		{
			char buffer[MaxMenuLineLength];
			if (file->ReadLine(buffer, sizeof(buffer)) < 0)
			{
				break;
			}
			char * const pcMenuLine = SkipWhitespace(buffer);
			const char * const errMsg = ParseMenuLine(pcMenuLine);
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

		file->Close();
	}
}

void Menu::AddItem(MenuItem *item, bool isSelectable) noexcept
{
	item->UpdateWidthAndHeight(lcd);
	MenuItem::AppendToList((isSelectable) ? &selectableItems : &unSelectableItems, item);
}

// Append a string to the string buffer and return its index
const char *Menu::AppendString(const char *s) noexcept
{
	// TODO: hold a fixed reference to '\0' -- if any strings passed in are empty, return this reference
	const size_t oldIndex = commandBufferIndex;
	if (commandBufferIndex < sizeof(commandBuffer))
	{
		SafeStrncpy(commandBuffer + commandBufferIndex, s, ARRAY_SIZE(commandBuffer) - commandBufferIndex);
		commandBufferIndex += strlen(commandBuffer + commandBufferIndex) + 1;
	}
	return commandBuffer + oldIndex;
}

// TODO: there is no error handling if a command within a sequence cannot be accepted...
void Menu::EncoderAction_ExecuteHelper(const char *const cmd) noexcept
{
	if (StringEqualsIgnoreCase(cmd, "return"))
	{
		Pop();										// up one level
	}
	else if (StringStartsWithIgnoreCase(cmd, "menu "))
	{
		Load(cmd + 5);
	}
	else if (toupper(cmd[0]) == 'G' || toupper(cmd[0]) == 'M' || toupper(cmd[0]) == 'T')
	{
		const bool success = reprap.GetGCodes().ProcessCommandFromLcd(cmd);
		if (!success)
		{
			reprap.GetDisplay().ErrorBeep();			// long low beep
		}
	}
}

void Menu::EncoderActionEnterItemHelper() noexcept
{
	if (highlightedItem != nullptr && highlightedItem->IsVisible())
	{
		String<MaxFilenameLength> command;
		if (highlightedItem->Select(command.GetRef()))
		{
			char *pcCurrentCommand = command.GetRef().Pointer();
			char *pcNextCommand;
			while ((pcNextCommand = strchr(pcCurrentCommand, '|')) != nullptr)
			{
				*pcNextCommand = '\0';
				EncoderAction_ExecuteHelper(pcCurrentCommand);
				pcCurrentCommand = pcNextCommand + 1;
			}
			EncoderAction_ExecuteHelper(pcCurrentCommand);
		}
		else if (highlightedItem->CanAdjust())
		{
			itemIsSelected = true;
		}
	}
}

void Menu::EncoderActionScrollItemHelper(int action) noexcept
{
	// Based mainly on file listing requiring we handle list of unknown length
	// before moving on to the next selectable item at the Menu level, we let the
	// currently selected MenuItem try to handle the scroll action itself.  It will
	// return the remainder of the scrolling that it was unable to accommodate.
	if (highlightedItem != nullptr && highlightedItem->IsVisible())
	{
		// Let the current menu item attempt to handle scroll wheel first
		action = highlightedItem->Advance(action);
	}

	if (action != 0)
	{
		// Otherwise we move through the remaining selectable menu items
		AdvanceHighlightedItem(action);

		if (highlightedItem != nullptr)
		{
			// Let the newly selected MenuItem handle any selection setup
			highlightedItem->Enter(action > 0);

			PixelNumber tLastOffset = rowOffset;
			rowOffset = highlightedItem->GetVisibilityRowOffset(tLastOffset, lcd.GetFontHeight(highlightedItem->GetFontNumber()));

			if (rowOffset != tLastOffset)
			{
				// We have scrolled the whole menu, so redraw it
				lcd.ClearAll();
				for (MenuItem *item = selectableItems; item != nullptr; item = item->GetNext())
				{
					item->SetChanged();
				}
				for (MenuItem *item = unSelectableItems; item != nullptr; item = item->GetNext())
				{
					item->SetChanged();
				}
			}
		}
	}
}

// Perform the specified encoder action
// If 'action' is zero then the button was pressed, else 'action' is the number of clicks (+ve for clockwise)
// EncoderAction is what's called in response to all wheel/button actions; a convenient place to set new timeout values
void Menu::EncoderAction(int action) noexcept
{
	if (displayingErrorMessage)
	{
		// Allow the message to be cancelled by a push
		if (action == 0)
		{
			timeoutValue = 1;					// cancel the timeout at the next tick
		}
	}
	else
	{
		if (itemIsSelected)						// send the wheel action (scroll or click) to the item itself
		{
			if (highlightedItem != nullptr && highlightedItem->IsVisible())
			{
				const bool done = highlightedItem->Adjust(action);
				if (done)
				{
					itemIsSelected = false;
				}
			}
			else
			{
				itemIsSelected = false;			// should not get here
			}
		}
		else if (action != 0)					// scroll without an item under selection
		{
			EncoderActionScrollItemHelper(action);
		}
		else									// click without an item under selection
		{
			EncoderActionEnterItemHelper();
		}

		if (!displayingErrorMessage && !displayingMessageBox)	// if the operation did not result in an error and we are not displaying a message box
		{
			lastActionTime = millis();
			timeoutValue = InactivityTimeout;
		}
	}
}

/*static*/ const char *Menu::SkipWhitespace(const char *s) noexcept
{
	while (*s == ' ' || *s == '\t')
	{
		++s;
	}
	return s;
}

/*static*/ char *Menu::SkipWhitespace(char *s) noexcept
{
	while (*s == ' ' || *s == '\t')
	{
		++s;
	}
	return s;
}

// Refresh is called every Spin() of the Display under most circumstances; an appropriate place to check if timeout action needs to be taken
void Menu::Refresh() noexcept
{
	if (
#if HAS_SBC_INTERFACE
		!reprap.UsingSbcInterface() &&
#endif
#if HAS_MASS_STORAGE
		!MassStorage::IsDriveMounted(0)
#else
		true	// When there is no mass storage drives cannot be mounted anyway and the above equals true
#endif
	   )
	{
		if (!displayingFixedMenu)
		{
			LoadFixedMenu();						// when the SD card is not mounted, we show a fixed menu for graceful recovery
		}
	}
	else if (displayingFixedMenu || (timeoutValue != 0 && (millis() - lastActionTime > timeoutValue)))
	{
		// Showing fixed menu but SD card is now mounted, or 6 seconds following latest user action
		// Go to the top menu (just discard information)
		timeoutValue = 0;
		numNestedMenus = 0;
		Load("main");
	}
	DrawAll();
}

void Menu::DrawAll() noexcept
{
	// First erase any displayed items that should now be invisible
	for (MenuItem *item = selectableItems; item != nullptr; item = item->GetNext())
	{
		item->EraseIfInvisible(lcd, rowOffset);
	}

	for (MenuItem *item = unSelectableItems; item != nullptr; item = item->GetNext())
	{
		item->EraseIfInvisible(lcd, rowOffset);
	}

	// Now draw items
	const PixelNumber rightMargin = lcd.GetNumCols() - currentMargin;
	for (MenuItem *item = selectableItems; item != nullptr; item = item->GetNext())
	{
		item->Draw(lcd, rightMargin, (item == highlightedItem));
	}

	for (MenuItem *item = unSelectableItems; item != nullptr; item = item->GetNext())
	{
		item->Draw(lcd, rightMargin, false);
	}
}

// Clear any highlighting. Call Refresh() after calling this to clear it on the display.
void Menu::ClearHighlighting() noexcept
{
	highlightedItem = nullptr;
	itemIsSelected = false;
}

// Move the highlighted item forwards or backwards through the selectable items, setting it to nullptr if nothing is selectable
// On entry, n is nonzero
void Menu::AdvanceHighlightedItem(int n) noexcept
{
	if (highlightedItem == nullptr)
	{
		// No item is selected, so pick the first selectable item
		highlightedItem = FindNextSelectableItem(nullptr);
	}
	else if (n > 0)
	{
		for (;;)
		{
			MenuItem * const p = FindNextSelectableItem(highlightedItem);
			if (n == 0 || p == nullptr || p == highlightedItem)
			{
				highlightedItem = p;
				return;
			}
			--n;
		}
	}
	else
	{
		for (;;)
		{
			MenuItem * const p = FindPrevSelectableItem(highlightedItem);
			if (n == 0 || p == nullptr || p == highlightedItem)
			{
				highlightedItem = p;
				return;
			}
			++n;
		}
	}
}

// Find the next selectable item, or the first one if nullptr is passed in
// Note, there may be no selectable items, or there may be just one
MenuItem *Menu::FindNextSelectableItem(MenuItem *p) const noexcept
{
	if (selectableItems == nullptr)
	{
		return nullptr;
	}

	MenuItem * initial = (p == nullptr || p->GetNext() == nullptr) ? selectableItems : p->GetNext();
	MenuItem * current = initial;							// set search start point
	do
	{
		if (current->IsVisible())
		{
			return current;
		}
		current = current->GetNext();
		if (current == nullptr)
		{
			current = selectableItems;
		}
	} while (current != initial);
	return nullptr;
}

// Find the previous selectable item, or the last one if nullptr is passed in
// Note, there may be no selectable items, and the one we pass in may not be selectable
MenuItem *Menu::FindPrevSelectableItem(MenuItem *p) const noexcept
{
	if (selectableItems == nullptr)
	{
		return nullptr;
	}

	MenuItem * const initial = (p == nullptr) ? selectableItems : p;	// set search start point
	MenuItem * current = initial;
	MenuItem * best = nullptr;
	do
	{
		if (current->IsVisible())
		{
			best = current;
		}
		current = current->GetNext();
		if (current == nullptr)
		{
			current = selectableItems;
		}
	} while (current != initial);
	return best;
}

#if SUPPORT_RESISTIVE_TOUCH

// Search for a selectable item that is close to the touched XY coordinates
void Menu::HandleTouch(PixelNumber x, PixelNumber y) noexcept
{
	constexpr int MaxXerror = 8, MaxYerror = 8;		// how far (in pixels) the X and Y coordinates of a touch event need to be to the outline of the item for us to disallow it

	if (displayingErrorMessage)
	{
		// Allow the message to be cancelled by a touch
		TouchBeep();
		timeoutValue = 1;						// cancel the timeout at the next tick
	}
	else
	{
		int bestError = MaxXerror + MaxYerror;
		MenuItem *null best = nullptr;
		for (MenuItem *p = selectableItems; p != nullptr; p = p->GetNext())
		{
			if (p->IsVisible())
			{
				const int xError = (x < p->GetMinX()) ? p->GetMinX() - x
										: (x > p->GetMaxX()) ? x - p->GetMaxX()
											: 0;
				if (xError < MaxXerror)
				{
					const int yError = (y <p->GetMinY()) ? p->GetMinY() - y
											: (y > p->GetMaxY()) ? y - p->GetMaxY()
												: 0;
					if (yError < MaxYerror && xError + yError < bestError)
					{
						bestError = xError + yError;
						best = p;
					}
				}
			}
		}

		if (best != nullptr)
		{
			TouchBeep();
			if (itemIsSelected)					// send the touch to the item itself
			{
				if (highlightedItem != nullptr && highlightedItem->IsVisible())
				{
					//TODO
				}
				else
				{
					itemIsSelected = false;			// should not get here
				}
			}
			else									// click without an item under selection
			{
				highlightedItem = best;
				EncoderActionEnterItemHelper();
			}

			if (!displayingErrorMessage && !displayingMessageBox)	// if the operation did not result in an error and we are not displaying a message box
			{
				lastActionTime = millis();
				timeoutValue = InactivityTimeout;
			}
		}
	}
}

/*static*/ void Menu::TouchBeep() noexcept
{
	reprap.GetDisplay().Beep(TouchBeepFrequency, TouchBeepLength);
}

#endif

#endif

// End
