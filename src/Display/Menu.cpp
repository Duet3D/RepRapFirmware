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

#if SUPPORT_12864_LCD

#include "ST7920/lcd7920.h"
#include "RepRap.h"
#include "Platform.h"
#include "Display/Display.h"
#include "GCodes/GCodes.h"
#include "Heating/Heat.h"
#include "Storage/MassStorage.h"
#include "Tools/Tool.h"

const uint32_t InactivityTimeout = 20000;		// inactivity timeout
const uint32_t ErrorTimeout = 6000;				// how long we display an error message for

Menu::Menu(Lcd7920& refLcd)
	: lcd(refLcd),
	  timeoutValue(0), lastActionTime(0),
	  selectableItems(nullptr), unSelectableItems(nullptr), highlightedItem(nullptr), numNestedMenus(0),
	  itemIsSelected(false), displayingFixedMenu(false), displayingErrorMessage(false), displayingMessageBox(false),
	  errorColumn(0), rowOffset(0)
{
}

void Menu::Load(const char* filename)
{
	if (numNestedMenus < MaxMenuNesting)
	{
		filenames[numNestedMenus].copy(filename);
		++numNestedMenus;
		rowOffset = 0;
		Reload();
	}
}

void Menu::LoadFixedMenu()
{
	displayingFixedMenu = true;
	numNestedMenus = 0;
	rowOffset = 0;
	currentMargin = 0;
	lcd.Clear();

	// Instead of Reload():
	lcd.SetRightMargin(NumCols - currentMargin);

	ResetCache();

	char acLine1[] = "text R3 C5 F0 T\"No SD Card Found\"";
	char acLine2[] = "button R15 C5 F0 T\"Mount SD\" A\"M21\"";

	const char *errMsg = ParseMenuLine(acLine1);
	if (nullptr != errMsg)
	{
		LoadError(errMsg, 1);
	}
	if (commandBufferIndex == sizeof(commandBuffer))
	{
		LoadError("|Menu buffer full", 1);
	}

	errMsg = ParseMenuLine(acLine2);
	if (nullptr != errMsg)
	{
		LoadError(errMsg, 2);
	}
	if (commandBufferIndex == sizeof(commandBuffer))
	{
		LoadError("|Menu buffer full", 2);
	}
}

// Display a M291 message box
void Menu::DisplayMessageBox(const MessageBox& mbox)
{
	ResetCache();
	displayingMessageBox = true;
	timeoutValue = 0;

	const PixelNumber topBottomMargin = 4;
	const PixelNumber sideMargin = 4;

	// Draw and a box and clear the interior
	lcd.SetRightMargin(NumCols);
	lcd.Line(topBottomMargin, sideMargin, topBottomMargin, NumCols - sideMargin - 1, PixelMode::PixelSet);
	lcd.Line(topBottomMargin, NumCols - sideMargin - 1, NumRows - topBottomMargin - 1, NumCols - sideMargin - 1, PixelMode::PixelSet);
	lcd.Line(NumRows - topBottomMargin - 1, sideMargin, NumRows - topBottomMargin - 1, NumCols - sideMargin - 1, PixelMode::PixelSet);
	lcd.Line(topBottomMargin, sideMargin, NumRows - topBottomMargin - 1, sideMargin, PixelMode::PixelSet);
	lcd.Clear(topBottomMargin + 1, sideMargin + 1, NumRows - topBottomMargin - 1, NumCols - sideMargin - 1);

	// We could draw the static text directly, but it is easier to use the existing classes
	const uint8_t fontToUse = 0;
	const PixelNumber insideMargin = 2;
	const PixelNumber rowHeight = lcd.GetFontHeight(fontToUse) + 1;
	const PixelNumber top = topBottomMargin + 1 + insideMargin;
	const PixelNumber left = sideMargin + 1 + insideMargin;
	const PixelNumber right = NumCols - left;
	const PixelNumber availableWidth = right - left;
	AddItem(new TextMenuItem(top, left, availableWidth, MenuItem::CentreAlign, fontToUse, MenuItem::AlwaysVisible, mbox.title.c_str()), false);
	AddItem(new TextMenuItem(top + rowHeight, left, availableWidth, MenuItem::CentreAlign, fontToUse, MenuItem::AlwaysVisible, mbox.message.c_str()), false);	// only 1 row for now

	// Add whichever XYZ jog buttons we have been asked to display - assume only XYZ for now
	const PixelNumber axisButtonWidth = availableWidth/4;
	const PixelNumber axisButtonStep = (availableWidth - 3 *axisButtonWidth)/2 + axisButtonWidth;
	if (IsBitSet(mbox.controls, X_AXIS))
	{
		AddItem(new ValueMenuItem(top + 2 * rowHeight, left, axisButtonWidth, MenuItem::CentreAlign, fontToUse, MenuItem::AlwaysVisible, true, 510, 1), true);
	}
	if (IsBitSet(mbox.controls, Y_AXIS))
	{
		AddItem(new ValueMenuItem(top + 2 * rowHeight, left + axisButtonStep, axisButtonWidth, MenuItem::CentreAlign, fontToUse, MenuItem::AlwaysVisible, true, 511, 1), true);
	}
	if (IsBitSet(mbox.controls, Z_AXIS))
	{
		AddItem(new ValueMenuItem(top + 2 * rowHeight, left + 2 * axisButtonStep, axisButtonWidth, MenuItem::CentreAlign, fontToUse, MenuItem::AlwaysVisible, true, 512, 2), true);
	}

	const PixelNumber okCancelButtonWidth = 30;
	if (mbox.mode & 2)
	{
		AddItem(new ButtonMenuItem(top + 3 * rowHeight, left, okCancelButtonWidth, fontToUse, MenuItem::AlwaysVisible, "OK", "M292 P0", nullptr), true);
	}
	if (mbox.mode & 1)
	{
		AddItem(new ButtonMenuItem(top + 3 * rowHeight, right - okCancelButtonWidth, okCancelButtonWidth, fontToUse, MenuItem::AlwaysVisible, "Cancel", "M292 P1", nullptr), true);
	}
}

// Clear the message box and display the menu underneath it
void Menu::ClearMessageBox()
{
	displayingMessageBox = false;
	Reload();
}

void Menu::Pop()
{
	// currentMargin = 0;
	lcd.Clear();
	rowOffset = 0;
	--numNestedMenus;
	Reload();
}

void Menu::LoadError(const char *msg, unsigned int line)
{
	// Remove selectable items that may obscure view of the error message
	ResetCache();

	lcd.Clear();
	lcd.SetFont(0);
	lcd.print("Error loading menu\nFile: ");
	lcd.print((numNestedMenus > 0) ? filenames[numNestedMenus - 1].c_str() : "(none)");
	if (line != 0)
	{
		lcd.print("\nLine ");
		lcd.print(line);
		if (errorColumn != 0)
		{
			lcd.print(" column ");
			lcd.print(errorColumn);
		}
	}
	lcd.write('\n');
	lcd.print(msg);

	lastActionTime = millis();
	timeoutValue = ErrorTimeout;
	displayingErrorMessage = true;
}

// Parse a line in a menu layout file returning any error message, or nullptr if there was no error.
// Leading whitespace has already been skipped.
const char *Menu::ParseMenuLine(char * const commandWord)
{
	errorColumn = 0;

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
		errorColumn = (args - commandWord) + 1;
		return "Bad command";
	}

	if (*args != 0)
	{
		*args = 0;		// null terminate the command word
		++args;
	}

	// Parse the arguments
	MenuItem::Visibility xVis = 0;
	unsigned int decimals = 0;
	unsigned int nparam = 0;
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
			row = SafeStrtoul(args, &args);
			break;

		case 'C':
			column = SafeStrtoul(args, &args);
			break;

		case 'F':
			fontNumber = min<unsigned int>(SafeStrtoul(args, &args), lcd.GetNumFonts() - 1);
			break;

		case 'V':
			xVis = SafeStrtoul(args, &args);
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

		case 'H':
			alignment = SafeStrtoul(args, &args);
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

	lcd.SetCursor(row + currentMargin, column + currentMargin);

	// Create an object resident in memory corresponding to the menu layout file's description
	if (StringEqualsIgnoreCase(commandWord, "text"))
	{
		const char *const acText = AppendString(text);
		MenuItem * const pNewItem = new TextMenuItem(row, column, width, alignment, fontNumber, xVis, acText);
		AddItem(pNewItem, false);
		column += pNewItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "image") && fname != nullptr)
	{
		ImageMenuItem * const pNewItem = new ImageMenuItem(row, column, xVis, fname);
		AddItem(pNewItem, false);
		column += pNewItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "button"))
	{
		const char * const textString = AppendString(text);
		const char * const actionString = AppendString(action);
		const char * const c_acFileString = AppendString(fname);
		ButtonMenuItem * const pNewItem = new ButtonMenuItem(row, column, width, fontNumber, xVis, textString, actionString, c_acFileString);
		AddItem(pNewItem, true);
		column += pNewItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "value"))
	{
		ValueMenuItem * const pNewItem = new ValueMenuItem(row, column, width, alignment, fontNumber, xVis, false, nparam, decimals);
		AddItem(pNewItem, false);
		column += pNewItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "alter"))
	{
		ValueMenuItem * const pNewItem = new ValueMenuItem(row, column, width, alignment, fontNumber, xVis, true, nparam, decimals);
		AddItem(pNewItem, true);
		column += pNewItem->GetWidth();
	}
	else if (StringEqualsIgnoreCase(commandWord, "files"))
	{
		const char * const actionString = AppendString(action);
		const char *const dir = AppendString(dirpath);
		const char *const acFileString = AppendString(fname);
		AddItem(new FilesMenuItem(row, 0, NumCols, fontNumber, xVis, actionString, dir, acFileString, nparam), true);
		row += nparam * lcd.GetFontHeight(fontNumber);
		column = 0;
	}
	else
	{
		errorColumn = 1;
		return "Unknown command";
	}

	return nullptr;
}

void Menu::ResetCache()
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

void Menu::Reload()
{
	displayingFixedMenu = false;
	if (numNestedMenus == 1)
	{
		currentMargin = 0;
		lcd.Clear();
	}
	else
	{
		currentMargin = 0;
		const PixelNumber right = NumCols;
		const PixelNumber bottom = NumRows;
		lcd.Clear(currentMargin, currentMargin, bottom, right);

		// Draw the outline
		// lcd.Line(currentMargin, currentMargin, bottom, currentMargin, PixelMode::PixelSet);
		// lcd.Line(currentMargin, currentMargin, currentMargin, right, PixelMode::PixelSet);
		// lcd.Line(bottom, currentMargin, bottom, right, PixelMode::PixelSet);
		// lcd.Line(currentMargin, right, bottom, right, PixelMode::PixelSet);

		// currentMargin += InnerMargin;
	}

	ResetCache();
	displayingErrorMessage = false;

	lcd.SetRightMargin(NumCols - currentMargin);
	const char * const fname = filenames[numNestedMenus - 1].c_str();
	FileStore * const file = reprap.GetPlatform().OpenFile(MENU_DIR, fname, OpenMode::read);
	if (file == nullptr)
	{
		LoadError("File not found", 0);
	}
	else
	{
		row = 0;
		column = 0;
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

void Menu::AddItem(MenuItem *item, bool isSelectable)
{
	item->UpdateWidthAndHeight(lcd);
	MenuItem::AppendToList((isSelectable) ? &selectableItems : &unSelectableItems, item);
}

// Append a string to the string buffer and return its index
const char *Menu::AppendString(const char *s)
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
void Menu::EncoderAction_ExecuteHelper(const char *const cmd)
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

void Menu::EncoderActionEnterItemHelper()
{
	if (highlightedItem != nullptr && highlightedItem->IsVisible())
	{
		String<MaxFilenameLength> command;
		if (highlightedItem->Select(command.GetRef()))
		{
			char *pcCurrentCommand = command.GetRef().Pointer();
			int nNextCommandIndex = StringContains(pcCurrentCommand, "|");
			while (-1 != nNextCommandIndex)
			{
				*(pcCurrentCommand + nNextCommandIndex) = '\0';
				EncoderAction_ExecuteHelper(pcCurrentCommand);
				pcCurrentCommand += nNextCommandIndex + 1;
				nNextCommandIndex = StringContains(pcCurrentCommand, "|");
			}
			EncoderAction_ExecuteHelper(pcCurrentCommand);
		}
		else if (highlightedItem->CanAdjust())
		{
			itemIsSelected = true;
		}
	}
}

void Menu::EncoderActionScrollItemHelper(int action)
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
				lcd.Clear();
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
void Menu::EncoderAction(int action)
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

// Refresh is called every Spin() of the Display under most circumstances; an appropriate place to check if timeout action needs to be taken
void Menu::Refresh()
{
	if (!reprap.GetPlatform().GetMassStorage()->IsDriveMounted(0))
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

void Menu::DrawAll()
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
	const PixelNumber rightMargin = NumCols - currentMargin;
	for (MenuItem *item = selectableItems; item != nullptr; item = item->GetNext())
	{
		item->Draw(lcd, rightMargin, (item == highlightedItem), rowOffset);
	}

	for (MenuItem *item = unSelectableItems; item != nullptr; item = item->GetNext())
	{
		item->Draw(lcd, rightMargin, false, rowOffset);
	}
}

// Clear any highlighting. Call Refresh() after calling this to clear it on the display.
void Menu::ClearHighlighting()
{
	highlightedItem = nullptr;
	itemIsSelected = false;
}

// Move the highlighted item forwards or backwards through the selectable items, setting it to nullptr if nothing is selectable
// On entry, n is nonzero
void Menu::AdvanceHighlightedItem(int n)
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
MenuItem *Menu::FindNextSelectableItem(MenuItem *p) const
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
MenuItem *Menu::FindPrevSelectableItem(MenuItem *p) const
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

#endif

// End
