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
#include "Movement/Move.h"
#include "Display.h"
#include "Tools/Tool.h"
#include "Networking/Network.h"

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

TextMenuItem *TextMenuItem::freelist = nullptr;

TextMenuItem::TextMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, Visibility xVis, CheckFunction bF, const char* t)
	: MenuItem(r, c, fn), text(t), m_xVisCase(xVis), m_bF(bF)
{
}

bool TextMenuItem::Visible() const
{
	return m_bF(m_xVisCase);
}

void TextMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset)
{
	if (Visible())
	{
		lcd.SetCursor(row - tOffset, column);
		// lcd.SetRightMargin(rightMargin);

		lcd.TextInvert(false);
		lcd.print(text);

		// lcd.SetCursor(row + currentMargin, column + currentMargin);
		// lcd.SetFont(fonts[fontNumber]);
		// lcd.print(text);
		// row = lcd.GetRow() - currentMargin;
		// column = lcd.GetColumn() - currentMargin;

		// lcd.ClearToMargin();
	}
}

// TODO need to clean up this design since it isn't meaningful to select a text item
const char* TextMenuItem::Select()
{
	return text;
}

ButtonMenuItem *ButtonMenuItem::freelist = nullptr;

ButtonMenuItem::ButtonMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, Visibility xVis, CheckFunction bF, const char* t, const char* cmd, char const* acFile)
	: MenuItem(r, c, fn), text(t), command(cmd), m_acFile(acFile), m_xVisCase(xVis), m_bF(bF)
{
	m_acCommand[0] = '\0';
}

bool ButtonMenuItem::Visible() const
{
	return m_bF(m_xVisCase);
}

void ButtonMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset)
{
	if (Visible() && column < NumCols)
	{
		lcd.SetCursor(row - tOffset, column);
		lcd.SetRightMargin(rightMargin);

		lcd.TextInvert(highlight);
		lcd.print(text); // TODO: create Print(char[], bool) to combine these two lines

		lcd.TextInvert(false);
		lcd.ClearToMargin();
	}
}

// TODO WS1: if we overflow the command or directory string, we should probably offer a return value that tells the caller to do nothing...

// NOTE: menu names must not begin with 'G', 'M' or 'T'
const char* ButtonMenuItem::Select()
{
	m_acCommand[0] = '\0';

	SafeStrncpy(m_acCommand, command, strlen(command) + 1);
	// WS1 assumed safe

	int nReplacementIndex = StringContains(m_acCommand, "menu");
	if (-1 != nReplacementIndex)
	{
		nReplacementIndex -= strlen("menu");

		// TODO WS1
		SafeStrncpy(m_acCommand + nReplacementIndex, m_acFile, min(strlen(m_acFile) + 1, sizeof(m_acCommand) - nReplacementIndex));
	}

	return m_acCommand;
}

PixelNumber ButtonMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, const LcdFont *oFont)
{
	PixelNumber tOffsetRequest = tCurrentOffset;

	// Are we off the bottom of the visible window?
	if (64 + tCurrentOffset <= row + oFont->height + 1)
	{
		// tOffsetRequest = tCurrentOffset + row - 3;
		tOffsetRequest = row - 3;
	}

	// Should we move back up?
	if (row < tCurrentOffset + 3)
	{
		if (row > 3)
			tOffsetRequest = row - 3;
		else
			tOffsetRequest = 0;
	}

	return tOffsetRequest;
}

ValueMenuItem::ValueMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, PixelNumber w, unsigned int v, unsigned int d)
	: MenuItem(r, c, fn), valIndex(v), currentValue(0.0), width(w), decimals(d), adjusting(false)
{
}

void ValueMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset)
{
	lcd.SetCursor(row - tOffset, column);
	lcd.SetRightMargin(min<PixelNumber>(column + width, rightMargin));
	lcd.TextInvert(highlight);

	bool error = false;
	if (!adjusting)
	{
		const unsigned int itemNumber = valIndex % 100;
		switch (valIndex/100)
		{
		case 0:		// heater current temperature
			currentValue = max<float>(reprap.GetGCodes().GetItemCurrentTemperature(itemNumber), 0.0f);
			break;

		case 1:		// heater active temperature
			currentValue = max<float>(reprap.GetGCodes().GetItemActiveTemperature(itemNumber), 0.0f);
			break;

		case 2:		// heater standby temperature
			currentValue = max<float>(reprap.GetGCodes().GetItemStandbyTemperature(itemNumber), 0.0f);
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

			case 10: // X
				{
					float m[MaxAxes];
					reprap.GetMove().GetCurrentMachinePosition(m, false);
					currentValue = m[X_AXIS];
				}
				break;

			case 11: // Y
				{
					float m[MaxAxes];
					reprap.GetMove().GetCurrentMachinePosition(m, false);
					currentValue = m[Y_AXIS];
				}
				break;

			case 12: // Z
				{
					float m[MaxAxes];
					reprap.GetMove().GetCurrentMachinePosition(m, false);
					currentValue = m[Z_AXIS];
				}
				break;

			case 13: // E0
				currentValue = reprap.GetGCodes().GetRawExtruderTotalByDrive(0);
				break;

			case 14: // E1
				currentValue = reprap.GetGCodes().GetRawExtruderTotalByDrive(1);
				break;

			case 15: // E2
				currentValue = reprap.GetGCodes().GetRawExtruderTotalByDrive(2);
				break;

			case 16: // E3
				currentValue = reprap.GetGCodes().GetRawExtruderTotalByDrive(3);
				break;

			// TODO: drives on some configurations could go up to 519, avoid this collision:
			case 19: // Z baby-step
				currentValue = reprap.GetGCodes().GetBabyStepOffset();
				break;

			case 20:
				currentValue = reprap.GetCurrentToolNumber();
				break;

			// Platform's IP address is the "planned", Network's IP address is the "actual"
			case 30:
				currentValue = reprap.GetNetwork().GetIPAddress(0)[0];
				break;

			case 31:
				currentValue = reprap.GetNetwork().GetIPAddress(0)[1];
				break;

			case 32:
				currentValue = reprap.GetNetwork().GetIPAddress(0)[2];
				break;

			case 33:
				currentValue = reprap.GetNetwork().GetIPAddress(0)[3];
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

PixelNumber ValueMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, const LcdFont *oFont)
{
	// TODO

	return 0;
}

bool ValueMenuItem::Adjust_SelectHelper()
{
	const unsigned int itemNumber = GetReferencedToolNumber();

	bool error = false;
	switch (valIndex/100)
	{
	case 1: // heater active temperature
		{
			if (1 > currentValue) // 0 is off
			{
				reprap.GetGCodes().SetItemActiveTemperature(itemNumber, -273.15f);
			}
			else // otherwise ensure the tool is made active at the same time (really only matters for 79)
			{
				if (80 > itemNumber)
				{
					reprap.SelectTool(itemNumber, false);
				}
				reprap.GetGCodes().SetItemActiveTemperature(itemNumber, currentValue);
			}
		}
		break;

	case 2: // heater standby temperature
		reprap.GetGCodes().SetItemStandbyTemperature(itemNumber, (1 > currentValue) ? -273.15f : currentValue);
		break;

	case 3: // fan %
		if (itemNumber == 99)
		{
			reprap.GetGCodes().SetMappedFanSpeed(currentValue * 0.01);
		}
		else
		{
			reprap.GetPlatform().SetFanValue(itemNumber, currentValue * 0.01);
		}
		break;

	case 4: // extruder %
		reprap.GetGCodes().SetExtrusionFactor(itemNumber, currentValue * 0.01);
		break;

	case 5: // misc.
		switch (itemNumber)
		{
		case 0:
			reprap.GetGCodes().SetSpeedFactor(currentValue * 0.01);
			break;

		case 20:
			reprap.SelectTool(currentValue, false);
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

unsigned int ValueMenuItem::GetReferencedToolNumber()
{
	unsigned int uToolNumber = valIndex % 100;

	if (79 == uToolNumber)
		uToolNumber = reprap.GetCurrentOrDefaultTool()->Number();

	return uToolNumber;
}

bool ValueMenuItem::Adjust_AlterHelper(int clicks)
{
	const unsigned int itemNumber = GetReferencedToolNumber();

	switch (valIndex/100)
	{
	case 1:	// heater active temperature
		if (itemNumber < 80) // Tool heaters
		{
			// If we're decreasing, make any value smaller than 95 go to 0
			// If we're increasing, make any value between 0 and 95 jump directly to 95
			// Also cap the maximum (currently 270)
			if (0 > clicks) // decrementing
			{
				currentValue += clicks;

				if (95 > (int)currentValue)
					currentValue = 0;
			}
			else // incrementing
			{
				if (0 == currentValue)
				{
					currentValue = (95 - 1);
					// --clicks;
				}

				currentValue = min<int>(currentValue + clicks, reprap.GetHeat().GetHighestTemperatureLimit(reprap.GetTool(itemNumber)->Heater(0)));
			}
		}
		else
		{
			currentValue += clicks;
		}
		break;

	case 2:	// heater standby temperature
		if (itemNumber < 80) // Tool heaters
		{
			if (0 > clicks) // decrementing
			{
				currentValue += clicks;

				if (95 > (int)currentValue)
					currentValue = 0;
			}
			else // incrementing
			{
				if (0 == currentValue)
				{
					currentValue = (95 - 1);
					// --clicks;
				}

				currentValue = min<int>(currentValue + clicks, reprap.GetHeat().GetHighestTemperatureLimit(reprap.GetTool(itemNumber)->Heater(0)));
			}
		}
		else
		{
			currentValue += clicks;
		}
		break;

	case 3: // fan %
		currentValue = constrain<int>(currentValue + clicks, 0, 100);
		break;

	case 5: // misc.
		switch (itemNumber)
		{
		case 0: // 500 Feed Rate
			currentValue = constrain<float>(currentValue + (float)clicks, 10, 500);
			break;

		case 20: // 520 Tool Selection
			currentValue = constrain<int>(currentValue + clicks, -1, 255);
			break;

		default:
			// error = true;
			break;
		}
		break;

	default:
		currentValue += (float)clicks;
		break;
	}

	return false;
}

bool ValueMenuItem::Adjust(int clicks)
{
	if (clicks == 0)	// if button has been pressed
	{
		return Adjust_SelectHelper();
	}

	// Wheel has scrolled: alter value
	return Adjust_AlterHelper(clicks);
}

FilesMenuItem *FilesMenuItem::freelist = nullptr;

FilesMenuItem::FilesMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, const char *cmd, const char *dir, const char *acFile, unsigned int nf, unsigned int uFontHeight)
	: MenuItem(r, c, fn), command(cmd), initialDirectory(dir), m_acFile(acFile), m_uDisplayLines(nf), m_uFontHeight(uFontHeight),
        m_uListingFirstVisibleIndex(0), m_uListingSelectedIndex(0), m_oMS(reprap.GetPlatform().GetMassStorage())
{
	m_acCommand[0] = '\0';

	// There's no guarantee that initialDirectory has a trailing '/'
	SafeStrncpy(m_acCurrentDirectory, initialDirectory, strlen(initialDirectory) + 1);
	// WS1 assumed safe
	if ('/' != m_acCurrentDirectory[strlen(m_acCurrentDirectory) - 1])
		SafeStrncpy(m_acCurrentDirectory + strlen(m_acCurrentDirectory), "/", 2);
		// WS1 assumed safe

	// We don't bother with m_uHardItemsInDirectory in init. list because --
	EnterDirectory();
}

void FilesMenuItem::vResetViewState()
{
	m_uListingSelectedIndex = 0;
	m_uListingFirstVisibleIndex = 0;
}

void FilesMenuItem::EnterDirectory()
{
	vResetViewState();

	m_uHardItemsInDirectory = 0;

	FileInfo oFileInfo;
	if (m_oMS->FindFirst(m_acCurrentDirectory, oFileInfo))
	{
		do
		{
			++m_uHardItemsInDirectory;
		}
		while (m_oMS->FindNext(oFileInfo));
	}
}

bool FilesMenuItem::bInSubdirectory() const
{
	const char *pcPathElement = m_acCurrentDirectory;
	unsigned int uNumSlashes = 0;

	while ('\0' != *pcPathElement)
	{
		if (('/' == *pcPathElement) && ('\0' != *(pcPathElement + 1))) // don't count a trailing slash
			++uNumSlashes;

		++pcPathElement;
	}

	return (1 < uNumSlashes);
}

unsigned int FilesMenuItem::uListingEntries() const
{
	return bInSubdirectory() ? (1 + m_uHardItemsInDirectory) : m_uHardItemsInDirectory;
}

void FilesMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset)
{
	lcd.SetCursor(row, column); // TODO: consider tOffset
	lcd.SetRightMargin(rightMargin);

	if (bInSubdirectory()) // manually add the ".." entry
	{
		// We are writing text to line numbers (0), 1, 2 ... m_uDisplayLines - 1 (0 = "..")
		// If m_uListingFirstVisibleIndex is 0, then we can see "..":
		//   The remaining fs entries are m_uListingFirstVisibleIndex, m_uListingFirstVisibleIndex + 1 ... m_uListingFirstVisibleIndex + m_uDisplayLines - 2
		// Otherwise (m_uListingFirstVisibleIndex is not 0, so we can't see "..")
		//   The fs entries are m_uListingFirstVisibleIndex, m_uListingFirstVisibleIndex + 1 ... m_uListingFirstVisibleIndex + m_uDisplayLines - 1
		// (We iterate the file system exactly the same, we just use 'i' to make sure we stop at the appropriate time.)

		uint8_t i = 0;

		// e.g. m_uListingFirstVisible = 0, we'll see "..", "a.gcode", "b.gcode", ...
		//      m_uListingFirstVisible = 1, we'll see "a.gcode", "b.gcode", ...
		if (0 == m_uListingFirstVisibleIndex)
		{
			lcd.SetCursor(row, column);

			if (highlight && (m_uListingSelectedIndex == m_uListingFirstVisibleIndex))
			{
				lcd.print("> ");
			}
			else
			{
				lcd.print("  ");
			}
			lcd.print("..");

			lcd.ClearToMargin();

			i = 1;
		}

		// Seek to the first file that is in view
		int nDirReferencedLocation = -1;
		FileInfo oFileInfo;
		if (m_oMS->FindFirst(m_acCurrentDirectory, oFileInfo))
		{
			do
			{
				++nDirReferencedLocation;
			}
			while ((nDirReferencedLocation < static_cast<int>(m_uListingFirstVisibleIndex - 1)) && m_oMS->FindNext(oFileInfo));
			// -- relying on short-circuit, do not change order!
		}

		bool bAnotherFile = (-1 != nDirReferencedLocation);


		// i iterates over the visible lines on the screen (0, 1, 2...)

		// We always iterate the entire viewport so that old listing lines that may not be overwritten are cleared
		for (/* uint8_t i = (0 == m_uListingFirstVisible ? 1 : 0) */ ; i < m_uDisplayLines; ++i, bAnotherFile = m_oMS->FindNext(oFileInfo))
		{
			lcd.SetCursor(row + (m_uFontHeight * i), column);

			// If there's actually a file to describe (not just ensuring viewport line clear)
			if (bAnotherFile)
			{
				if (highlight && (m_uListingSelectedIndex == (i + m_uListingFirstVisibleIndex)))
					lcd.print("> ");
				else
					lcd.print("  ");

				if (oFileInfo.isDirectory)
					lcd.print("./");

				lcd.print(oFileInfo.fileName);
			}

			lcd.ClearToMargin();
		}
	}
	else // logical root, no ".." entry
	{
		// We are writing text to line numbers 0, 1 ... m_uDisplayLines - 1
		// These are fs entries m_uListingFirstVisibleIndex, m_uListingFirstVisibleIndex + 1 ... m_uListingFirstVisibleIndex + m_uDisplayLines - 1

		// Seek to the first file that is in view
		int nDirListingLocation = -1;
		FileInfo oFileInfo;
		if (m_oMS->FindFirst(m_acCurrentDirectory, oFileInfo))
		{
			do
			{
				++nDirListingLocation;
			}
			while ((nDirListingLocation < static_cast<int>(m_uListingFirstVisibleIndex)) && m_oMS->FindNext(oFileInfo));
			// -- relying on short-circuit, do not change order!
		}

		bool bAnotherFile = (-1 != nDirListingLocation);

		// We always iterate the entire viewport so that old listing lines that may not be overwritten are cleared
		for (uint8_t i = 0; i < m_uDisplayLines; ++i, bAnotherFile = m_oMS->FindNext(oFileInfo))
		{
			lcd.SetCursor(row + (m_uFontHeight * i), column);

			// If there's actually a file to describe (not just ensuring viewport line clear)
			if (bAnotherFile)
			{
				if (highlight && (m_uListingSelectedIndex == (i + m_uListingFirstVisibleIndex)))
					lcd.print("> ");
				else
					lcd.print("  ");

				if (oFileInfo.isDirectory)
					lcd.print("./");

				lcd.print(oFileInfo.fileName);
			}

			lcd.ClearToMargin();
		}
	}

	// TODO: cache these filenames to avoid the SD overhead each time...
}

void FilesMenuItem::Enter(bool bForwardDirection)
{
	if (bForwardDirection || 0 == uListingEntries())
	{
		m_uListingSelectedIndex = 0;
		m_uListingFirstVisibleIndex = 0;
	}
	else
	{
		m_uListingSelectedIndex = uListingEntries() - 1;
		m_uListingFirstVisibleIndex = ((uListingEntries() > m_uDisplayLines) ? (uListingEntries() - m_uDisplayLines) : 0);
	}
}

int FilesMenuItem::Advance(int nCounts)
{
	// In case of empty directory, there's nothing the control itself can do
	if (0 == uListingEntries())
	{
		// Force the menu system to the next item in the list
		// ++nCounts;
	}
	else
	{
		while (nCounts > 0)
		{
			// Advancing one more would take us past the end of the list
			// Instead, return the remaining count so that the other
			// selectable menu items can be scrolled.
			if (uListingEntries() == m_uListingSelectedIndex + 1)
				break;

			++m_uListingSelectedIndex;
			--nCounts;

			// Move the visible portion of the list down, if required
			if (m_uListingSelectedIndex == m_uListingFirstVisibleIndex + m_uDisplayLines)
				++m_uListingFirstVisibleIndex;
		}

		while (nCounts < 0)
		{
			// We're already at the first item in the visible list;
			// return the remaining action to the menu system.
			if (0 == m_uListingSelectedIndex)
				break;

			--m_uListingSelectedIndex;
			++nCounts;

			// Move the visible portion of the list up, if required
			if (m_uListingSelectedIndex < m_uListingFirstVisibleIndex)
				--m_uListingFirstVisibleIndex;
		}
	}

	return nCounts;
}

const char* FilesMenuItem::Select()
{
	char* acReVal = nullptr;

	// Several cases:
	// TEST 1. ".." entry - call EnterDirectory(), using saved state information
	// TEST 2. Directory - call EnterDirectory(), adding to saved state information
	// 3. File - run command with filename as argument

	// Get information on the item selected

	if (bInSubdirectory() && (0 == m_uListingSelectedIndex)) // meaning ".."
	{
		// TODO: go up one level rather than to logical root
		// There's no guarantee that initialDirectory has a trailing '/'
		SafeStrncpy(m_acCurrentDirectory, initialDirectory, strlen(initialDirectory) + 1);
		// WS1 assumed safe
		if ('/' != m_acCurrentDirectory[strlen(m_acCurrentDirectory) - 1])
			SafeStrncpy(m_acCurrentDirectory + strlen(m_acCurrentDirectory), "/", 2);
		    // WS1 assumed safe

		EnterDirectory();
		// return nullptr;
	}
	else
	{
		// If subdir:
		// If ".." is visible, the selected file is visible index m_uListingSelectedIndex, fs index m_uListingSelectedIndex - 1
		// If ".." is not visible, the selected file is visible index m_uListingSelectedIndex, fs index m_uListingSelectedIndex - 1
		// If logical root:
		// ".." is never visible, so the selected file is visible index m_uListingSelectedIndex, fs index m_uListingSelectedIndex

		unsigned int uFindFileIndex = bInSubdirectory() ? m_uListingSelectedIndex - 1 : m_uListingSelectedIndex;

		// Seek to the selected file
		int nDirectoryLocation = -1;
		FileInfo oFileInfo;
		if (m_oMS->FindFirst(m_acCurrentDirectory, oFileInfo))
		{
			do
			{
				++nDirectoryLocation;
			}
			while ((nDirectoryLocation != static_cast<int>(uFindFileIndex)) && m_oMS->FindNext(oFileInfo)); // relying on short-circuit -- don't change order!
		}

		if (nDirectoryLocation == static_cast<int>(uFindFileIndex)) // handles empty directory (no action)
		{
			if (oFileInfo.isDirectory)
			{
				// Build the new directory, and ensure it's terminated with a forward slash
				// TODO WS1
				SafeStrncpy(m_acCurrentDirectory + strlen(m_acCurrentDirectory), oFileInfo.fileName, min(strlen(oFileInfo.fileName) + 1, sizeof(m_acCurrentDirectory) - strlen(m_acCurrentDirectory)));
				SafeStrncpy(m_acCurrentDirectory + strlen(m_acCurrentDirectory), "/", min(2u, sizeof(m_acCurrentDirectory) - strlen(m_acCurrentDirectory)));

				EnterDirectory();
				// return nullptr;
			}
			else
			{
				m_acCommand[0] = '\0';

				SafeStrncpy(m_acCommand, command, strlen(command) + 1);
				// WS1 assumed safe

				int nReplacementIndex = StringContains(m_acCommand, "#0");
				if (-1 != nReplacementIndex)
				{
					nReplacementIndex -= strlen("#0");

					SafeStrncpy(m_acCommand + nReplacementIndex, "\"", min(2u, sizeof(m_acCommand) - nReplacementIndex));
					++nReplacementIndex;

					SafeStrncpy(m_acCommand + nReplacementIndex, m_acCurrentDirectory, min(strlen(m_acCurrentDirectory) + 1, sizeof(m_acCommand) - nReplacementIndex));
					nReplacementIndex += strlen(m_acCurrentDirectory);

					SafeStrncpy(m_acCommand + nReplacementIndex, oFileInfo.fileName, min(strlen(oFileInfo.fileName) + 1, sizeof(m_acCommand) - nReplacementIndex));
					nReplacementIndex += strlen(oFileInfo.fileName);

					SafeStrncpy(m_acCommand + nReplacementIndex, "\"", min(2u, sizeof(m_acCommand) - nReplacementIndex));
					++nReplacementIndex;

					SafeStrncpy(m_acCommand + nReplacementIndex, command + StringContains(command, "#0"), min(strlen(command + StringContains(command, "#0")) + 1, sizeof(m_acCommand) - nReplacementIndex));
					// nReplacementIndex = ... // unused
				}

				// TODO: do this on the way in and it might be less work...
				//   On the other hand, this only occurs when an item is selected so it's O(1) vs. O(n)
				nReplacementIndex = StringContains(m_acCommand, "menu");
				if (-1 != nReplacementIndex)
				{
					nReplacementIndex -= strlen("menu");

					SafeStrncpy(m_acCommand + nReplacementIndex, m_acFile, min(strlen(m_acFile) + 1, sizeof(m_acCommand) - nReplacementIndex));
				}

				acReVal = m_acCommand;
				// return m_acCommand; // would otherwise break encapsulation, but the return is const
			}
		}
	}

	return acReVal;
}

PixelNumber FilesMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, const LcdFont *oFont)
{
	// TODO

	return 0;
}

// End

