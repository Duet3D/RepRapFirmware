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
#include "PrintMonitor.h"

MenuItem::MenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis)
	: row(r), column(c), width(w), fontNumber(fn), visCase(vis), itemChanged(true), highlighted(false), next(nullptr)
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

bool MenuItem::IsVisible() const
{
	switch (visCase)
	{
	default:
	case 0:		return true;
	case 2:		return reprap.GetGCodes().IsReallyPrinting();
	case 3:		return !reprap.GetGCodes().IsReallyPrinting();
	case 4:		return reprap.GetPrintMonitor().IsPrinting();
	case 5:		return !reprap.GetPrintMonitor().IsPrinting();
	case 6:		return reprap.GetGCodes().IsPaused() || reprap.GetGCodes().IsPausing();
	case 7:		return reprap.GetGCodes().IsReallyPrinting() || reprap.GetGCodes().IsResuming();
	case 10:	return reprap.GetPlatform().GetMassStorage()->IsDriveMounted(0);
	case 11:	return !reprap.GetPlatform().GetMassStorage()->IsDriveMounted(0);
	case 20:	return reprap.GetCurrentOrDefaultTool()->HasTemperatureFault();
	case 28:	return reprap.GetHeat().GetStatus(reprap.GetHeat().GetBedHeater(0)) == Heat::HS_fault;
	}
}

TextMenuItem::TextMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char* t)
	: MenuItem(r, c, w, fn, vis), text(t)
{
}

void TextMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset)
{
	// We ignore the 'highlight' parameter because text items are not selectable
	if (IsVisible() && itemChanged)
	{
		lcd.SetFont(fontNumber);
		lcd.SetCursor(row - tOffset, column);
		lcd.SetRightMargin(min<PixelNumber>(rightMargin, column + width));
		lcd.TextInvert(false);
		lcd.print(text);
		lcd.ClearToMargin();
		itemChanged = false;
	}
}

void TextMenuItem::UpdateWidth(Lcd7920& lcd, PixelNumber offScreenRow, PixelNumber numCols)
{
	if (width == 0)
	{
		lcd.SetFont(fontNumber);
		lcd.SetCursor(offScreenRow, 0);
		lcd.SetRightMargin(numCols);
		lcd.TextInvert(false);
		lcd.print(text);
		width = lcd.GetColumn();
	}
}

ButtonMenuItem::ButtonMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char* t, const char* cmd, char const* acFile)
	: MenuItem(r, c, w, fn, vis), text(t), command(cmd), m_acFile(acFile)
{
	m_acCommand[0] = '\0';
}

void ButtonMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset)
{
	if (IsVisible() && (itemChanged || highlight != highlighted) && column < NumCols)
	{
		lcd.SetFont(fontNumber);
		lcd.SetCursor(row - tOffset, column);
		lcd.SetRightMargin(min<PixelNumber>(rightMargin, column + width));
		lcd.TextInvert(highlight);
		lcd.WriteSpaces(1);				// space at start in case highlighted
		lcd.print(text);
		lcd.ClearToMargin();
		lcd.TextInvert(false);
		itemChanged = false;
		highlighted = highlight;
	}
}

void ButtonMenuItem::UpdateWidth(Lcd7920& lcd, PixelNumber offScreenRow, PixelNumber numCols)
{
	if (width == 0)
	{
		lcd.SetFont(fontNumber);
		lcd.SetCursor(offScreenRow, 0);
		lcd.SetRightMargin(numCols);
		lcd.TextInvert(false);
		lcd.WriteSpaces(1);				// space at start to allow for highlighting
		lcd.print(text);
		lcd.WriteSpaces(1);				// space at end to allow for highlighting
		width = lcd.GetColumn();
	}
}

// TODO WS1: if we overflow the command or directory string, we should probably offer a return value that tells the caller to do nothing...

// NOTE: menu names must not begin with 'G', 'M' or 'T'
const char* ButtonMenuItem::Select()
{
	const int nReplacementIndex = StringContains(command, "#0");
	if (-1 != nReplacementIndex)
	{
		m_acCommand.copy(command, nReplacementIndex);
		m_acCommand.cat(m_acFile);
	}
	else
	{
		m_acCommand.copy(command);
	}

	return m_acCommand.c_str();
}

PixelNumber ButtonMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const
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

ValueMenuItem::ValueMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, bool adj, unsigned int v, unsigned int d)
	: MenuItem(r, c, ((w != 0) ? w : DefaultWidth), fn, vis), valIndex(v), currentValue(0.0), decimals(d), adjustable(adj), adjusting(false)
{
}

void ValueMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset)
{
	bool error = false;
	if (!adjusting)
	{
		const unsigned int itemNumber = valIndex % 100;
		const float oldValue = currentValue;

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
				currentValue = reprap.GetNetwork().GetIPAddress(0).GetQuad(0);
				break;

			case 31:
				currentValue = reprap.GetNetwork().GetIPAddress(0).GetQuad(1);
				break;

			case 32:
				currentValue = reprap.GetNetwork().GetIPAddress(0).GetQuad(2);
				break;

			case 33:
				currentValue = reprap.GetNetwork().GetIPAddress(0).GetQuad(3);
				break;

			default:
				error = true;
			}
			break;

		default:
			error = true;
			break;
		}

		if (error || currentValue != oldValue)
		{
			itemChanged = true;
		}
	}

	if (itemChanged || (highlight != highlighted))
	{
		lcd.SetFont(fontNumber);
		lcd.SetCursor(row - tOffset, column);
		lcd.SetRightMargin(min<PixelNumber>(column + width, rightMargin));
		lcd.TextInvert(highlight);
		if (adjustable)
		{
			lcd.WriteSpaces(1);
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
		itemChanged = false;
		highlighted = highlight;
	}
}

const char* ValueMenuItem::Select()
{
	adjusting = true;
	return nullptr;
}

PixelNumber ValueMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const
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
			reprap.GetGCodes().SetSpeedFactor(currentValue);
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

unsigned int ValueMenuItem::GetReferencedToolNumber() const
{
	unsigned int uToolNumber = valIndex % 100;
	if (79 == uToolNumber)
	{
		uToolNumber = reprap.GetCurrentOrDefaultTool()->Number();
	}

	return uToolNumber;
}

bool ValueMenuItem::Adjust_AlterHelper(int clicks)
{
	itemChanged = true;			// we will probably change the value, so it will need to be re-displayed
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
				{
					currentValue = 0;
				}
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

FilesMenuItem::FilesMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char *cmd, const char *dir, const char *acFile, unsigned int nf)
	: MenuItem(r, c, w, fn, vis), numDisplayLines(nf), command(cmd), initialDirectory(dir), m_acFile(acFile),
        m_uListingFirstVisibleIndex(0), m_uListingSelectedIndex(0), m_oMS(reprap.GetPlatform().GetMassStorage())
{
	// There's no guarantee that initialDirectory has a trailing '/'
	m_acCurrentDirectory.copy(initialDirectory);
	const size_t len = m_acCurrentDirectory.strlen();
	if (len == 0 || '/' != m_acCurrentDirectory[len - 1])
	{
		m_acCurrentDirectory.cat('/');
	}

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
	if (m_oMS->FindFirst(m_acCurrentDirectory.c_str(), oFileInfo))
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
	const char *pcPathElement = m_acCurrentDirectory.c_str();
	unsigned int uNumSlashes = 0;

	while ('\0' != *pcPathElement)
	{
		if (('/' == *pcPathElement) && ('\0' != *(pcPathElement + 1))) // don't count a trailing slash
		{
			++uNumSlashes;
		}
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
	// The 'highlight' parameter is not used to highlight this item, but it is still used to tell whether this item is selected or not
	if (itemChanged || highlighted != highlight)
	{
		lcd.SetFont(fontNumber);
		lcd.SetRightMargin(rightMargin);
		uint8_t line = 0;

		// If we are in a subdirectory then we prepend ".." to the list of files
		unsigned int dirEntriesToSkip;
		if (bInSubdirectory())
		{
			if (m_uListingFirstVisibleIndex == 0)
			{
				lcd.SetCursor(row, column);
				lcd.print("  ..");
				lcd.ClearToMargin();
				if (highlight && m_uListingSelectedIndex == 0)
				{
					// Overwriting the initial spaces with '>' avoids shifting the following text when we change the selection
					lcd.SetCursor(row, column);
					lcd.print(">");
				}
				line = 1;
				dirEntriesToSkip = 0;
			}
			else
			{
				dirEntriesToSkip = m_uListingFirstVisibleIndex - 1;
			}
		}
		else
		{
			dirEntriesToSkip = m_uListingFirstVisibleIndex;
		}

		// Seek to the first file that is in view
		FileInfo oFileInfo;
		bool gotFileInfo = m_oMS->FindFirst(m_acCurrentDirectory.c_str(), oFileInfo);
		while (gotFileInfo && dirEntriesToSkip != 0)
		{
			--dirEntriesToSkip;
			gotFileInfo =  m_oMS->FindNext(oFileInfo);
		}

		// We always iterate the entire viewport so that old listing lines that may not be overwritten are cleared
		while (line < numDisplayLines)
		{
			lcd.SetCursor(row + (lcd.GetFontHeight() * line), column);

			// If there's actually a file to describe (not just ensuring viewport line clear)
			if (gotFileInfo)
			{
				lcd.print("  ");
				if (oFileInfo.isDirectory)
				{
					lcd.print("./");
				}
				lcd.print(oFileInfo.fileName);
				lcd.ClearToMargin();
				if (highlight && m_uListingSelectedIndex == line + m_uListingFirstVisibleIndex)
				{
					lcd.SetCursor(row + (lcd.GetFontHeight() * line), column);
					lcd.print(">");
				}
			}
			else
			{
				lcd.ClearToMargin();
			}

			++line;
			if (line == numDisplayLines)
			{
				break;		// skip getting more file info for efficiency
			}
			gotFileInfo = m_oMS->FindNext(oFileInfo);
		}

		m_oMS->AbandonFindNext();				// release the mutex, there may be more files that we don't have room to display

		itemChanged = false;
		highlighted = highlight;
	}
}

void FilesMenuItem::Enter(bool bForwardDirection)
{
	if (bForwardDirection || uListingEntries() == 0)
	{
		m_uListingSelectedIndex = 0;
		m_uListingFirstVisibleIndex = 0;					// select the first item and start the list form the first item
	}
	else
	{
		m_uListingSelectedIndex = uListingEntries() - 1;	// select the last item
		m_uListingFirstVisibleIndex = ((uListingEntries() > numDisplayLines) ? (uListingEntries() - numDisplayLines) : 0);
	}
	itemChanged = true;
}

int FilesMenuItem::Advance(int nCounts)
{
	// In case of empty directory, there's nothing the control itself can do
	if (uListingEntries() != 0)
	{
		while (nCounts > 0)
		{
			// Advancing one more would take us past the end of the list
			// Instead, return the remaining count so that the other selectable menu items can be scrolled.
			if (m_uListingSelectedIndex + 1 == uListingEntries())
			{
				break;
			}

			++m_uListingSelectedIndex;
			--nCounts;

			// Move the visible portion of the list down, if required
			if (m_uListingSelectedIndex == m_uListingFirstVisibleIndex + numDisplayLines)
			{
				++m_uListingFirstVisibleIndex;
			}
		}

		while (nCounts < 0)
		{
			// We're already at the first item in the visible list; return the remaining action to the menu system.
			if (0 == m_uListingSelectedIndex)
			{
				break;
			}

			--m_uListingSelectedIndex;
			++nCounts;

			// Move the visible portion of the list up, if required
			if (m_uListingSelectedIndex < m_uListingFirstVisibleIndex)
			{
				--m_uListingFirstVisibleIndex;
			}
		}

		itemChanged = true;
	}

	return nCounts;
}

const char* FilesMenuItem::Select()
{
	const char* acReVal = nullptr;

	// Several cases:
	// TEST 1. ".." entry - call EnterDirectory(), using saved state information
	// TEST 2. Directory - call EnterDirectory(), adding to saved state information
	// 3. File - run command with filename as argument

	// Get information on the item selected

	if (bInSubdirectory() && 0 == m_uListingSelectedIndex) // meaning ".."
	{
		// TODO: go up one level rather than to logical root
		// There's no guarantee that initialDirectory has a trailing '/'
		m_acCurrentDirectory.copy(initialDirectory);
		const size_t len = m_acCurrentDirectory.strlen();
		if (len == 0 || '/' != m_acCurrentDirectory[len - 1])
		{
			m_acCurrentDirectory.cat('/');
		}
		EnterDirectory();
	}
	else
	{
		// If subdir:
		// If ".." is visible, the selected file is visible index m_uListingSelectedIndex, fs index m_uListingSelectedIndex - 1
		// If ".." is not visible, the selected file is visible index m_uListingSelectedIndex, fs index m_uListingSelectedIndex - 1
		// If logical root:
		// ".." is never visible, so the selected file is visible index m_uListingSelectedIndex, fs index m_uListingSelectedIndex
		unsigned int dirEntriesToSkip = bInSubdirectory() ? m_uListingSelectedIndex - 1 : m_uListingSelectedIndex;

		// Seek to the selected file
		FileInfo oFileInfo;
		bool gotFileInfo = m_oMS->FindFirst(m_acCurrentDirectory.c_str(), oFileInfo);
		while (gotFileInfo && dirEntriesToSkip != 0)
		{
			--dirEntriesToSkip;
			gotFileInfo = m_oMS->FindNext(oFileInfo);
		}
		m_oMS->AbandonFindNext();

		if (gotFileInfo) // handles empty directory (no action)
		{
			if (oFileInfo.isDirectory)
			{
				// Build the new directory, and ensure it's terminated with a forward slash
				m_acCurrentDirectory.cat(oFileInfo.fileName);
				m_acCurrentDirectory.cat('/');
				EnterDirectory();
			}
			else
			{
				int nReplacementIndex = StringContains(command, "#0");
				if (nReplacementIndex != -1)
				{
					m_acCommand.copy(command, nReplacementIndex);
					m_acCommand.cat('"');
					m_acCommand.cat(m_acCurrentDirectory.c_str());
					m_acCommand.cat(oFileInfo.fileName);
					m_acCommand.cat('"');
					m_acCommand.cat(command + nReplacementIndex + strlen("#0"));
				}
				else
				{
					m_acCommand.copy(command);
				}

				// TODO: do this on the way in and it might be less work...
				//   On the other hand, this only occurs when an item is selected so it's O(1) vs. O(n)
				nReplacementIndex = StringContains(m_acCommand.c_str(), "menu");
				if (nReplacementIndex != -1)
				{
					m_acCommand.Truncate(nReplacementIndex);
					m_acCommand.cat(m_acFile);
				}

				acReVal = m_acCommand.c_str();
			}
		}
	}

	return acReVal;
}

PixelNumber FilesMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const
{
	// TODO
	return 0;
}

// Image menu item members
// The image file format is:
// Byte 0 = number of columns
// Byte 1 = number of rows
// Remaining bytes = data, 1 row at a time. If the number of columns is not a multiple of 8 then the data for each row is padded to a multiple of 8 bits.
ImageMenuItem::ImageMenuItem(PixelNumber r, PixelNumber c, Visibility vis, const char *pFileName)
	: MenuItem(r, c, 0, 0, vis)
{
	fileName.copy(pFileName);
}

void ImageMenuItem::Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset)
{
	if (itemChanged || highlight != highlighted)
	{
		FileStore * const fs = reprap.GetPlatform().OpenFile(MENU_DIR, fileName.c_str(), OpenMode::read);
		if (fs != nullptr)
		{
			uint8_t widthAndHeight[2];
			if (fs->Read(widthAndHeight, 2) == 2)
			{
				const PixelNumber cols = widthAndHeight[0];
				const PixelNumber rows = widthAndHeight[1];
				if (cols != 0 && cols <= NumCols && rows != 0)
				{
					const size_t bytesPerRow = (cols + 7)/8;
					for (PixelNumber irow = 0; irow < rows; ++irow)
					{
						uint8_t buffer[NumCols/8];
						if (fs->Read(buffer, bytesPerRow) != (int)bytesPerRow)
						{
							break;
						}
						lcd.BitmapRow(row - tOffset + irow, column,  cols, buffer, highlight);
						++irow;
					}
				}
			}
			fs->Close();
		}
		itemChanged = false;
		highlighted = highlight;
	}
}

void ImageMenuItem::UpdateWidth(Lcd7920& lcd, PixelNumber offScreenRow, PixelNumber numCols)
{
	if (width == 0)
	{
		FileStore * const fs = reprap.GetPlatform().OpenFile(MENU_DIR, fileName.c_str(), OpenMode::read);
		if (fs != nullptr)
		{
			uint8_t w;
			fs->Read(w);			// read the number of columns
			fs->Close();
			width = w;
		}
	}
}

// End
