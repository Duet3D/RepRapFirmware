/*
 * FilesMenuItem.cpp
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#include "FilesMenuItem.h"

#if SUPPORT_DIRECT_LCD && HAS_MASS_STORAGE

FilesMenuItem::FilesMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, const char *_ecv_array cmd, const char *_ecv_array dir, const char *_ecv_array acFile, unsigned int nf) noexcept
	: MenuItem(r, c, w, LeftAlign, fn), numDisplayLines(nf), command(cmd), initialDirectory(dir), m_acFile(acFile),
        m_uListingFirstVisibleIndex(0), m_uListingSelectedIndex(0)
{
	// There's no guarantee that initialDirectory has a trailing '/'
	currentDirectory.copy(initialDirectory);
	const size_t len = currentDirectory.strlen();
	if (len == 0 || '/' != currentDirectory[len - 1])
	{
		currentDirectory.cat('/');
	}

	initialDirectoryNesting = GetDirectoryNesting();
	sdCardState = notStarted;
}

void FilesMenuItem::vResetViewState() noexcept
{
	m_uListingSelectedIndex = 0;
	m_uListingFirstVisibleIndex = 0;
}

void FilesMenuItem::EnterDirectory() noexcept
{
	vResetViewState();

	m_uHardItemsInDirectory = 0;
	FileInfo oFileInfo;
	if (MassStorage::FindFirst(currentDirectory.c_str(), oFileInfo))
	{
		do
		{
			if (oFileInfo.fileName[0] != '.')
			{
				++m_uHardItemsInDirectory;
			}
		}
		while (MassStorage::FindNext(oFileInfo));
	}

	itemChanged = true;							// force a redraw
}

uint8_t FilesMenuItem::GetDirectoryNesting() const noexcept
{
	const char *pcPathElement = currentDirectory.c_str();
	uint8_t uNumSlashes = 0;

	while ('\0' != *pcPathElement)
	{
		if (('/' == *pcPathElement) && ('\0' != *(pcPathElement + 1))) // don't count a trailing slash
		{
			++uNumSlashes;
		}
		++pcPathElement;
	}
	return uNumSlashes;
}

bool FilesMenuItem::bInSubdirectory() const noexcept
{
	return GetDirectoryNesting() > initialDirectoryNesting;
}

unsigned int FilesMenuItem::uListingEntries() const noexcept
{
	return bInSubdirectory() ? (1 + m_uHardItemsInDirectory) : m_uHardItemsInDirectory;
}

void FilesMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept
{
	// The 'highlight' parameter is not used to highlight this item, but it is still used to tell whether this item is selected or not
	if (!IsVisible())
	{
		sdCardState = notStarted;
	}
	else if (!drawn || itemChanged || highlighted != highlight)
	{
		switch (sdCardState)
		{
		case notStarted:
			if (MassStorage::CheckDriveMounted(currentDirectory.c_str()))
			{
				sdCardState = mounted;
				EnterDirectory();
			}
			else
			{
				sdCardState = mounting;
			}
			break;

		case mounting:
			{
				const size_t card = (isdigit(currentDirectory[0]) && currentDirectory[1] == ':') ? currentDirectory[0] - '0' : 0;
				String<StringLength50> reply;
				switch(MassStorage::Mount(card, reply.GetRef(), false))
				{
				case GCodeResult::notFinished:
					return;

				case GCodeResult::ok:
					sdCardState = mounted;
					EnterDirectory();
					break;

				default:
					reply.copy("Internal error");
					// no break
				case GCodeResult::error:
					sdCardState = error;
					lcd.SetFont(fontNumber);
					lcd.SetCursor(row, column);
					lcd.SetRightMargin(rightMargin);
					lcd.ClearToMargin();
					lcd.SetCursor(row, column);
					lcd.printf("%s", reply.c_str());
					break;
				}
			}
			break;

		case mounted:
			ListFiles(lcd, rightMargin, highlight);
			break;

		case error:
			break;
		}
	}
}

void FilesMenuItem::ListFiles(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept
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
			lcd.printf("  ..");
			lcd.ClearToMargin();
			if (highlight && m_uListingSelectedIndex == 0)
			{
				// Overwriting the initial spaces with '>' avoids shifting the following text when we change the selection
				lcd.SetCursor(row, column);
				lcd.write('>');
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
	bool gotFileInfo = MassStorage::FindFirst(currentDirectory.c_str(), oFileInfo);
	while (gotFileInfo)
	{
		if (oFileInfo.fileName[0] != '.')
		{
			if (dirEntriesToSkip == 0)
			{
				break;
			}
			--dirEntriesToSkip;
		}
		gotFileInfo = MassStorage::FindNext(oFileInfo);
	}

	// We always iterate the entire viewport so that old listing lines that may not be overwritten are cleared
	while (line < numDisplayLines)
	{
		lcd.SetCursor(row + (lcd.GetFontHeight() * line), column);

		// If there's actually a file to describe (not just ensuring viewport line clear)
		if (gotFileInfo)
		{
			lcd.printf("  ");
			if (oFileInfo.isDirectory)
			{
				lcd.printf("./");
			}
			lcd.printf("%s", oFileInfo.fileName.c_str());
			lcd.ClearToMargin();
			if (highlight && m_uListingSelectedIndex == line + m_uListingFirstVisibleIndex)
			{
				lcd.SetCursor(row + (lcd.GetFontHeight() * line), column);
				lcd.write('>');
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

		do
		{
			gotFileInfo = MassStorage::FindNext(oFileInfo);
		} while (gotFileInfo && oFileInfo.fileName[0] == '.');
	}

	MassStorage::AbandonFindNext();				// release the mutex, there may be more files that we don't have room to display

	itemChanged = false;
	drawn = true;
	highlighted = highlight;
}

void FilesMenuItem::Enter(bool bForwardDirection) noexcept
{
	if (bForwardDirection || uListingEntries() == 0)
	{
		m_uListingSelectedIndex = 0;
		m_uListingFirstVisibleIndex = 0;					// select the first item and start the list from the first item
	}
	else
	{
		m_uListingSelectedIndex = uListingEntries() - 1;	// select the last item
		m_uListingFirstVisibleIndex = ((uListingEntries() > numDisplayLines) ? (uListingEntries() - numDisplayLines) : 0);
	}
	itemChanged = true;
}

int FilesMenuItem::Advance(int nCounts) noexcept
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

bool FilesMenuItem::Select(const StringRef& cmd) noexcept
{
	// Several cases:
	// TEST 1. ".." entry - call EnterDirectory(), using saved state information
	// TEST 2. Directory - call EnterDirectory(), adding to saved state information
	// 3. File - run command with filename as argument

	// Get information on the item selected

	if (bInSubdirectory() && 0 == m_uListingSelectedIndex) // meaning ".."
	{
		// TODO: go up one level rather than to logical root
		// There's no guarantee that initialDirectory has a trailing '/'
		currentDirectory.copy(initialDirectory);
		const size_t len = currentDirectory.strlen();
		if (len == 0 || '/' != currentDirectory[len - 1])
		{
			currentDirectory.cat('/');
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
		bool gotFileInfo = MassStorage::FindFirst(currentDirectory.c_str(), oFileInfo);
		while (gotFileInfo)
		{
			if (oFileInfo.fileName[0] != '.')
			{
				if (dirEntriesToSkip == 0)
				{
					break;
				}
				--dirEntriesToSkip;
			}
			gotFileInfo = MassStorage::FindNext(oFileInfo);
		}
		MassStorage::AbandonFindNext();

		if (gotFileInfo)	// handles empty directory (no action)
		{
			if (oFileInfo.isDirectory)
			{
				// Build the new directory, and ensure it's terminated with a forward slash
				currentDirectory.cat(oFileInfo.fileName.c_str());
				currentDirectory.cat('/');
				EnterDirectory();
			}
			else
			{
				int nReplacementIndex = StringContains(command, "#0");
				if (nReplacementIndex != -1)
				{
					cmd.copy(command, nReplacementIndex);
					cmd.cat('"');
					cmd.cat(currentDirectory.c_str());
					cmd.cat(oFileInfo.fileName.c_str());
					cmd.cat('"');
					cmd.cat(command + nReplacementIndex + strlen("#0"));
				}
				else
				{
					cmd.copy(command);
				}

				// TODO: do this on the way in and it might be less work...
				//   On the other hand, this only occurs when an item is selected so it's O(1) vs. O(n)
				nReplacementIndex = cmd.Contains("menu");
				if (nReplacementIndex != -1)
				{
					cmd.Truncate(nReplacementIndex);
					cmd.cat(m_acFile);
				}

				return true;
			}
		}
	}

	return false;
}

void FilesMenuItem::UpdateWidthAndHeight(Lcd& lcd) noexcept
{
	// The width is always set for a FilesMenuItem so we just need to determine the height
	if (height == 0)
	{
		lcd.SetFont(fontNumber);
		height = lcd.GetFontHeight() * numDisplayLines;
	}
}

PixelNumber FilesMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept
{
	// TODO
	return 0;
}

#endif

// End
