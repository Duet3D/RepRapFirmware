/*
 * MenuItem.cpp
 *
 *  Created on: 7 May 2018
 *      Author: David
 */

#include "MenuItem.h"

#if SUPPORT_12864_LCD

#include <Platform/RepRap.h>
#include <Heating/Heat.h>
#include <Platform/Platform.h>
#include <GCodes/GCodes.h>
#include <Movement/Move.h>
#include "Display.h"
#include <Tools/Tool.h>
#include <PrintMonitor/PrintMonitor.h>

#ifdef __LPC17xx__
# include "Network.h"
#else
# include <Networking/Network.h>
#endif

MenuItem::MenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis) noexcept
	: row(r), column(c), width(w), height(0), align(a), fontNumber(fn), visCase(vis), itemChanged(true), highlighted(false), drawn(false), next(nullptr)
{
}

/*static*/ void MenuItem::AppendToList(MenuItem **root, MenuItem *item) noexcept
{
	while (*root != nullptr)
	{
		root = &((*root)->next);
	}
	item->next = nullptr;
	*root = item;
}

// Print the item at the correct place with the correct alignment
void MenuItem::PrintAligned(Lcd& lcd, PixelNumber tOffset, PixelNumber rightMargin) noexcept
{
	PixelNumber colsToSkip = 0;
	lcd.SetFont(fontNumber);
	if (align != 0)
	{
		lcd.SetCursor(lcd.GetNumRows(), column);
		lcd.SetRightMargin(min<PixelNumber>(rightMargin, column + width));
		CorePrint(lcd);
		const PixelNumber w = lcd.GetColumn() - column;
		if (w < width)
		{
			colsToSkip = (align == RightAlign)
							? width - w - 1				// when right aligning, leave 1 pixel of space at the end
								: (width - w)/2;
		}
	}

	lcd.SetCursor(row - tOffset, column);
	lcd.SetRightMargin(min<PixelNumber>(rightMargin, column + width));
	lcd.TextInvert(highlighted);
	if (colsToSkip != 0)
	{
		lcd.ClearToMargin();
		lcd.SetCursor(row, column + colsToSkip);
	}
	CorePrint(lcd);
	if (align == 0)
	{
		lcd.ClearToMargin();
	}
	lcd.TextInvert(false);
}

bool MenuItem::IsVisible() const noexcept
{
	switch (visCase)
	{
	default:
	case 0:		return true;
	case 2:		return reprap.GetGCodes().IsReallyPrinting();
	case 3:		return !reprap.GetGCodes().IsReallyPrinting();
	case 4:		return reprap.GetPrintMonitor().IsPrinting();
	case 5:		return !reprap.GetPrintMonitor().IsPrinting();
	case 6:		{
					const PauseState ps = reprap.GetGCodes().GetPauseState();
					return ps == PauseState::pausing || ps == PauseState::paused;
				}
	case 7:		return reprap.GetGCodes().IsReallyPrintingOrResuming();
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	case 10:	return
# if HAS_MASS_STORAGE
					MassStorage::IsDriveMounted(0)
# endif
# if HAS_MASS_STORAGE && HAS_SBC_INTERFACE
					||
# endif
# if HAS_SBC_INTERFACE
					reprap.UsingSbcInterface()
# endif
					;
	case 11:	return
# if HAS_MASS_STORAGE
					!MassStorage::IsDriveMounted(0)
# endif
# if HAS_MASS_STORAGE && HAS_SBC_INTERFACE
					&&
# endif
# if HAS_SBC_INTERFACE
					!reprap.UsingSbcInterface()
# endif
					;
#endif
	case 20:
		{		const auto tool = reprap.GetCurrentOrDefaultTool();			// this can be null, especially during startup
				return tool.IsNotNull() && tool->HasTemperatureFault();
		}
	case 28:	return reprap.GetHeat().GetStatus(reprap.GetHeat().GetBedHeater(0)) == HeaterStatus::fault;
	}
}

// Erase this item if it is drawn but should not be visible
void MenuItem::EraseIfInvisible(Lcd& lcd, PixelNumber tOffset) noexcept
{
	if (drawn && !IsVisible())
	{
		lcd.Clear(row - tOffset, column, row + height, column + width);
		drawn = false;
	}
}

TextMenuItem::TextMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis, const char* t) noexcept
	: MenuItem(r, c, w, a, fn, vis), text(t)
{
}

void TextMenuItem::CorePrint(Lcd& lcd) noexcept
{
	lcd.printf("%s", text);
}

void TextMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) noexcept
{
	// We ignore the 'highlight' parameter because text items are not selectable
	if (IsVisible() && (!drawn || itemChanged))
	{
		PrintAligned(lcd, tOffset, rightMargin);
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

ButtonMenuItem::ButtonMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char* t, const char* cmd, char const* acFile) noexcept
	: MenuItem(r, c, w, CentreAlign, fn, vis), text(t), command(cmd), m_acFile(acFile)
{
}

void ButtonMenuItem::CorePrint(Lcd& lcd) noexcept
{
	lcd.WriteSpaces(1);				// space at start in case highlighted
	lcd.printf("%s", text);
	lcd.WriteSpaces(1);				// space at end to allow for highlighting
}

void ButtonMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) noexcept
{
	if (IsVisible() && (itemChanged || !drawn || highlight != highlighted) && column < lcd.GetNumCols())
	{
		highlighted = highlight;
		PrintAligned(lcd, tOffset, rightMargin);
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

ValueMenuItem::ValueMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis, bool adj, unsigned int v, unsigned int d) noexcept
	: MenuItem(r, c, ((w != 0) ? w : DefaultWidth), a, fn, vis), valIndex(v), currentFormat(PrintFormat::undefined), decimals(d), adjusting(AdjustMode::displaying), adjustable(adj)
{
}

void ValueMenuItem::CorePrint(Lcd& lcd) noexcept
{
	if (adjustable)
	{
		lcd.WriteSpaces(1);
	}

	if (error)
	{
		lcd.printf("***");
	}
	else
	{
		switch (currentFormat)
		{
		case PrintFormat::asFloat:
			lcd.printf("%.*f", decimals, (double)currentValue.f);
			break;

		case PrintFormat::asPercent:
			lcd.printf("%.*f%%", decimals, (double)currentValue.f);
			break;

		case PrintFormat::asUnsigned:
			lcd.printf("%u", currentValue.u);
			break;

		case PrintFormat::asSigned:
			lcd.printf("%d", currentValue.i);
			break;

		case PrintFormat::asText:
			lcd.printf("%s", textValue);
			break;

		case PrintFormat::asIpAddress:
			lcd.printf("%u.%u.%u.%u", currentValue.u & 0x000000FF, (currentValue.u >> 8) & 0x0000000FF, (currentValue.u >> 16) & 0x0000000FF, (currentValue.u >> 24) & 0x0000000FF);
			break;

		case PrintFormat::asTime:
			{
				unsigned int hours = currentValue.u/3600,
					minutes = (currentValue.u / 60) % 60,
					seconds = currentValue.u % 60;
				lcd.printf("%u:%02u:%02u", hours, minutes, seconds);
			}
			break;

		case PrintFormat::undefined:
		default:
			lcd.printf("***");
			break;
		}
	}
}

void ValueMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) noexcept
{
	if (IsVisible())
	{
		error = false;
		textValue = nullptr;

		if (valIndex == 501)
		{
			// Item 501 is a special case because it is text, not a number. We store the current message sequence number in currentValue.
			uint16_t newSeq;
			textValue = reprap.GetLatestMessage(newSeq);
			if (newSeq != currentValue.u)
			{
				itemChanged = true;
				currentValue.u = newSeq;
				currentFormat = PrintFormat::asText;
			}
		}
		else if (adjusting != AdjustMode::adjusting)
		{
			const unsigned int itemNumber = valIndex % 100;
			const Value oldValue = currentValue;
			currentFormat = PrintFormat::asFloat;

			switch (valIndex/100)
			{
			case 0:		// heater current temperature
				currentValue.f = max<float>(reprap.GetGCodes().GetItemCurrentTemperature(itemNumber), 0.0);
				break;

			case 1:		// heater active temperature
				currentValue.f = max<float>(reprap.GetGCodes().GetItemActiveTemperature(itemNumber), 0.0);
				break;

			case 2:		// heater standby temperature
				currentValue.f = max<float>(reprap.GetGCodes().GetItemStandbyTemperature(itemNumber), 0.0);
				break;

			case 3:		// fan %
				currentValue.f = ((itemNumber == 99)
								? reprap.GetGCodes().GetMappedFanSpeed()
								: reprap.GetFansManager().GetFanValue(itemNumber)
							   ) * 100.0;
				currentFormat = PrintFormat::asPercent;
				break;

			case 4:		// extruder %
				currentValue.f = reprap.GetGCodes().GetExtrusionFactor(itemNumber) * 100.0;
				currentFormat = PrintFormat::asPercent;
				break;

			case 5:		// misc
				switch (itemNumber)
				{
				case 0:
					currentValue.f = reprap.GetGCodes().GetSpeedFactor() * 100.0;
					currentFormat = PrintFormat::asPercent;
					break;

				// case 1 is the latest message sent by M117, but it handled at the start

				case 10: // X
				case 11: // Y
				case 12: // Z
				case 13: // U
				case 14: // V
				case 15: // W
					currentValue.f = reprap.GetGCodes().GetUserCoordinate(itemNumber - 10);
					break;

				case 20:
					currentValue.i = reprap.GetCurrentToolNumber();
					currentFormat = PrintFormat::asSigned;
					break;

				case 21:	// Z baby-step
					currentValue.f = reprap.GetGCodes().GetTotalBabyStepOffset(Z_AXIS);
					break;

				// Platform's IP address is the "planned", Network's IP address is the "actual"
				case 30:
				case 31:
				case 32:
				case 33:
					currentValue.u = reprap.GetNetwork().GetIPAddress(0).GetQuad(itemNumber - 30);
					currentFormat = PrintFormat::asUnsigned;
					break;

				case 34:	// IP address in one go
					currentValue.u = reprap.GetNetwork().GetIPAddress(0).GetV4LittleEndian();
					currentFormat = PrintFormat::asIpAddress;
					break;

				case 35:	// Percentage of file that has been processed
					currentValue.f = (reprap.GetPrintMonitor().IsPrinting())
										? reprap.GetPrintMonitor().FractionOfFilePrinted() * 100.0
											: 0;
					currentFormat = PrintFormat::asPercent;
					break;

				case 36:	// Print time remaining, file-based
					currentValue.u = (reprap.GetPrintMonitor().IsPrinting())
										? static_cast<int>(reprap.GetPrintMonitor().EstimateTimeLeft(PrintEstimationMethod::fileBased))
											: 0;
					currentFormat = PrintFormat::asTime;
					break;

				case 37:	// Print time remaining, filament-based
					currentValue.u = (reprap.GetPrintMonitor().IsPrinting())
										? static_cast<int>(reprap.GetPrintMonitor().EstimateTimeLeft(PrintEstimationMethod::filamentBased))
											: 0;
					currentFormat = PrintFormat::asTime;
					break;

				case 38:	// requested speed
					currentValue.f = reprap.GetMove().GetRequestedSpeedMmPerSec();
					break;

				case 39:	// top speed
					currentValue.f = reprap.GetMove().GetTopSpeedMmPerSec();
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
				itemChanged = true;
			}
			else
			{
				switch (currentFormat)
				{
				case PrintFormat::undefined:
					itemChanged = true;
					break;

				case PrintFormat::asFloat:
					if (currentValue.f != oldValue.f)
					{
						itemChanged = true;
					}
					break;

				case PrintFormat::asSigned:
					if (currentValue.i != oldValue.i)
					{
						itemChanged = true;
					}
					break;

				case PrintFormat::asUnsigned:
				case PrintFormat::asIpAddress:
				case PrintFormat::asText:
				case PrintFormat::asTime:
				default:
					if (currentValue.u != oldValue.u)
					{
						itemChanged = true;
					}
					break;
				}
			}
		}

		if (itemChanged || !drawn || (highlight != highlighted))
		{
			highlighted = highlight;
			PrintAligned(lcd, tOffset, rightMargin);
			itemChanged = false;
			drawn = true;
		}
	}
}

bool ValueMenuItem::Select(const StringRef& cmd) noexcept
{
	adjusting = AdjustMode::adjusting;
	return false;
}

void ValueMenuItem::UpdateWidthAndHeight(Lcd& lcd) noexcept
{
	// The width is always set for a ValueMenuItem so we just need to determine the height
	if (height == 0)
	{
		lcd.SetFont(fontNumber);
		height = lcd.GetFontHeight();
	}
}

PixelNumber ValueMenuItem::GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept
{
	// TODO
	return 0;
}

bool ValueMenuItem::Adjust_SelectHelper() noexcept
{
	if (adjusting == AdjustMode::adjusting)
	{
		const unsigned int itemNumber = GetReferencedToolNumber();

		bool err = false;
		switch (valIndex/100)
		{
		case 1: // heater active temperature
			reprap.GetGCodes().SetItemActiveTemperature(itemNumber, (currentValue.f < 1.0) ? ABS_ZERO : currentValue.f);
			break;

		case 2: // heater standby temperature
			reprap.GetGCodes().SetItemStandbyTemperature(itemNumber, (currentValue.f < 1.0) ? ABS_ZERO : currentValue.f);
			break;

		case 3: // fan %
			if (itemNumber == 99)
			{
				reprap.GetGCodes().SetMappedFanSpeed(currentValue.f * 0.01);
			}
			else
			{
				reprap.GetFansManager().SetFanValue(itemNumber, currentValue.f * 0.01);
			}
			break;

		case 4: // extruder %
			reprap.GetGCodes().SetExtrusionFactor(itemNumber, currentValue.f * 0.01);
			break;

		case 5: // misc.
			switch (itemNumber)
			{
			case 0:
				reprap.GetGCodes().SetSpeedFactor(currentValue.f * 0.01);
				break;

			case 20:
				reprap.SelectTool(currentValue.i, false);
				break;

			case 21: // baby stepping
				break;

			default:
				err = true;
				break;
			}
			break;

		default:
			err = true;
			break;
		}

		if (err)
		{
			reprap.GetDisplay().ErrorBeep();
		}
	}

	adjusting = AdjustMode::displaying;
	return true;
}

unsigned int ValueMenuItem::GetReferencedToolNumber() const noexcept
{
	unsigned int uToolNumber = valIndex % 100;
	if (79 == uToolNumber)
	{
		uToolNumber = reprap.GetCurrentOrDefaultTool()->Number();
	}

	return uToolNumber;
}

// Adjust the value of this item by 'clicks' click of the encoder. 'clicks' is nonzero.
// Return true if we have finished adjusting it.
bool ValueMenuItem::Adjust_AlterHelper(int clicks) noexcept
{
	itemChanged = true;			// we will probably change the value, so it will need to be re-displayed
	const unsigned int itemNumber = GetReferencedToolNumber();

	switch (valIndex/100)
	{
	case 1:	// heater active temperature
	case 2:	// heater standby temperature
		if (itemNumber < 80) // Tool heaters
		{
			// If we're decreasing, make any value smaller than 95 go to 0
			// If we're increasing, make any value between 0 and 95 jump directly to 95
			// Also cap the maximum
			if (0 > clicks) // decrementing
			{
				currentValue.f += (float)clicks;
				if (95.0 > currentValue.f)
				{
					currentValue.f = 0.0;
				}
			}
			else // incrementing
			{
				if (0.0 == currentValue.f)
				{
					currentValue.f = 95.0 - 1.0;
				}
				currentValue.f = min<int>(currentValue.f + (float)clicks, reprap.GetHeat().GetHighestTemperatureLimit(reprap.GetTool(itemNumber)->GetHeater(0)));
			}
		}
		else
		{
			currentValue.f += (float)clicks;
		}
		break;

	case 3: // fan %
		currentValue.f = constrain<float>(currentValue.f + (float)clicks, 0.0, 100.0);
		break;

	case 5: // misc.
		switch (itemNumber)
		{
		case 0: // 500 Feed Rate
			currentValue.f = constrain<float>(currentValue.f + (float)clicks, 10.0, 500.0);
			break;

		case 20: // 520 Tool Selection
			currentValue.i = constrain<int>((int)currentValue.f + clicks, -1, 255);
			break;

		case 21: // 521 baby stepping
			{
				String<ShortGCodeLength> cmd;
				cmd.printf("M290 Z%.2f", (double)(0.02 * clicks));
				(void) reprap.GetGCodes().ProcessCommandFromLcd(cmd.c_str());
				adjusting = AdjustMode::liveAdjusting;
			}
			break;

		default:
			if (itemNumber >= 10 && itemNumber < 10 + reprap.GetGCodes().GetVisibleAxes())	// 510-518 axis position adjustment
			{
				String<ShortGCodeLength> cmd;
				const float amount = ((itemNumber == 12) ? 0.02 : 0.1) * clicks;			// 0.02mm Z resolution, 0.1mm for other axes
				cmd.printf("M120 G91 G1 F3000 %c%.2f M121", 'X' + (itemNumber - 10), (double)amount);
				(void) reprap.GetGCodes().ProcessCommandFromLcd(cmd.c_str());
				adjusting = AdjustMode::liveAdjusting;
			}
			break;
		}
		break;

	default:
		currentValue.f += (float)clicks;
		break;
	}

	return false;
}

// Adjust this element, returning true if we have finished adjustment.
// 'clicks' is the number of encoder clicks to adjust by, or 0 if the button was pushed.
bool ValueMenuItem::Adjust(int clicks) noexcept
{
	return (clicks == 0)						// if button has been pressed
			?  Adjust_SelectHelper()
				: Adjust_AlterHelper(clicks);
}

#if HAS_MASS_STORAGE
FilesMenuItem::FilesMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char *cmd, const char *dir, const char *acFile, unsigned int nf) noexcept
	: MenuItem(r, c, w, LeftAlign, fn, vis), numDisplayLines(nf), command(cmd), initialDirectory(dir), m_acFile(acFile),
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

void FilesMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) noexcept
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
			ListFiles(lcd, rightMargin, highlight, tOffset);
			break;

		case error:
			break;
		}
	}
}

void FilesMenuItem::ListFiles(Lcd& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) noexcept
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

// Image menu item members
// The image file format is:
// Byte 0 = number of columns
// Byte 1 = number of rows
// Remaining bytes = data, 1 row at a time. If the number of columns is not a multiple of 8 then the data for each row is padded to a multiple of 8 bits.
ImageMenuItem::ImageMenuItem(PixelNumber r, PixelNumber c, Visibility vis, const char *pFileName) noexcept
	: MenuItem(r, c, 0, 0, 0, vis)
{
	fileName.copy(pFileName);
}

void ImageMenuItem::Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) noexcept
{
	if (IsVisible() && (!drawn || itemChanged || highlight != highlighted))
	{
		FileStore * const fs = reprap.GetPlatform().OpenFile(MENU_DIR, fileName.c_str(), OpenMode::read);
		if (fs != nullptr)
		{
			uint8_t widthAndHeight[2];
			if (fs->Read(widthAndHeight, 2) == 2)
			{
				const PixelNumber cols = widthAndHeight[0];
				const PixelNumber rows = widthAndHeight[1];
				if (cols != 0 && cols <= lcd.GetNumCols() && rows != 0)
				{
					const size_t bytesPerRow = (cols + 7)/8;
					for (PixelNumber irow = 0; irow < rows; ++irow)
					{
						uint8_t buffer[lcd.GetNumCols()/8];
						if (fs->Read(buffer, bytesPerRow) != (int)bytesPerRow)
						{
							break;
						}
						lcd.BitmapRow(row - tOffset + irow, column,  cols, buffer, highlight);
					}
				}
			}
			fs->Close();
		}
		itemChanged = false;
		drawn = true;
		highlighted = highlight;
	}
}

void ImageMenuItem::UpdateWidthAndHeight(Lcd& lcd) noexcept
{
	if (width == 0 || height == 0)
	{
		FileStore * const fs = reprap.GetPlatform().OpenFile(MENU_DIR, fileName.c_str(), OpenMode::read);
		if (fs != nullptr)
		{
			uint8_t w[2];
			fs->Read(w, 2);			// read the number of columns
			fs->Close();
			width = w[0];
			height = w[1];
		}
	}
}

#endif

// End
