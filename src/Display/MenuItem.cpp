/*
 * MenuItem.cpp
 *
 *  Created on: 7 May 2018
 *      Author: David
 */

#include "MenuItem.h"

#if SUPPORT_DIRECT_LCD

#include <Platform/RepRap.h>
#include <Heating/Heat.h>
#include <Platform/Platform.h>
#include <GCodes/GCodes.h>
#include <Tools/Tool.h>
#include <PrintMonitor/PrintMonitor.h>

MenuItem::MenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn) noexcept
	: visStr(nullptr), row(r), column(c), width(w), height(0), align(a), fontNumber(fn), visCase(AlwaysVisible), itemChanged(true), highlighted(false), drawn(false), next(nullptr)
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
void MenuItem::PrintAligned(Lcd& lcd, PixelNumber rightMargin) noexcept
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

	lcd.SetCursor(row, column);
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
	const GCodes& gc = reprap.GetGCodes();
	if (visStr != nullptr)
	{
		return gc.EvaluateConditionForDisplay(visStr);
	}

	switch (visCase)
	{
	default:
	case 0:		return true;
	case 2:		return gc.IsReallyPrinting();
	case 3:		return !gc.IsReallyPrinting();
	case 4:		return reprap.GetPrintMonitor().IsPrinting();
	case 5:		return !reprap.GetPrintMonitor().IsPrinting();
	case 6:		{
					const PauseState ps = gc.GetPauseState();
					return ps == PauseState::pausing || ps == PauseState::paused;
				}
	case 7:		return gc.IsReallyPrintingOrResuming();
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
		{
			const auto tool = gc.GetPrimaryMovementState().GetLockedCurrentOrDefaultTool();			// this can be null, especially during startup
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

#endif

// End
