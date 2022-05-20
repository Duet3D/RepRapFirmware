/*
 * MenuItem.h
 *
 *  Created on: 7 May 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_MENUITEM_H_
#define SRC_DISPLAY_MENUITEM_H_

#include "RepRapFirmware.h"

#if SUPPORT_DIRECT_LCD

#include "General/FreelistManager.h"
#include "Lcd/Lcd.h"
#include "Storage/MassStorage.h"

// Menu item class hierarchy
class MenuItem
{
public:
	typedef uint8_t Alignment;
	typedef uint8_t FontNumber;
	typedef uint8_t Visibility;

	static constexpr Alignment LeftAlign = 0, CentreAlign = 1, RightAlign = 2;
	static constexpr Visibility AlwaysVisible = 0;

	// Draw this element on the LCD respecting 'maxWidth' and 'highlight'
	virtual void Draw(Lcd& lcd, PixelNumber maxWidth, bool highlight) noexcept = 0;

	// Select this element with a push of the encoder.
	// If it returns nullptr false go into adjustment mode, if we can adjust the item.
	// If it returns true, execute the command returned via the parameter.
	virtual bool Select(const StringRef& cmd) noexcept { return false; }

	// Actions to be taken when the menu system selects this item
	virtual void Enter(bool forwardDirection) noexcept { };

	// Actions to be taken when the menu system receives encoder counts and this item is currently selected
	// TODO: may be able to merge down with Adjust()
	virtual int Advance(int nCounts) noexcept { return nCounts; }

	// Return true if we can select this element for adjustment
	virtual bool CanAdjust() const noexcept { return false; }

	// Adjust this element, returning true if we have finished adjustment.
	// 'clicks' is the number of encoder clicks to adjust by, or 0 if the button was pushed.
	virtual bool Adjust(int clicks) noexcept { return true; }

	// If the width was specified as zero, update it with the actual width. Also update the height.
	virtual void UpdateWidthAndHeight(Lcd& lcd) noexcept = 0;

	// DC: I don't know what this one is for, the person who wrote it didn't document it
	virtual PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept { return 0; }

	virtual ~MenuItem() noexcept { }

	MenuItem *null GetNext() const noexcept { return next; }
	FontNumber GetFontNumber() const noexcept { return fontNumber; }
	void SetChanged() noexcept { itemChanged = true; }

	// The following functions to set the visibility do not update the display. They should only be called immediately after creating the item and before displaying it for the first time.
	void SetVisibility(Visibility vis) noexcept { visCase = vis; }
	void SetVisibility(const char *_ecv_array vis) noexcept { visStr = vis; }

	bool IsVisible() const noexcept;

	// Erase this item if it is drawn but should not be visible
	void EraseIfInvisible(Lcd& lcd, PixelNumber tOffset) noexcept;

	// Return the width of this item in pixels
	PixelNumber GetWidth() const noexcept { return width; }
	PixelNumber GetHeight() const noexcept { return height; }

	PixelNumber GetMinX() const noexcept { return column; }
	PixelNumber GetMinY() const noexcept { return row; }
	PixelNumber GetMaxX() const noexcept { return column + width - 1; }
	PixelNumber GetMaxY() const noexcept { return row + height - 1; }

	static void AppendToList(MenuItem **root, MenuItem *item) noexcept;

protected:
	MenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn) noexcept;

	// Print the item starting at the current cursor position, which may be off screen. Used to find the width and also to really print the item.
	// Overridden for items that support variable alignment
	virtual void CorePrint(Lcd& lcd) noexcept { }

	// Print the item at the correct place with the correct alignment
	void PrintAligned(Lcd& lcd, PixelNumber rightMargin) noexcept;

	const char *_ecv_array _ecv_null visStr;
	const PixelNumber row, column;
	PixelNumber width, height;
	const Alignment align;
	const FontNumber fontNumber;
	Visibility visCase;

	uint8_t itemChanged : 1,
			highlighted : 1,
			drawn : 1;

private:
	MenuItem *next;
};

#endif

#endif /* SRC_DISPLAY_MENUITEM_H_ */
