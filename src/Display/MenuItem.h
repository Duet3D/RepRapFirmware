/*
 * MenuItem.h
 *
 *  Created on: 7 May 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_MENUITEM_H_
#define SRC_DISPLAY_MENUITEM_H_

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

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

	MenuItem *GetNext() const noexcept { return next; }
	FontNumber GetFontNumber() const noexcept { return fontNumber; }
	void SetChanged() noexcept { itemChanged = true; }
	bool IsVisible() const noexcept;

	// Erase this item if it is drawn but should not be visible
	void EraseIfInvisible(Lcd& lcd, PixelNumber tOffset) noexcept;

	// Return the width of this item in pixels
	PixelNumber GetWidth() const noexcept { return width; }
	PixelNumber GetHeight() const noexcept { return height; }

	static void AppendToList(MenuItem **root, MenuItem *item) noexcept;

protected:
	MenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility v) noexcept;

	// Print the item starting at the current cursor position, which may be off screen. Used to find the width and also to really print the item.
	// Overridden for items that support variable alignment
	virtual void CorePrint(Lcd& lcd) noexcept { }

	// Print the item at the correct place with the correct alignment
	void PrintAligned(Lcd& lcd, PixelNumber rightMargin) noexcept;

	const PixelNumber row, column;
	PixelNumber width, height;
	const Alignment align;
	const FontNumber fontNumber;
	const Visibility visCase;

	bool itemChanged;
	bool highlighted;
	bool drawn;

private:
	MenuItem *next;
};

class TextMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<TextMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<TextMenuItem>(p); }

	TextMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis, const char *t) noexcept;
	void Draw(Lcd& lcd, PixelNumber maxWidth, bool highlight) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

protected:
	void CorePrint(Lcd& lcd) noexcept override;

private:
	const char *text;
};

class ButtonMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<ButtonMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<ButtonMenuItem>(p); }

	ButtonMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char *t, const char *cmd, const char *acFile) noexcept;
	void Draw(Lcd& lcd, PixelNumber maxWidth, bool highlight) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;
	bool Select(const StringRef& cmd) noexcept override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept override;

protected:
	void CorePrint(Lcd& lcd) noexcept override;

private:
	const char *text;
	const char *command;
	const char *m_acFile; // used when action ("command") is "menu"
};

class ValueMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<ValueMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<ValueMenuItem>(p); }

	ValueMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis, bool adj, unsigned int v, unsigned int d) noexcept;
	void Draw(Lcd& lcd, PixelNumber maxWidth, bool highlight) noexcept override;
	bool Select(const StringRef& cmd) noexcept override;
	bool CanAdjust() const noexcept override { return true; }
	bool Adjust(int clicks) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept override;

	unsigned int GetReferencedToolNumber() const noexcept;

protected:
	void CorePrint(Lcd& lcd) noexcept override;

private:
	enum class AdjustMode : uint8_t { displaying, adjusting, liveAdjusting };
	enum class PrintFormat : uint8_t { undefined, asFloat, asUnsigned, asSigned, asPercent, asText, asIpAddress, asTime };

	bool Adjust_SelectHelper() noexcept;
	bool Adjust_AlterHelper(int clicks) noexcept;

	static constexpr PixelNumber DefaultWidth =  25;			// default numeric field width

	const unsigned int valIndex;
	const char *textValue;				// for temporary use when printing

	// Variables currentValue, currentFormat and decimals together define the display format of the item
	union Value
	{	float f;
		uint32_t u;
		int32_t i;
	};

	Value currentValue;
	PrintFormat currentFormat;
	uint8_t decimals;
	AdjustMode adjusting;
	bool adjustable;
	bool error;							// for temporary use when printing
};

#if HAS_MASS_STORAGE
class FilesMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<FilesMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<FilesMenuItem>(p); }

	FilesMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char *cmd, const char *dir, const char *acFile, unsigned int nf) noexcept;
	void Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept override;
	void Enter(bool bForwardDirection) noexcept override;
	int Advance(int nCounts) noexcept override;
	bool Select(const StringRef& cmd) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept override;

	void EnterDirectory() noexcept;

protected:
	void vResetViewState() noexcept;

private:
	void ListFiles(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept;
	uint8_t GetDirectoryNesting() const noexcept;

	const unsigned int numDisplayLines;

	const char *command;
	const char *initialDirectory;
	const char *m_acFile; // used when action ("command") includes "menu"

	// Working
	String<MaxFilenameLength> currentDirectory;

	bool bInSubdirectory() const noexcept;
	unsigned int uListingEntries() const noexcept;

	// Files on the file system, real count i.e. no ".." included
	unsigned int m_uHardItemsInDirectory;

	// Logical items (c. files) for display, referenced to uListingEntries() count
	unsigned int m_uListingFirstVisibleIndex;
	unsigned int m_uListingSelectedIndex;

	enum CardState : uint8_t { notStarted, mounting, mounted, error } sdCardState;
	uint8_t initialDirectoryNesting;
};
#endif

class ImageMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<ImageMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<ImageMenuItem>(p); }

	ImageMenuItem(PixelNumber r, PixelNumber c, Visibility vis, const char *pFileName) noexcept;

	void Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

private:
	String<MaxFilenameLength> fileName;
};

#endif

#endif /* SRC_DISPLAY_MENUITEM_H_ */
