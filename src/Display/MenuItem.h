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
#include "ST7920/lcd7920.h"
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
	virtual void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight, PixelNumber tOffset) = 0;

	// Select this element with a push of the encoder.
	// If it returns nullptr false go into adjustment mode, if we can adjust the item.
	// If it returns true, execute the command returned via the parameter.
	virtual bool Select(const StringRef& cmd) { return false; }

	// Actions to be taken when the menu system selects this item
	virtual void Enter(bool forwardDirection) {};

	// Actions to be taken when the menu system receives encoder counts and this item is currently selected
	// TODO: may be able to merge down with Adjust()
	virtual int Advance(int nCounts) { return nCounts; }

	// Return true if we can select this element for adjustment
	virtual bool CanAdjust() { return false; }

	// Adjust this element, returning true if we have finished adjustment.
	// 'clicks' is the number of encoder clicks to adjust by, or 0 if the button was pushed.
	virtual bool Adjust(int clicks) { return true; }

	// If the width was specified as zero, update it with the actual width. Also update the height.
	virtual void UpdateWidthAndHeight(Lcd7920& lcd) = 0;

	// DC: I don't know what this one is for, the person who wrote it didn't document it
	virtual PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const { return 0; }

	virtual ~MenuItem() { }

	MenuItem *GetNext() const { return next; }
	FontNumber GetFontNumber() const { return fontNumber; }
	void SetChanged() { itemChanged = true; }
	bool IsVisible() const;

	// Erase this item if it is drawn but should not be visible
	void EraseIfInvisible(Lcd7920& lcd, PixelNumber tOffset);

	// Return the width of this item in pixels
	PixelNumber GetWidth() const { return width; }
	PixelNumber GetHeight() const { return height; }

	static void AppendToList(MenuItem **root, MenuItem *item);

protected:
	MenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility v);

	// Print the item starting at the current cursor position, which may be off screen. Used to find the width and also to really print the item.
	// Overridden for items that support variable alignment
	virtual void CorePrint(Lcd7920& lcd) { }

	// Print the item at the correct place with the correct alignment
	void PrintAligned(Lcd7920& lcd, PixelNumber tOffset, PixelNumber rightMargin);

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
	void* operator new(size_t sz) { return Allocate<TextMenuItem>(); }
	void operator delete(void* p) { Release<TextMenuItem>(p); }

	TextMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis, const char *t);
	void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight, PixelNumber tOffset) override;
	void UpdateWidthAndHeight(Lcd7920& lcd) override;

protected:
	void CorePrint(Lcd7920& lcd) override;

private:
	const char *text;
};

class ButtonMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<ButtonMenuItem>(); }
	void operator delete(void* p) { Release<ButtonMenuItem>(p); }

	ButtonMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char *t, const char *cmd, const char *acFile);
	void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight, PixelNumber tOffset) override;
	void UpdateWidthAndHeight(Lcd7920& lcd) override;
	bool Select(const StringRef& cmd) override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const override;

protected:
	void CorePrint(Lcd7920& lcd) override;

private:
	const char *text;
	const char *command;
	const char *m_acFile; // used when action ("command") is "menu"
};

class ValueMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<ValueMenuItem>(); }
	void operator delete(void* p) { Release<ValueMenuItem>(p); }

	ValueMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis, bool adj, unsigned int v, unsigned int d);
	void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight, PixelNumber tOffset) override;
	bool Select(const StringRef& cmd) override;
	bool CanAdjust() override { return true; }
	bool Adjust(int clicks) override;
	void UpdateWidthAndHeight(Lcd7920& lcd) override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const override;

	unsigned int GetReferencedToolNumber() const;

protected:
	void CorePrint(Lcd7920& lcd) override;

private:
	enum class AdjustMode : uint8_t { displaying, adjusting, liveAdjusting };
	enum class PrintFormat : uint8_t { undefined, asFloat, asUnsigned, asSigned, asPercent, asText, asIpAddress, asTime };

	bool Adjust_SelectHelper();
	bool Adjust_AlterHelper(int clicks);

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

class FilesMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<FilesMenuItem>(); }
	void operator delete(void* p) { Release<FilesMenuItem>(p); }

	FilesMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char *cmd, const char *dir, const char *acFile, unsigned int nf);
	void Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) override;
	void Enter(bool bForwardDirection) override;
	int Advance(int nCounts) override;
	bool Select(const StringRef& cmd) override;
	void UpdateWidthAndHeight(Lcd7920& lcd) override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const override;

	void EnterDirectory();

protected:
	void vResetViewState();

private:
	void ListFiles(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset);
	uint8_t GetDirectoryNesting() const;

	const unsigned int numDisplayLines;

	const char *command;
	const char *initialDirectory;
	const char *m_acFile; // used when action ("command") includes "menu"

	// Working
	String<MaxFilenameLength> currentDirectory;

	bool bInSubdirectory() const;
	unsigned int uListingEntries() const;

	// Files on the file system, real count i.e. no ".." included
	unsigned int m_uHardItemsInDirectory;

	// Logical items (c. files) for display, referenced to uListingEntries() count
	unsigned int m_uListingFirstVisibleIndex;
	unsigned int m_uListingSelectedIndex;

	MassStorage *const m_oMS;

	enum CardState : uint8_t { notStarted, mounting, mounted, error } sdCardState;
	uint8_t initialDirectoryNesting;
};

class ImageMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<ImageMenuItem>(); }
	void operator delete(void* p) { Release<ImageMenuItem>(p); }

	ImageMenuItem(PixelNumber r, PixelNumber c, Visibility vis, const char *pFileName);

	void Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) override;
	void UpdateWidthAndHeight(Lcd7920& lcd) override;

private:
	String<MaxFilenameLength> fileName;
};

#endif

#endif /* SRC_DISPLAY_MENUITEM_H_ */
