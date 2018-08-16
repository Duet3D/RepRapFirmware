/*
 * MenuItem.h
 *
 *  Created on: 7 May 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_MENUITEM_H_
#define SRC_DISPLAY_MENUITEM_H_

#include "Libraries/General/FreelistManager.h"
#include "RepRapFirmware.h"
#include "ST7920/lcd7920.h"
#include "Storage/MassStorage.h"

// Menu item class hierarchy
class MenuItem
{
public:
	typedef uint8_t FontNumber;
	typedef uint8_t Visibility;
	typedef bool (*CheckFunction) (uint8_t);

	// Draw this element on the LCD respecting 'maxWidth' and 'highlight'
	virtual void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight, PixelNumber tOffset) = 0;

	// Select this element with a push of the encoder.
	// If it returns nullptr then go into adjustment mode.
	// Else execute the returned command.
	virtual const char* Select() = 0;

	virtual bool Visible() const { return true; }

	// Actions to be taken when the menu system selects this item
	virtual void Enter(bool bForwardDirection) {};

	// Actions to be taken when the menu system receives encoder counts
	// and this item is currently selected
	// TODO: may be able to merge down with Adjust()
	virtual int Advance(int nCounts) { return nCounts; }

	// Adjust this element, returning true if we have finished adjustment.
	// 'clicks' is the number of encoder clicks to adjust by, or 0 if the button was pushed.
	virtual bool Adjust(int clicks) { return true; }
	virtual bool CanAdjust() { return true; }

	virtual ~MenuItem() { }

	MenuItem *GetNext() const { return next; }
	FontNumber GetFontNumber() const { return fontNumber; }

	virtual PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, const LcdFont *oFont) { return 0; }

	static void AppendToList(MenuItem **root, MenuItem *item);

protected:
	MenuItem(PixelNumber r, PixelNumber c, FontNumber fn);

	const PixelNumber row, column;
	const FontNumber fontNumber;

private:
	MenuItem *next;
};

// TODO: this could be removed if we disallow text usage on scrolling screens
class TextMenuItem : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<TextMenuItem>(); }
	void operator delete(void* p) { Release<TextMenuItem>(p); }

	TextMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, Visibility xVis, CheckFunction bF, const char *t);
	bool Visible() const override;
	void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight, PixelNumber tOffset) override;

	const char* Select() override;

private:
	static TextMenuItem *freelist;

	const char *text;

	const Visibility m_xVisCase;
	const CheckFunction m_bF;
};

class ButtonMenuItem : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<ButtonMenuItem>(); }
	void operator delete(void* p) { Release<ButtonMenuItem>(p); }

	ButtonMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, Visibility xVis, CheckFunction bF, const char *t, const char *cmd, const char *acFile);
	bool Visible() const override;
	void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight, PixelNumber tOffset) override;
	const char* Select() override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, const LcdFont *oFont) override;

private:
	static ButtonMenuItem *freelist;

	const char *text;
	const char *command;
	const char *m_acFile; // used when action ("command") is "menu"

	const Visibility m_xVisCase;
	const CheckFunction m_bF;

	// Scratch -- consumer is required to use as soon as it's returned
	// NOT THREAD SAFE!
	char m_acCommand[MaxFilenameLength + 20]; // TODO fix to proper max length
};

class ValueMenuItem : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<ValueMenuItem>(); }
	void operator delete(void* p) { Release<ValueMenuItem>(p); }

	ValueMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, PixelNumber w, unsigned int v, unsigned int d);
	void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight, PixelNumber tOffset) override;
	const char* Select() override;
	bool Adjust(int clicks) override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, const LcdFont *oFont) override;

	unsigned int GetReferencedToolNumber();

private:
	bool Adjust_SelectHelper();
	bool Adjust_AlterHelper(int clicks);

	const unsigned int valIndex; // TODO: strong case for subclassing here -- each subclass can have a desired behavior
	float currentValue;
	PixelNumber width;
	uint8_t decimals;
	bool adjusting;
};

class FilesMenuItem : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<FilesMenuItem>(); }
	void operator delete(void* p) { Release<FilesMenuItem>(p); }

	FilesMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, const char *cmd, const char *dir, const char *acFile, unsigned int nf, unsigned int uFontHeight);
	void Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) override;
	void Enter(bool bForwardDirection) override;
	int Advance(int nCounts) override;
	const char* Select() override;
	bool CanAdjust() override { return false; }

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, const LcdFont *oFont) override;

	void EnterDirectory();

	/* bool ShowBasedOnPrinterState() { return true; } */

protected:
	void vResetViewState();

private:
	static FilesMenuItem *freelist;

	const char *command;
	const char *initialDirectory;
	const char *m_acFile; // used when action ("command") includes "menu"

	const unsigned int m_uDisplayLines;
	const unsigned int m_uFontHeight; // TODO: base has access to font number, is this necessary?  API within other class may need to change to accommodate...

	// Working
	char m_acCurrentDirectory[MaxFilenameLength];

	bool bInSubdirectory() const;
	unsigned int uListingEntries() const;

	// Scratch -- consumer is required to use as soon as it's returned
	// NOT THREAD SAFE!
	char m_acCommand[MaxFilenameLength + 20]; // TODO fix to proper max length

	// Files on the file system, real count i.e. no ".." included
	unsigned int m_uHardItemsInDirectory;

	// Logical items (c. files) for display, referenced to uListingEntries() count
	unsigned int m_uListingFirstVisibleIndex;
	unsigned int m_uListingSelectedIndex;

	MassStorage *const m_oMS;
};

#endif /* SRC_DISPLAY_MENUITEM_H_ */

