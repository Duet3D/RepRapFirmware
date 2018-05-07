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
#include "ST7920/Lcd7920.h"

// Menu item class hierarchy
class MenuItem
{
public:
	friend class Menu;
	typedef uint8_t FontNumber;

	virtual void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight) = 0;
	virtual ~MenuItem() { }

protected:
	MenuItem(PixelNumber r, PixelNumber c, FontNumber fn);

	PixelNumber row, column;
	FontNumber fontNumber;

private:
	MenuItem *next;
};

class ButtonMenuItem : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<ButtonMenuItem>(); }
	void operator delete(void* p) { Release<ButtonMenuItem>(p); }

	ButtonMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, const char *t, const char *cmd);
	void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight) override;

private:
	static ButtonMenuItem *freelist;

	const char *text;
	const char *command;
};

class ValueMenuItem : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<ValueMenuItem>(); }
	void operator delete(void* p) { Release<ValueMenuItem>(p); }

	ValueMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, PixelNumber w, unsigned int v, unsigned int d);
	void Draw(Lcd7920& lcd, PixelNumber maxWidth, bool highlight) override;

private:
	unsigned int valIndex;
	PixelNumber width;
	uint8_t decimals;
};

class FilesMenuItem : public MenuItem
{
public:
	void* operator new(size_t sz) { return Allocate<FilesMenuItem>(); }
	void operator delete(void* p) { Release<FilesMenuItem>(p); }

	FilesMenuItem(PixelNumber r, PixelNumber c, FontNumber fn, const char *cmd, const char *dir, unsigned int nf);
	void Draw(Lcd7920& lcd, PixelNumber rightMargin, bool highlight) override;

private:
	static FilesMenuItem *freelist;

	const char *command;
	const char *initialDirectory;
	uint8_t numFiles;
};

#endif /* SRC_DISPLAY_MENUITEM_H_ */
