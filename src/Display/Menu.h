/*
 * Menu.h
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_MENU_H_
#define SRC_DISPLAY_MENU_H_

#include "RepRapFirmware.h"

class Lcd7920;

class MenuItem
{
public:
	friend class EnumeratedMenu;

protected:
	MenuItem();

private:
	MenuItem *next;
};

class Menu
{
public:
	virtual void Show(Lcd7920& lcd) = 0;

	void SetParent(Menu *p) { parent = p; }

protected:
	Menu(const char *nm) : name(nm) { parent = nullptr; }

	Menu *parent;
	const char *name;
};

class EnumeratedMenu : public Menu
{
public:
	EnumeratedMenu(const char *nm);

	void AddItem(MenuItem *newItem);
	void Show(Lcd7920& lcd) override;

private:
	MenuItem *items;
};

class FilesMenu : public Menu
{
public:
	FilesMenu(const char *nm);

	void Show(Lcd7920& lcd) override;

private:
	static constexpr size_t DisplayedFilenameLength = 20;
	static constexpr size_t MaxFiles = 6;
	String<DisplayedFilenameLength> fileNames[MaxFiles];
	String<MaxFilenameLength> previousFile;					// the last filename displayed on the previous page
};

#endif /* SRC_DISPLAY_MENU_H_ */
