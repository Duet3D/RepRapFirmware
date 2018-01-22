/*
 * Menu.cpp
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#include "Menu.h"
#include "ST7920/lcd7920.h"

MenuItem::MenuItem()
{
	// TODO Auto-generated constructor stub

}

void EnumeratedMenu::AddItem(MenuItem *newItem)
{
	MenuItem*& p = items;
	while (p != nullptr)
	{
		p = p->next;
	}
	newItem->next = nullptr;
	p = newItem;
}

// End
