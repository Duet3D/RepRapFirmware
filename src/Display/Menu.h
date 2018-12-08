/*
 * Menu.h
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_MENU_H_
#define SRC_DISPLAY_MENU_H_

#include "RepRapFirmware.h"

#if SUPPORT_12864_LCD

#include "MenuItem.h"

class MessageBox;

// Class to represent either a full page menu or a popup menu.
// For space reasons we store only a single instance of this class. Each nested menu is indented by a fixed margin from its parent.
class Menu
{
public:
	Menu(Lcd7920& refLcd);
	void Load(const char* filename);							// load a menu file
	void Pop();
	void EncoderAction(int action);
	void Refresh();
	void ClearHighlighting();
	void DisplayMessageBox(const MessageBox& mbox);
	void ClearMessageBox();

private:
	void LoadFixedMenu();
	void ResetCache();
	void Reload();
	void DrawAll();
	const char *ParseMenuLine(char * s);
	void LoadError(const char *msg, unsigned int line);
	void AddItem(MenuItem *item, bool isSelectable);
	const char *AppendString(const char *s);

	void EncoderActionEnterItemHelper();
	void EncoderActionScrollItemHelper(int action);
	void EncoderAction_ExecuteHelper(const char *const cmd);

	void AdvanceHighlightedItem(int n);
	MenuItem *FindNextSelectableItem(MenuItem *p) const;
	MenuItem *FindPrevSelectableItem(MenuItem *p) const;

	static const char *SkipWhitespace(const char *s);
	static char *SkipWhitespace(char *s);
	static bool CheckVisibility(MenuItem::Visibility vis);

	static const size_t CommandBufferSize = 2500;
	static const size_t MaxMenuLineLength = 120;				// adjusts behaviour in Reload()
	static const size_t MaxMenuFilenameLength = 18;
	static const size_t MaxMenuNesting = 8;						// maximum number of nested menus
	static const PixelNumber InnerMargin = 2;					// how many pixels we keep clear inside the border
	static const PixelNumber OuterMargin = 8 + InnerMargin;		// how many pixels of the previous menu we leave on each side

	Lcd7920& lcd;

	uint32_t timeoutValue;										// how long to time out after 0 = no timeout
	uint32_t lastActionTime;

	MenuItem *selectableItems;									// selectable items at the innermost level
	MenuItem *unSelectableItems;								// unselectable items at the innermost level
	MenuItem *highlightedItem;									// which item is selected, or nullptr if nothing selected
	String<MaxMenuFilenameLength> filenames[MaxMenuNesting];
	size_t numNestedMenus;
	bool itemIsSelected;
	bool displayingFixedMenu;
	bool displayingErrorMessage;
	bool displayingMessageBox;

	// Variables used while parsing
	size_t commandBufferIndex;
	unsigned int errorColumn;									// column in the current line at which ParseMenuLine hit an error
	MenuItem::FontNumber fontNumber;
	PixelNumber currentMargin;
	PixelNumber row, column;
	PixelNumber rowOffset;

	// Buffer for commands to be executed when the user presses a selected item
	char commandBuffer[CommandBufferSize];
};

#endif

#endif /* SRC_DISPLAY_MENU_H_ */
