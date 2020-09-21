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
	Menu(Lcd& refLcd) noexcept;
	~Menu();

	void Load(const char* filename) noexcept;							// load a menu file
	void Pop() noexcept;
	void EncoderAction(int action) noexcept;
	void Refresh() noexcept;
	void ClearHighlighting() noexcept;
	void DisplayMessageBox(const MessageBox& mbox) noexcept;
	void ClearMessageBox() noexcept;

private:
	void LoadFixedMenu() noexcept;
	void ResetCache() noexcept;
	void Reload() noexcept;
	void DrawAll() noexcept;
	const char *ParseMenuLine(char * s) noexcept;
	void LoadError(const char *msg, unsigned int line) noexcept;
	void AddItem(MenuItem *item, bool isSelectable) noexcept;
	const char *AppendString(const char *s) noexcept;

	void EncoderActionEnterItemHelper() noexcept;
	void EncoderActionScrollItemHelper(int action) noexcept;
	void EncoderAction_ExecuteHelper(const char *const cmd) noexcept;

	void AdvanceHighlightedItem(int n) noexcept;
	MenuItem *FindNextSelectableItem(MenuItem *p) const noexcept;
	MenuItem *FindPrevSelectableItem(MenuItem *p) const noexcept;

	static const char *SkipWhitespace(const char *s) noexcept;
	static char *SkipWhitespace(char *s) noexcept;
	static bool CheckVisibility(MenuItem::Visibility vis) noexcept;

#ifdef __LPC17xx__
    static const size_t CommandBufferSize = 1024;
#else
    static const size_t CommandBufferSize = 2500;
#endif
	static const size_t MaxMenuLineLength = 120;				// adjusts behaviour in Reload()
	static const size_t MaxMenuFilenameLength = 18;
	static const size_t MaxMenuNesting = 8;						// maximum number of nested menus
	static const PixelNumber InnerMargin = 2;					// how many pixels we keep clear inside the border
	static const PixelNumber OuterMargin = 8 + InnerMargin;		// how many pixels of the previous menu we leave on each side

	Lcd& lcd;

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
