/*
 * TextMenuItem.h
 *
 *  Created on: 16 May 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_TEXTMENUITEM_H_
#define SRC_DISPLAY_TEXTMENUITEM_H_

#include "MenuItem.h"

#if SUPPORT_DIRECT_LCD

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

#endif

#endif /* SRC_DISPLAY_TEXTMENUITEM_H_ */
