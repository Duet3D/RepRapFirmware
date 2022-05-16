/*
 * ButtonMenuItem.h
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_BUTTONMENUITEM_H_
#define SRC_DISPLAY_BUTTONMENUITEM_H_

#include "MenuItem.h"

#if SUPPORT_DIRECT_LCD

class ButtonMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<ButtonMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<ButtonMenuItem>(p); }

	ButtonMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, const char *t, const char *cmd, const char *acFile) noexcept;
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

#endif

#endif /* SRC_DISPLAY_BUTTONMENUITEM_H_ */
