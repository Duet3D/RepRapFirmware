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
	DECLARE_FREELIST_NEW_DELETE(ButtonMenuItem)

	ButtonMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, const char *_ecv_array t, const char *_ecv_array cmd, const char *_ecv_array acFile) noexcept;
	void Draw(Lcd &_ecv_from lcd, PixelNumber maxWidth, bool highlight) noexcept override;
	void UpdateWidthAndHeight(Lcd &_ecv_from lcd) noexcept override;
	bool Select(const StringRef& cmd) noexcept override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept override;

protected:
	void CorePrint(Lcd &_ecv_from lcd) noexcept override;

private:
	const char *_ecv_array text;
	const char *_ecv_array command;
	const char *_ecv_array m_acFile;		// used when action ("command") is "menu"
};

#endif

#endif /* SRC_DISPLAY_BUTTONMENUITEM_H_ */
