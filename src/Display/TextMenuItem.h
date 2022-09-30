/*
 * TextMenuItem.h
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_TEXTMENUITEM_H_
#define SRC_DISPLAY_TEXTMENUITEM_H_

#include "MenuItem.h"

#if SUPPORT_DIRECT_LCD

class TextMenuItem final : public MenuItem
{
public:
	DECLARE_FREELIST_NEW_DELETE(TextMenuItem)

	TextMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, const char *_ecv_array t) noexcept;
	void Draw(Lcd& lcd, PixelNumber maxWidth, bool highlight) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

protected:
	void CorePrint(Lcd& lcd) noexcept override;

private:
	const char *_ecv_array text;
};

#endif

#endif /* SRC_DISPLAY_TEXTMENUITEM_H_ */
