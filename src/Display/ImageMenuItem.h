/*
 * ImageMenuItem.h
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_IMAGEMENUITEM_H_
#define SRC_DISPLAY_IMAGEMENUITEM_H_

#include "MenuItem.h"

#if SUPPORT_DIRECT_LCD

class ImageMenuItem final : public MenuItem
{
public:
	DECLARE_FREELIST_NEW_DELETE(ImageMenuItem)

	ImageMenuItem(PixelNumber r, PixelNumber c, const char *_ecv_array pFileName) noexcept;

	void Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

private:
	String<MaxFilenameLength> fileName;
};

#endif

#endif /* SRC_DISPLAY_IMAGEMENUITEM_H_ */
