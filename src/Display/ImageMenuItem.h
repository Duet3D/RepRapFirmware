/*
 * ImageMenuItem.h
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_IMAGEMENUITEM_H_
#define SRC_DISPLAY_IMAGEMENUITEM_H_

#include "MenuItem.h"

class ImageMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<ImageMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<ImageMenuItem>(p); }

	ImageMenuItem(PixelNumber r, PixelNumber c, Visibility vis, const char *_ecv_array pFileName) noexcept;

	void Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight, PixelNumber tOffset) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

private:
	String<MaxFilenameLength> fileName;
};

#endif /* SRC_DISPLAY_IMAGEMENUITEM_H_ */
