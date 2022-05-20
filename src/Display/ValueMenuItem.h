/*
 * ValueMenuItem.h
 *
 *  Created on: 25 Apr 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_VALUEMENUITEM_H_
#define SRC_DISPLAY_VALUEMENUITEM_H_

#include "MenuItem.h"

#if SUPPORT_DIRECT_LCD

#include <ObjectModel/ObjectModel.h>

class ValueMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<ValueMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<ValueMenuItem>(p); }

	ValueMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, bool adj, const char *_ecv_array _ecv_null om, unsigned int v, unsigned int d) noexcept;
	void Draw(Lcd& lcd, PixelNumber maxWidth, bool highlight) noexcept override;
	bool Select(const StringRef& cmd) noexcept override;
	bool CanAdjust() const noexcept override { return true; }
	bool Adjust(int clicks) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept override;

	unsigned int GetReferencedToolNumber() const noexcept;

protected:
	void CorePrint(Lcd& lcd) noexcept override;

private:
	enum class AdjustMode : uint8_t { displaying, adjusting, liveAdjusting };

	bool Adjust_SelectHelper() noexcept;
	bool Adjust_AlterHelper(int clicks) noexcept;

	static constexpr PixelNumber DefaultWidth =  25;			// default numeric field width

	ExpressionValue currentValue;								// the last value fetched for the item
	const char *_ecv_array _ecv_null const omText;				// an object model expression to fetch the item value (optional)
	const unsigned int valIndex;								// the item index, if no object model expression was provided
	const char *_ecv_array _ecv_null textValue;					// for temporary use when printing

	const uint8_t decimals : 4,
				  adjustable : 1;
	AdjustMode adjusting;
	uint8_t error : 1,											// for temporary use when printing
			asPercent : 1;										// true if we print this as a percentage
};


#endif

#endif /* SRC_DISPLAY_VALUEMENUITEM_H_ */
