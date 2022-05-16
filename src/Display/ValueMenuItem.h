/*
 * ValueMenuItem.h
 *
 *  Created on: 16 May 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_VALUEMENUITEM_H_
#define SRC_DISPLAY_VALUEMENUITEM_H_

#include "MenuItem.h"

#if SUPPORT_DIRECT_LCD

class ValueMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<ValueMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<ValueMenuItem>(p); }

	ValueMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, Alignment a, FontNumber fn, Visibility vis, bool adj, unsigned int v, unsigned int d) noexcept;
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
	enum class PrintFormat : uint8_t { undefined, asFloat, asUnsigned, asSigned, asPercent, asText, asIpAddress, asTime };

	bool Adjust_SelectHelper() noexcept;
	bool Adjust_AlterHelper(int clicks) noexcept;

	static constexpr PixelNumber DefaultWidth =  25;			// default numeric field width

	const unsigned int valIndex;
	const char *textValue;				// for temporary use when printing

	// Variables currentValue, currentFormat and decimals together define the display format of the item
	union Value
	{	float f;
		uint32_t u;
		int32_t i;
	};

	Value currentValue;
	PrintFormat currentFormat;
	uint8_t decimals;
	AdjustMode adjusting;
	bool adjustable;
	bool error;							// for temporary use when printing
};

#endif

#endif /* SRC_DISPLAY_VALUEMENUITEM_H_ */
