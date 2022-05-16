/*
 * FilesMenuItem.h
 *
 *  Created on: 16 May 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_FILESMENUITEM_H_
#define SRC_DISPLAY_FILESMENUITEM_H_

#include "MenuItem.h"

#if SUPPORT_DIRECT_LCD && HAS_MASS_STORAGE

class FilesMenuItem final : public MenuItem
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<FilesMenuItem>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<FilesMenuItem>(p); }

	FilesMenuItem(PixelNumber r, PixelNumber c, PixelNumber w, FontNumber fn, Visibility vis, const char *cmd, const char *dir, const char *acFile, unsigned int nf) noexcept;
	void Draw(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept override;
	void Enter(bool bForwardDirection) noexcept override;
	int Advance(int nCounts) noexcept override;
	bool Select(const StringRef& cmd) noexcept override;
	void UpdateWidthAndHeight(Lcd& lcd) noexcept override;

	PixelNumber GetVisibilityRowOffset(PixelNumber tCurrentOffset, PixelNumber fontHeight) const noexcept override;

	void EnterDirectory() noexcept;

protected:
	void vResetViewState() noexcept;

private:
	void ListFiles(Lcd& lcd, PixelNumber rightMargin, bool highlight) noexcept;
	uint8_t GetDirectoryNesting() const noexcept;

	const unsigned int numDisplayLines;

	const char *command;
	const char *initialDirectory;
	const char *m_acFile; // used when action ("command") includes "menu"

	// Working
	String<MaxFilenameLength> currentDirectory;

	bool bInSubdirectory() const noexcept;
	unsigned int uListingEntries() const noexcept;

	// Files on the file system, real count i.e. no ".." included
	unsigned int m_uHardItemsInDirectory;

	// Logical items (c. files) for display, referenced to uListingEntries() count
	unsigned int m_uListingFirstVisibleIndex;
	unsigned int m_uListingSelectedIndex;

	enum CardState : uint8_t { notStarted, mounting, mounted, error } sdCardState;
	uint8_t initialDirectoryNesting;
};

#endif

#endif /* SRC_DISPLAY_FILESMENUITEM_H_ */
