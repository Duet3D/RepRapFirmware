/*
 * MassStorageEmbedded.cpp
 *
 * Implementations of functions declared in MassStorage.h when we are using a readonly file system appended to the binary
 *  Created on: 31 Jul 2021
 *      Author: David
 */

#include "MassStorage.h"

#if HAS_EMBEDDED_FILES

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>
#include "FileStore.h"

void MassStorage::Init() noexcept
{
	//TODO
}

FileStore* MassStorage::OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept
{
	if (mode == OpenMode::read)
	{
		// Try to open the embedded file
		//TODO
	}
	return nullptr;
}

bool MassStorage::FileExists(const char *filePath) noexcept
{
	//TODO
	return false;
}

// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
bool MassStorage::DirectoryExists(const StringRef& path) noexcept
{
	//TODO
	return false;
}

bool MassStorage::DirectoryExists(const char *path) noexcept
{
	//TODO
	return false;
}

bool MassStorage::IsDriveMounted(size_t drive) noexcept
{
	return drive == 0;
}

bool MassStorage::FindFirst(const char *directory, FileInfo &file_info) noexcept
{
	//TODO
	return false;
}

bool MassStorage::FindNext(FileInfo &file_info) noexcept
{
	//TODO
	return false;
}

void MassStorage::AbandonFindNext() noexcept
{
	//TODO
}

#endif

// End
