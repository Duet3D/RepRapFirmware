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

// Members of MassStorage that are replaced
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

// Members of FileStore that are replaced

// Return the file size in bytes
FilePosition FileStore::Length() const noexcept
{
	//TODO
	return 0;
}

// Return the current position in the file, assuming we are reading the file
FilePosition FileStore::Position() const noexcept
{
	//TODO
	return 0;
}

// Open a file
bool FileStore::Open(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept
{
	//TODO
	return false;
}

// Create a second reference to this file
void FileStore::Duplicate() noexcept
{
	//TODO
}

#endif

// End
