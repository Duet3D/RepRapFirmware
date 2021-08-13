/*
 * MassStorageEmbedded.cpp
 *
 * Implementations of functions declared in MassStorage.h when we are using a readonly file system appended to the binary
 *  Created on: 31 Jul 2021
 *      Author: David
 */

#include "MassStorage.h"

#if SAM4S
constexpr uint32_t FlashStart = IFLASH0_ADDR;
#endif

#if HAS_EMBEDDED_FILES

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>
#include "FileStore.h"

struct EmbeddedFileDescriptor
{
	uint32_t nameOffset;
	uint32_t contentOffset;
	uint32_t contentLength;

	const char* GetName() const noexcept { return reinterpret_cast<const char*>(FlashStart + nameOffset); }
	const char* GetContent() const noexcept { return reinterpret_cast<const char*>(FlashStart + contentOffset); }
};

struct EmbeddedFilesHeader
{
	uint32_t magic;
	uint32_t directoriesOffset;
	uint32_t numFiles;
	const EmbeddedFileDescriptor files[];				// gcc extension: array of unspecified length at end of a struct

	static constexpr uint32_t MagicValue = 0;	//TODO what is it?
	const char* GetDirectories() const noexcept { return reinterpret_cast<const char*>(FlashStart + directoriesOffset); }
};

extern const EmbeddedFilesHeader _firmware_end;

static const char *fileSearchDirectory = nullptr;
static const char *fileSearchNextNumber = 0;

// Members of MassStorage that are replaced
bool MassStorage::FileExists(const char *filePath) noexcept
{
	if (_firmware_end.magic == EmbeddedFilesHeader::MagicValue)
	{
		uint32_t numFiles = _firmware_end.numFiles;
		const EmbeddedFileDescriptor *filePtr = _firmware_end.files;
		while (numFiles != 0)
		{
			if (StringEqualsIgnoreCase(filePath, filePtr->GetName()))
			{
				return true;
			}
			++filePtr;
			--numFiles;
		}
	}
	return false;
}

// Test whether a directory exists. Any trailing '/' has already been removed.
bool EmbeddedFiles::DirectoryExists(const StringRef& path) noexcept
{
	if (_firmware_end.magic == EmbeddedFilesHeader::MagicValue)
	{
		if (path[0] == 0)
		{
			return true;				// root directory
		}

		const char *cd = _firmware_end.GetDirectories();
		while (cd[0] != 0)
		{
			if (StringEqualsIgnoreCase(cd, path.c_str()))
			{
				return true;
			}
			cd += strlen(cd) + 1;
		}
	}
	return false;
}

bool EmbeddedFiles::FindFirst(const char *directory, FileInfo &file_info) noexcept
{
	if (_firmware_end.magic == EmbeddedFilesHeader::MagicValue)
	{
		//TODO
	}
	return false;
}

bool EmbeddedFiles::FindNext(FileInfo &file_info) noexcept
{
	if (_firmware_end.magic == EmbeddedFilesHeader::MagicValue)
	{
		//TODO
	}
	return false;
}

// Members of FileStore that are replaced (probably to be moved back into FileStore)

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

#endif

// End
