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

static const char *fileSearchDirectory;
static uint32_t fileSearchNextNumber;

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

// Find the next file starting from fileSearchNextNumber that is in directory fileSearchDirectory
static bool FindNextFile(FileInfo& info) noexcept
{
	while (fileSearchNextNumber < _firmware_end.numFiles)
	{
		const EmbeddedFileDescriptor& fd = _firmware_end.files[fileSearchNextNumber++];
		const char *fname = fd.GetName();
		if (StringStartsWithIgnoreCase(fname, fileSearchDirectory))
		{
			// The file path starts with the correct directory, but it could be in a subdirectory
			const char *p = fname + strlen(fileSearchDirectory);
			if (*p == '/')
			{
				++p;			// point to start of possible filename
				const char *q = p;
				while (*q != 0 && *q != '/')
				{
					++q;
				}
				if (*q == 0)
				{
					// Found a file in the right directory
					info.fileName.copy(p);
					info.isDirectory = false;
					info.lastModified = 0;
					info.size = fd.contentLength;
					return true;
				}
			}
		}
	}
	return true;
}

// Find the first file. Any trailing "/" in the directory has been removed.
bool EmbeddedFiles::FindFirst(const char *directory, FileInfo &info) noexcept
{
	if (_firmware_end.magic == EmbeddedFilesHeader::MagicValue)
	{
		// Check that we have the directory, and store a pointer to it
		if (directory[0] == 0)
		{
			fileSearchDirectory = "";					// root directory
		}
		else
		{
			const char *cd = _firmware_end.GetDirectories();
			for (;;)
			{
				if (cd[0] == 0)
				{
					return false;
				}
				if (StringEqualsIgnoreCase(cd, directory))
				{
					fileSearchDirectory = cd;
					break;
				}
				cd += strlen(cd) + 1;
			}
		}

		// fileSearchDirectory now points to the directory string - we need to save it for the FindNext call
		fileSearchNextNumber = 0;
		return FindNextFile(info);
	}
	return false;
}

bool EmbeddedFiles::FindNext(FileInfo &info) noexcept
{
	if (_firmware_end.magic == EmbeddedFilesHeader::MagicValue)
	{
		return FindNextFile(info);
	}
	return false;
}

// Seek to a position
FilePosition EmbeddedFiles::Seek(int32_t fileIndex, FilePosition pos) noexcept
{
	return min<FilePosition>(pos, Length(fileIndex));
}

// Return the file size in bytes, or 0 if the file index is invalid
FilePosition EmbeddedFiles::Length(int32_t fileIndex) noexcept
{
	return (_firmware_end.magic == EmbeddedFilesHeader::MagicValue && fileIndex >= 0 && fileIndex < (int32_t)_firmware_end.numFiles)
			? _firmware_end.files[fileIndex].contentLength
				: 0;
}

int EmbeddedFiles::Read(FileIndex fileIndex, char* extBuf, size_t nBytes) noexcept
{
	//TODO
	return -1;
}

FileIndex EmbeddedFiles::OpenFile(const char *filePath) noexcept
{
	//TODO
	return -1;
}

#endif

// End
