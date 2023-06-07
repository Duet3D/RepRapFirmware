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

extern const uint32_t _firmware_end[];

struct EmbeddedFileDescriptor
{
	uint32_t nameOffset;
	uint32_t contentOffset;
	uint32_t contentLength;

	const char* GetName() const noexcept { return reinterpret_cast<const char*>(&_firmware_end) + nameOffset; }
	const char* GetContent() const noexcept { return reinterpret_cast<const char*>(&_firmware_end) + contentOffset; }
};

struct EmbeddedFilesHeader
{
	uint32_t magic;
	uint32_t directoriesOffset;
	uint32_t numFiles;
	const EmbeddedFileDescriptor files[999];				// the array length is actually 'numFiles'

	static constexpr uint32_t MagicValue = 0x543C2BEF;
	const char* GetDirectories() const noexcept { return reinterpret_cast<const char*>(&_firmware_end) + directoriesOffset; }
};

static const EmbeddedFilesHeader& fileSystem = *reinterpret_cast<const EmbeddedFilesHeader*>(&_firmware_end);

static const char *fileSearchDirectory;
static uint32_t fileSearchNextNumber;

// Skip any leading "0:" in a path. We don't worry about "1:", "2:" etc. because ":" is not a valid filename character, so the path won't match anything.
static const char *SkipDriveNumber(const char *path) noexcept
{
	return (path[0] == '0' && path[1] == ':') ? path + 2 : path;
}

// Members of MassStorage that are replaced
bool MassStorage::FileExists(const char *filePath) noexcept
{
	if (fileSystem.magic == EmbeddedFilesHeader::MagicValue)
	{
		filePath = SkipDriveNumber(filePath);
		uint32_t numFiles = fileSystem.numFiles;
		const EmbeddedFileDescriptor *filePtr = fileSystem.files;
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
bool EmbeddedFiles::DirectoryExists(const StringRef& dirPath) noexcept
{
	if (fileSystem.magic == EmbeddedFilesHeader::MagicValue)
	{
		const char * const path = SkipDriveNumber(dirPath.c_str());
		if (path[0] == 0)
		{
			return true;				// root directory
		}

		const char *cd = fileSystem.GetDirectories();
		while (cd[0] != 0)
		{
			if (StringEqualsIgnoreCase(cd, path))
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
	while (fileSearchNextNumber < fileSystem.numFiles)
	{
		const EmbeddedFileDescriptor& fd = fileSystem.files[fileSearchNextNumber];
		++fileSearchNextNumber;								// don't look at the same file again, whether we find a match or not
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
	return false;
}

// Find the first file. Any trailing "/" in the directory has been removed.
bool EmbeddedFiles::FindFirst(const char *directory, FileInfo &info) noexcept
{
	if (fileSystem.magic == EmbeddedFilesHeader::MagicValue)
	{
		directory = SkipDriveNumber(directory);

		// Check that we have the directory, and store a pointer to it
		if (directory[0] == 0)
		{
			fileSearchDirectory = "";					// root directory
		}
		else
		{
			const char *cd = fileSystem.GetDirectories();
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
	if (fileSystem.magic == EmbeddedFilesHeader::MagicValue)
	{
		return FindNextFile(info);
	}
	return false;
}


// Return the file size in bytes, or 0 if the file index is invalid
FilePosition EmbeddedFiles::Length(int32_t fileIndex) noexcept
{
	return (fileSystem.magic == EmbeddedFilesHeader::MagicValue && fileIndex >= 0 && fileIndex < (int32_t)fileSystem.numFiles)
			? fileSystem.files[fileIndex].contentLength
				: 0;
}

// Open a file
FileIndex EmbeddedFiles::OpenFile(const char *filePath) noexcept
{
	if (fileSystem.magic == EmbeddedFilesHeader::MagicValue)
	{
		filePath = SkipDriveNumber(filePath);
		for (FileIndex fi = 0; fi < (FileIndex)fileSystem.numFiles; ++fi)
		{
			if (StringEqualsIgnoreCase(filePath, fileSystem.files[fi].GetName()))
			{
				return fi;
			}
		}
	}
	return (FileIndex)-1;
}

// Read from a file
int EmbeddedFiles::Read(FileIndex fileIndex, FilePosition pos, char* extBuf, size_t nBytes) noexcept
{
	if (fileSystem.magic == EmbeddedFilesHeader::MagicValue && fileIndex >= 0 && fileIndex < (int32_t)fileSystem.numFiles)
	{
		const FilePosition fileLength = fileSystem.files[fileIndex].contentLength;
		if (pos < fileLength)
		{
			if (nBytes > fileLength - pos)
			{
				nBytes = fileLength - pos;
			}
			memcpy(extBuf, fileSystem.files[fileIndex].GetContent() + pos, nBytes);
			return nBytes;
		}
		return 0;
	}
	return -1;
}

#endif

// End
