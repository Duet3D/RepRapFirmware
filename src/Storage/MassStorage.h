#ifndef MASSSTORAGE_H
#define MASSSTORAGE_H

#include "RepRapFirmware.h"
#include "Pins.h"
#include "FileWriteBuffer.h"
#include "Libraries/Fatfs/ff.h"
#include "GCodes/GCodeResult.h"
#include "FileStore.h"
#include "FileInfoParser.h"
#include "RTOSIface/RTOSIface.h"

#include <ctime>

// Info returned by FindFirst/FindNext calls
struct FileInfo
{
	time_t lastModified;
	uint32_t size;
	String<MaxFilenameLength> fileName;
	bool isDirectory;
};

namespace MassStorage
{
	bool CombineName(const StringRef& out, const char* directory, const char* fileName);		// returns false if error i.e. filename too long
	const char* GetMonthName(const uint8_t month);

#if HAS_MASS_STORAGE
	void Init();
	FileStore* OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize);
	bool FindFirst(const char *directory, FileInfo &file_info);
	bool FindNext(FileInfo &file_info);
	void AbandonFindNext();
	bool Delete(const char* filePath);
	bool MakeDirectory(const char *parentDir, const char *dirName);
	bool MakeDirectory(const char *directory);
	bool Rename(const char *oldFilePath, const char *newFilePath);
	bool FileExists(const char *filePath);
	bool DirectoryExists(const StringRef& path);									// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
	bool DirectoryExists(const char *path);
	time_t GetLastModifiedTime(const char *filePath);
	bool SetLastModifiedTime(const char *file, time_t time);
	GCodeResult Mount(size_t card, const StringRef& reply, bool reportSuccess);
	GCodeResult Unmount(size_t card, const StringRef& reply);
	bool IsDriveMounted(size_t drive);
	bool CheckDriveMounted(const char* path);
	bool IsCardDetected(size_t card);
	unsigned int InvalidateFiles(const FATFS *fs, bool doClose);					// Invalidate all open files on the specified file system, returning the number of files invalidated
	bool AnyFileOpen(const FATFS *fs);												// Return true if any files are open on the file system
	void CloseAllFiles();
	unsigned int GetNumFreeFiles();
	void Spin();
	const Mutex& GetVolumeMutex(size_t vol);
	bool GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly);
	void RecordSimulationTime(const char *printingFilePath, uint32_t simSeconds);	// Append the simulated printing time to the end of the file
	FileWriteBuffer *AllocateWriteBuffer();
	void ReleaseWriteBuffer(FileWriteBuffer *buffer);

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};

	InfoResult GetCardInfo(size_t slot, uint64_t& capacity, uint64_t& freeSpace, uint32_t& speed, uint32_t& clSize);
#endif

};

#endif
