#ifndef MASSSTORAGE_H
#define MASSSTORAGE_H

#include "RepRapFirmware.h"
#include "Pins.h"
#include "FileWriteBuffer.h"
#include "Libraries/Fatfs/ff.h"
#include "GCodes/GCodeResult.h"
#include "FileStore.h"
#include "FileInfoParser.h"

#include <ctime>

#include "RTOSIface.h"

// Info returned by FindFirst/FindNext calls
struct FileInfo
{
	bool isDirectory;
	char fileName[MaxFilenameLength];
	uint32_t size;
	time_t lastModified;
};

class MassStorage
{
public:
	FileStore* OpenFile(const char* directory, const char* fileName, OpenMode mode);
	bool FindFirst(const char *directory, FileInfo &file_info);
	bool FindNext(FileInfo &file_info);
	void AbandonFindNext();
	const char* GetMonthName(const uint8_t month);
	static void CombineName(const StringRef& out, const char* directory, const char* fileName);
	bool Delete(const char* directory, const char* fileName, bool silent = false);
	bool MakeDirectory(const char *parentDir, const char *dirName);
	bool MakeDirectory(const char *directory);
	bool Rename(const char *oldFilename, const char *newFilename);
	bool FileExists(const char *file) const;
	bool FileExists(const char* directory, const char *fileName) const;
	bool DirectoryExists(const StringRef& path) const;								// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
	bool DirectoryExists(const char *path) const;
	bool DirectoryExists(const char* directory, const char* subDirectory);
	time_t GetLastModifiedTime(const char* directory, const char *fileName) const;
	bool SetLastModifiedTime(const char* directory, const char *file, time_t time);
	GCodeResult Mount(size_t card, const StringRef& reply, bool reportSuccess);
	GCodeResult Unmount(size_t card, const StringRef& reply);
	bool IsDriveMounted(size_t drive) const { return drive < NumSdCards && info[drive].isMounted; }
	bool CheckDriveMounted(const char* path);
	bool IsCardDetected(size_t card) const;
	unsigned int InvalidateFiles(const FATFS *fs, bool doClose);					// Invalidate all open files on the specified file system, returning the number of files invalidated
	bool AnyFileOpen(const FATFS *fs) const;										// Return true if any files are open on the file system
	void CloseAllFiles();
	unsigned int GetNumFreeFiles() const;
	void Spin();
	const Mutex& GetVolumeMutex(size_t vol) const { return info[vol].volMutex; }
	bool GetFileInfo(const char *directory, const char *fileName, GCodeFileInfo& info, bool quitEarly) { return infoParser.GetFileInfo(directory, fileName, info, quitEarly); }
	void RecordSimulationTime(const char *printingFilename, uint32_t simSeconds);	// Append the simulated printing time to the end of the file

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};

	InfoResult GetCardInfo(size_t slot, uint64_t& capacity, uint64_t& freeSpace, uint32_t& speed, uint32_t& clSize);

friend class Platform;
friend class FileStore;

protected:
	MassStorage(Platform* p);
	void Init();

	FileWriteBuffer *AllocateWriteBuffer();
	void ReleaseWriteBuffer(FileWriteBuffer *buffer);

private:
	enum class CardDetectState : uint8_t
	{
		notPresent = 0,
		inserting,
		present,
		removing
	};

	struct SdCardInfo
	{
		FATFS fileSystem;
		uint32_t cdChangedTime;
		uint32_t mountStartTime;
		Mutex volMutex;
		Pin cdPin;
		bool mounting;
		bool isMounted;
		CardDetectState cardState;
	};

	unsigned int InternalUnmount(size_t card, bool doClose);
	static time_t ConvertTimeStamp(uint16_t fdate, uint16_t ftime);

	SdCardInfo info[NumSdCards];

	Mutex fsMutex, dirMutex;

	FileInfoParser infoParser;
	DIR findDir;
	FileWriteBuffer *freeWriteBuffers;
	FileStore files[MAX_FILES];
};

#endif
