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

class MassStorage
{
public:
	static bool CombineName(const StringRef& out, const char* directory, const char* fileName);		// returns false if error i.e. filename too long
	static const char* GetMonthName(const uint8_t month);

	FileStore* OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize);
	bool FindFirst(const char *directory, FileInfo &file_info);
	bool FindNext(FileInfo &file_info);
	void AbandonFindNext();
	bool Delete(const char* filePath);
	bool MakeDirectory(const char *parentDir, const char *dirName);
	bool MakeDirectory(const char *directory);
	bool Rename(const char *oldFilePath, const char *newFilePath);
	bool FileExists(const char *filePath) const;
	bool DirectoryExists(const StringRef& path) const;								// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
	bool DirectoryExists(const char *path) const;
	time_t GetLastModifiedTime(const char *filePath) const;
	bool SetLastModifiedTime(const char *file, time_t time);
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
	bool GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly) { return infoParser.GetFileInfo(filePath, info, quitEarly); }
	void RecordSimulationTime(const char *printingFilePath, uint32_t simSeconds);	// Append the simulated printing time to the end of the file

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
