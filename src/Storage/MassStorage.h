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

class ObjectModel;

namespace MassStorage
{
	bool CombineName(const StringRef& out, const char* directory, const char* fileName) noexcept;	// returns false if error i.e. filename too long
	const char* GetMonthName(const uint8_t month) noexcept;

#if HAS_MASS_STORAGE
	void Init() noexcept;
	FileStore* OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept;
	bool FindFirst(const char *directory, FileInfo &file_info) noexcept;
	bool FindNext(FileInfo &file_info) noexcept;
	void AbandonFindNext() noexcept;
	bool Delete(const char* filePath, bool messageIfFailed) noexcept;
	bool EnsurePath(const char* filePath, bool messageIfFailed) noexcept;
	bool MakeDirectory(const char *directory, bool messageIfFailed) noexcept;
	bool Rename(const char *oldFilePath, const char *newFilePath, bool messageIfFailed) noexcept;
	bool FileExists(const char *filePath) noexcept;
	bool DirectoryExists(const StringRef& path) noexcept;									// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
	bool DirectoryExists(const char *path) noexcept;
	time_t GetLastModifiedTime(const char *filePath) noexcept;
	bool SetLastModifiedTime(const char *file, time_t time) noexcept;
	GCodeResult Mount(size_t card, const StringRef& reply, bool reportSuccess) noexcept;
	GCodeResult Unmount(size_t card, const StringRef& reply) noexcept;
	bool IsDriveMounted(size_t drive) noexcept;
	bool CheckDriveMounted(const char* path) noexcept;
	bool IsCardDetected(size_t card) noexcept;
	unsigned int InvalidateFiles(const FATFS *fs, bool doClose) noexcept;					// Invalidate all open files on the specified file system, returning the number of files invalidated
	bool AnyFileOpen(const FATFS *fs) noexcept;												// Return true if any files are open on the file system
	void CloseAllFiles() noexcept;
	unsigned int GetNumFreeFiles() noexcept;
	void Spin() noexcept;
	const Mutex& GetVolumeMutex(size_t vol) noexcept;
	bool GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly) noexcept;
	void RecordSimulationTime(const char *printingFilePath, uint32_t simSeconds) noexcept;	// Append the simulated printing time to the end of the file
	FileWriteBuffer *AllocateWriteBuffer() noexcept;
	void ReleaseWriteBuffer(FileWriteBuffer *buffer) noexcept;
	void Diagnostics(MessageType mtype) noexcept;

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};

	InfoResult GetCardInfo(size_t slot, uint64_t& capacity, uint64_t& freeSpace, uint32_t& speed, uint32_t& clSize) noexcept;

# if SUPPORT_OBJECT_MODEL
	inline size_t GetNumVolumes() noexcept { return NumSdCards; }
	const ObjectModel *GetVolume(size_t vol) noexcept;
# endif

#endif

};

#endif
