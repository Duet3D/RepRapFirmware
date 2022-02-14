#ifndef MASSSTORAGE_H
#define MASSSTORAGE_H

#include <RepRapFirmware.h>
#include "FileWriteBuffer.h"
#include <Libraries/Fatfs/ff.h>
#include "FileStore.h"
#include "FileInfoParser.h"
#include <RTOSIface/RTOSIface.h>

#include <ctime>

// Info returned by FindFirst/FindNext calls
struct FileInfo
{
	time_t lastModified;
	uint32_t size;
	String<MaxFilenameLength> fileName;
	bool isDirectory;
};

#if HAS_EMBEDDED_FILES

// Functions that we call out to when using an embedded filesystem
namespace EmbeddedFiles
{
	bool DirectoryExists(const StringRef& path) noexcept;
	bool FindFirst(const char *directory, FileInfo& info) noexcept;
	bool FindNext(FileInfo& info) noexcept;
	FilePosition Length(FileIndex fileIndex) noexcept;
	int Read(FileIndex fileIndex, FilePosition pos, char* extBuf, size_t nBytes) noexcept;
	FileIndex OpenFile(const char *filePath) noexcept;
}

#endif

class ObjectModel;

namespace MassStorage
{
	bool CombineName(const StringRef& outbuf, const char* directory, const char* fileName) noexcept;	// returns false if error i.e. filename too long
	const char* GetMonthName(const uint8_t month) noexcept;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	void Init() noexcept;
	FileStore* OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept;
	bool FileExists(const char *filePath) noexcept;
	void CloseAllFiles() noexcept;
	void Spin() noexcept;

# ifdef DUET3_MB6HC
	size_t GetNumVolumes() noexcept;														// The number of SD slots may be 1 or 2 on the 6HC
# else
	inline size_t GetNumVolumes() noexcept { return NumSdCards; }
# endif
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	FileWriteBuffer *AllocateWriteBuffer() noexcept;
	size_t GetFileWriteBufferLength() noexcept;
	void ReleaseWriteBuffer(FileWriteBuffer *buffer) noexcept;
	bool Delete(const char* filePath, bool messageIfFailed) noexcept;
#endif

#if HAS_SBC_INTERFACE
	bool AnyFileOpen() noexcept;															// Return true if any files are open on the file system
	void InvalidateAllFiles() noexcept;
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	bool DirectoryExists(const StringRef& path) noexcept;									// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
	bool DirectoryExists(const char *_ecv_array path) noexcept;
	unsigned int GetNumFreeFiles() noexcept;
	bool IsDriveMounted(size_t drive) noexcept;
	bool FindFirst(const char *_ecv_array directory, FileInfo &file_info) noexcept;
	bool FindNext(FileInfo &file_info) noexcept;
	void AbandonFindNext() noexcept;
	GCodeResult GetFileInfo(const char *_ecv_array filePath, GCodeFileInfo& info, bool quitEarly) noexcept;
	GCodeResult Mount(size_t card, const StringRef& reply, bool reportSuccess) noexcept;
	GCodeResult Unmount(size_t card, const StringRef& reply) noexcept;
	void Diagnostics(MessageType mtype) noexcept;
#endif

#if HAS_MASS_STORAGE
	bool EnsurePath(const char *_ecv_array filePath, bool messageIfFailed) noexcept;
	bool MakeDirectory(const char *_ecv_array directory, bool messageIfFailed) noexcept;
	bool Rename(const char *_ecv_array oldFilePath, const char *_ecv_array newFilePath, bool deleteExisting, bool messageIfFailed) noexcept;
	time_t GetLastModifiedTime(const char *_ecv_array filePath) noexcept;
	bool SetLastModifiedTime(const char *_ecv_array file, time_t t) noexcept;
	bool CheckDriveMounted(const char* path) noexcept;
	bool IsCardDetected(size_t card) noexcept;
	unsigned int InvalidateFiles(const FATFS *fs, bool doClose) noexcept;					// Invalidate all open files on the specified file system, returning the number of files invalidated
	bool AnyFileOpen(const FATFS *fs) noexcept;												// Return true if any files are open on the file system
	Mutex& GetVolumeMutex(size_t vol) noexcept;
	void RecordSimulationTime(const char *_ecv_array printingFilePath, uint32_t simSeconds) noexcept;	// Append the simulated printing time to the end of the file
	uint16_t GetVolumeSeq(unsigned int volume) noexcept;

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};

	struct SdCardReturnedInfo
	{
		uint64_t cardCapacity;
		uint64_t partitionSize;
		uint64_t freeSpace;
		uint32_t clSize;
		uint32_t speed;
	};

	InfoResult GetCardInfo(size_t slot, SdCardReturnedInfo& returnedInfo) noexcept;

# ifdef DUET3_MB6HC
	GCodeResult ConfigureSdCard(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Configure additional SD card slots
# endif

# if SUPPORT_OBJECT_MODEL
	const ObjectModel *_ecv_from GetVolume(size_t vol) noexcept;
# endif

#endif

}


#endif
