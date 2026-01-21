#ifndef MASSSTORAGE_H
#define MASSSTORAGE_H

#include <RepRapFirmware.h>
#include "FileWriteBuffer.h"
#include <Libraries/Fatfs/ff.h>
#include "FileStore.h"
#include "FileInfoParser.h"
#include <RTOSIface/RTOSIface.h>

#include <ctime>

#ifdef __ECV__

// Redefinitions of FatFs functions that we use with extra type information
_ecv_spec FRESULT f_open (FIL* fp, const TCHAR *_ecv_array path, BYTE mode) noexcept;			/* Open or create a file */
_ecv_spec FRESULT f_close (FIL* fp) noexcept;											/* Close an open file object */
_ecv_spec FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br) noexcept;			/* Read data from the file */
_ecv_spec FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw) noexcept;	/* Write data to the file */
_ecv_spec FRESULT f_lseek (FIL* fp, FSIZE_t ofs) noexcept;							/* Move file pointer of the file object */
_ecv_spec FRESULT f_truncate (FIL* fp) noexcept;										/* Truncate the file */
_ecv_spec FRESULT f_sync (FIL* fp) noexcept;											/* Flush cached data of the writing file */
_ecv_spec FRESULT f_opendir (DIR* dp, const TCHAR *_ecv_array path) noexcept;					/* Open a directory */
_ecv_spec FRESULT f_closedir (DIR* dp) noexcept;										/* Close an open directory */
_ecv_spec FRESULT f_readdir (DIR* dp, FILINFO* fno) noexcept;							/* Read a directory item */
_ecv_spec FRESULT f_findfirst (DIR* dp, FILINFO* fno, const TCHAR *_ecv_array path, const TCHAR* pattern) noexcept;	/* Find first file */
_ecv_spec FRESULT f_findnext (DIR* dp, FILINFO* fno) noexcept;						/* Find next file */
_ecv_spec FRESULT f_mkdir (const TCHAR *_ecv_array path) noexcept;								/* Create a sub directory */
_ecv_spec FRESULT f_unlink (const TCHAR *_ecv_array path) noexcept;								/* Delete an existing file or directory */
_ecv_spec FRESULT f_rename (const TCHAR *_ecv_array path_old, const TCHAR *_ecv_array path_new) noexcept;	/* Rename/Move a file or directory */
_ecv_spec FRESULT f_stat (const TCHAR *_ecv_array path, FILINFO* fno) noexcept;					/* Get file status */
_ecv_spec FRESULT f_chmod (const TCHAR *_ecv_array path, BYTE attr, BYTE mask) noexcept;			/* Change attribute of a file/dir */
_ecv_spec FRESULT f_utime (const TCHAR *_ecv_array path, const FILINFO* fno) noexcept;			/* Change timestamp of a file/dir */
_ecv_spec FRESULT f_chdir (const TCHAR *_ecv_array path) noexcept;								/* Change current directory */
_ecv_spec FRESULT f_chdrive (const TCHAR *_ecv_array path) noexcept;								/* Change current drive */
_ecv_spec FRESULT f_getcwd (TCHAR *_ecv_array buff, UINT len) noexcept;							/* Get current directory */
_ecv_spec FRESULT f_getfree (const TCHAR *_ecv_array path, DWORD* nclst, FATFS** fatfs) noexcept;	/* Get number of free clusters on the drive */
_ecv_spec FRESULT f_getlabel (const TCHAR *_ecv_array path, TCHAR* label, DWORD* vsn) noexcept;	/* Get volume label */
_ecv_spec FRESULT f_setlabel (const TCHAR *_ecv_array label) noexcept;							/* Set volume label */
_ecv_spec FRESULT f_forward (FIL* fp, UINT(*func)(const BYTE *_ecv_array ,UINT) noexcept, UINT btf, UINT* bf) noexcept;	/* Forward data to the stream */
_ecv_spec FRESULT f_expand (FIL* fp, FSIZE_t fsz, BYTE opt) noexcept;					/* Allocate a contiguous block to the file */
_ecv_spec FRESULT f_mount (FATFS *_ecv_null fs, const TCHAR *_ecv_array path, BYTE opt) noexcept;			/* Mount/Unmount a logical drive */
_ecv_spec FRESULT f_mkfs (const TCHAR *_ecv_array path, const MKFS_PARM* opt, void* work, UINT len) noexcept;	/* Create a FAT volume */
_ecv_spec FRESULT f_fdisk (BYTE pdrv, const LBA_t ptbl[], void* work) noexcept;		/* Divide a physical drive into some partitions */
_ecv_spec FRESULT f_setcp (WORD cp) noexcept;											/* Set current code page */

#endif

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
class GlobalVariables;

// Enum used to report whether we want a message of a file delete fails
enum class ErrorMessageMode : uint8_t
{
	noMessage = 0,
	messageUnlessMissing,
	messageAlways
};

namespace MassStorage
{
	bool CombineName(const StringRef& outbuf, const char *_ecv_array _ecv_null directory, const char *_ecv_array fileName) noexcept;	// returns false if error i.e. filename too long
	const char *_ecv_array GetMonthName(const uint8_t month) noexcept;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	void Init() noexcept;
	FileStore *_ecv_null OpenFile(const char *_ecv_array filePath, OpenMode mode, uint32_t preAllocSize) noexcept;
	bool FileExists(const char *_ecv_array filePath) noexcept;
	void CloseAllFiles() noexcept;
	void Spin() noexcept;

# ifdef DUET3_MB6HC
	size_t GetNumVolumes() noexcept;														// The number of SD slots may be 1 or 2 on the 6HC
# else
	inline size_t GetNumVolumes() noexcept { return NumSdCards; }
# endif
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	FileWriteBuffer *_ecv_null AllocateWriteBuffer() noexcept;
	size_t GetFileWriteBufferLength() noexcept;
	void ReleaseWriteBuffer(FileWriteBuffer *buffer) noexcept;
	bool Delete(const StringRef& filePath, ErrorMessageMode errorMessageMode, bool recursive = false) noexcept;
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
	GCodeResult GetFileInfo(const char *_ecv_array filePath, GCodeFileInfo& info, bool quitEarly, GlobalVariables *_ecv_null customVars) noexcept;
	GCodeResult Mount(size_t card, const StringRef& reply, bool reportSuccess) noexcept;
	GCodeResult Unmount(size_t card, const StringRef& reply) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;

# if SUPPORT_ASYNC_MOVES
	FileStore *_ecv_null DuplicateOpenHandle(const FileStore *f) noexcept;	// Duplicate a file handle, with the duplicate having its own position in the file. Use only when files opened in read-only mode.
# endif
#endif

#if HAS_MASS_STORAGE
	bool EnsurePath(const char *_ecv_array filePath, bool messageIfFailed) noexcept;
	bool MakeDirectory(const char *_ecv_array directory, bool messageIfFailed) noexcept;
	bool Rename(const char *_ecv_array oldFilePath, const char *_ecv_array newFilePath, bool deleteExisting, bool messageIfFailed) noexcept;
	time_t GetLastModifiedTime(const char *_ecv_array filePath) noexcept;
	bool SetLastModifiedTime(const char *_ecv_array file, time_t t) noexcept;
	bool CheckDriveMounted(const char *_ecv_array path) noexcept;
	bool IsCardDetected(size_t card) noexcept;
	unsigned int InvalidateFiles(const FATFS *fs) noexcept;									// Invalidate all open files on the specified file system, returning the number of files invalidated
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
