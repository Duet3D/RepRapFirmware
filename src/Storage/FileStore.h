// This class handles input from, and output to, files.

#ifndef FILESTORE_H
#define FILESTORE_H

#include <Core.h>
#if HAS_MASS_STORAGE
# include "Libraries/Fatfs/ff.h"
#endif
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
# include "CRC32.h"
#endif

class Platform;
class FileWriteBuffer;

#if HAS_EMBEDDED_FILES
typedef int32_t FileIndex;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

enum class OpenMode : uint8_t
{
	read,			// open an existing file for reading
	write,			// write a file, replacing any existing file of the same name
	writeWithCrc,	// as write but calculate the CRC as we go
	append			// append to an existing file, or create a new file if it is not found
};

enum class FileUseMode : uint8_t
{
	free,			// file object is free
	readOnly,		// file object is in use for reading only
	readWrite,		// file object is in use for reading and writing
	invalidated		// file object is in use but file system has been invalidated
};

class FileStore
{
public:
	FileStore() noexcept;

    bool Open(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept;
	bool Read(char& b) noexcept
		{ return Read((char *_ecv_array)&b, sizeof(char)); }					// Read 1 character
	bool Read(uint8_t& b) noexcept
		{ return Read(reinterpret_cast<char&>(b)); }							// Read 1 byte
	int Read(char *_ecv_array buf, size_t nBytes) noexcept;						// Read a block of nBytes length
	int Read(uint8_t *_ecv_array buf, size_t nBytes) noexcept
		{ return Read(reinterpret_cast<char *_ecv_array>(buf), nBytes); }		// Read a block of nBytes length
	int ReadLine(char* buf, size_t nBytes) noexcept;			// As Read but stop after '\n' or '\r\n' and null-terminate
	bool Close() noexcept;										// Shut the file and tidy up
	bool ForceClose() noexcept;
	bool Seek(FilePosition pos) noexcept;						// Jump to pos in the file
	FilePosition Length() const noexcept;						// File size in bytes
	bool IsCloseRequested() const noexcept { return closeRequested; }
	bool IsFree() const noexcept { return usageMode == FileUseMode::free; }
	FilePosition Position() const noexcept;						// Return the current position in the file, assuming we are reading the file
	void Duplicate() noexcept;									// Create a second reference to this file

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	FileWriteBuffer *GetWriteBuffer() const noexcept;			// Return a pointer to the remaining space for writing
	bool Write(char b) noexcept;								// Write 1 byte
	bool Write(const char *_ecv_array s, size_t len) noexcept;				// Write a block of len bytes
	bool Write(const uint8_t *_ecv_array s, size_t len) noexcept;			// Write a block of len bytes
	bool Write(const char *_ecv_array s) noexcept;							// Write a string
	bool Flush() noexcept;										// Write remaining buffer data
	bool Truncate() noexcept;									// Truncate file at current file pointer
	uint32_t GetCRC32() const noexcept;
#endif

#if HAS_SBC_INTERFACE
	void Invalidate() noexcept;									// Invalidate the file
#endif

#if HAS_MASS_STORAGE
	uint32_t ClusterSize() const noexcept;						// Cluster size in bytes
	bool Invalidate(const FATFS *fs, bool doClose) noexcept;	// Invalidate the file if it uses the specified FATFS object
	bool IsOpenOn(const FATFS *fs) const noexcept;				// Return true if the file is open on the specified file system
	bool IsSameFile(const FIL& otherFile) const noexcept;		// Return true if the passed file is the same as ours
# if 0	// not currently used
	bool SetClusterMap(uint32_t[]) noexcept;					// Provide a cluster map for fast seeking
# endif
#endif

#if 0	// not currently used
	bool GoToEnd() noexcept;									// Position the file at the end (so you can write on the end).
#endif

private:
	void Init() noexcept;
	bool Store(const char *_ecv_array s, size_t len, size_t *bytesWritten) noexcept;	// Write data to the non-volatile storage

	volatile unsigned int openCount;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	FileWriteBuffer *writeBuffer;
	CRC32 crc;
#endif

#if HAS_MASS_STORAGE
    FIL file;
	static uint32_t longestWriteTime;
#endif

#if HAS_SBC_INTERFACE
	FileHandle handle;
	FilePosition length;
#endif

#if HAS_EMBEDDED_FILES
	FileIndex fileIndex;
#endif

#if HAS_EMBEDDED_FILES || HAS_SBC_INTERFACE
	FilePosition offset;
#endif

	volatile bool closeRequested;
	FileUseMode usageMode;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool calcCrc;
#endif
};

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

inline FileWriteBuffer *FileStore::GetWriteBuffer() const noexcept { return writeBuffer; }

inline bool FileStore::Write(const uint8_t *_ecv_array s, size_t len) noexcept { return Write(reinterpret_cast<const char *_ecv_array>(s), len); }

inline uint32_t FileStore::GetCRC32() const noexcept
{
	return crc.Get();
}

#endif

#endif	// HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

#endif	// #ifndef FILESTORE_H
