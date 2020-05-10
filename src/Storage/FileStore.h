// This class handles input from, and output to, files.

#ifndef FILESTORE_H
#define FILESTORE_H

#include "Core.h"
#include "Libraries/Fatfs/ff.h"
#include "CRC32.h"

class Platform;
class FileWriteBuffer;

#if HAS_MASS_STORAGE

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
	bool Read(char& b) noexcept;								// Read 1 byte
	bool Read(uint8_t& b) noexcept
		{ return Read((char&)b); }								// Read 1 byte
	int Read(char* buf, size_t nBytes) noexcept;				// Read a block of nBytes length
	int Read(uint8_t* buf, size_t nBytes) noexcept
		{ return Read((char*)buf, nBytes); }					// Read a block of nBytes length
	int ReadLine(char* buf, size_t nBytes) noexcept;			// As Read but stop after '\n' or '\r\n' and null-terminate
	FileWriteBuffer *GetWriteBuffer() const noexcept;			// Return a pointer to the remaining space for writing
	bool Write(char b) noexcept;								// Write 1 byte
	bool Write(const char *s, size_t len) noexcept;				// Write a block of len bytes
	bool Write(const uint8_t *s, size_t len) noexcept;			// Write a block of len bytes
	bool Write(const char* s) noexcept;							// Write a string
	bool Close() noexcept;										// Shut the file and tidy up
	bool ForceClose() noexcept;
	bool Seek(FilePosition pos) noexcept;						// Jump to pos in the file
	FilePosition Position() const noexcept;						// Return the current position in the file, assuming we are reading the file
	uint32_t ClusterSize() const noexcept;						// Cluster size in bytes
	FilePosition Length() const noexcept;						// File size in bytes
#if 0	// not currently used
	bool GoToEnd() noexcept;									// Position the file at the end (so you can write on the end).
#endif

	void Duplicate() noexcept;									// Create a second reference to this file
	bool Flush() noexcept;										// Write remaining buffer data
	bool Truncate() noexcept;									// Truncate file at current file pointer
	bool Invalidate(const FATFS *fs, bool doClose) noexcept;	// Invalidate the file if it uses the specified FATFS object
	bool IsOpenOn(const FATFS *fs) const noexcept;				// Return true if the file is open on the specified file system
	uint32_t GetCRC32() const noexcept;
	bool IsSameFile(const FIL& otherFile) const noexcept;		// Return true if the passed file is the same as ours
	bool IsCloseRequested() const noexcept { return closeRequested; }
	bool IsFree() const noexcept { return usageMode == FileUseMode::free; }

#if 0	// not currently used
	bool SetClusterMap(uint32_t[]) noexcept;					// Provide a cluster map for fast seeking
#endif

private:
	void Init() noexcept;
	FRESULT Store(const char *s, size_t len, size_t *bytesWritten) noexcept; // Write data to the non-volatile storage

    FIL file;
	FileWriteBuffer *writeBuffer;
	volatile unsigned int openCount;
	volatile bool closeRequested;
	bool calcCrc;
	FileUseMode usageMode;

	CRC32 crc;

	static uint32_t longestWriteTime;
};

inline FileWriteBuffer *FileStore::GetWriteBuffer() const noexcept { return writeBuffer; }

inline bool FileStore::Write(const uint8_t *s, size_t len) noexcept { return Write(reinterpret_cast<const char *>(s), len); }

inline uint32_t FileStore::GetCRC32() const noexcept
{
	return crc.Get();
}

#endif

#endif
