// This class handles input from, and output to, files.

#ifndef FILESTORE_H
#define FILESTORE_H

#include "Core.h"
#include "Libraries/Fatfs/ff.h"
#include "CRC32.h"

class Platform;
class FileWriteBuffer;

class FileStore
{
public:
	bool Read(char& b);								// Read 1 byte
	int Read(char* buf, size_t nBytes);				// Read a block of nBytes length
	int ReadLine(char* buf, size_t nBytes);			// As Read but stop after '\n' or '\r\n' and null-terminate
	bool Write(char b);								// Write 1 byte
	bool Write(const char *s, size_t len);			// Write a block of len bytes
	bool Write(const uint8_t *s, size_t len);		// Write a block of len bytes
	bool Write(const char* s);						// Write a string
	bool Close();									// Shut the file and tidy up
	bool Seek(FilePosition pos);					// Jump to pos in the file
	FilePosition Position() const;					// Return the current position in the file, assuming we are reading the file
#if 0	// not currently used
	bool GoToEnd();									// Position the file at the end (so you can write on the end).
#endif
	FilePosition Length() const;					// File size in bytes
	float FractionRead() const;						// How far in we are
	void Duplicate();								// Create a second reference to this file
	bool Flush();									// Write remaining buffer data
	void Invalidate(const FATFS *fs);				// Invalidate the file if it uses the specified FATFS object
	bool IsOpenOn(const FATFS *fs) const;			// Return true if the file is open on the specified file system
	uint32_t GetCRC32() const;

#if 0	// not currently used
	bool SetClusterMap(uint32_t[]);					// Provide a cluster map for fast seeking
#endif
	static float GetAndClearLongestWriteTime();		// Return the longest time it took to write a block to a file, in milliseconds

	friend class Platform;

protected:

	FileStore(Platform* p);
	void Init();
    bool Open(const char* directory, const char* fileName, bool write);
    FRESULT Store(const char *s, size_t len, size_t *bytesWritten); // Write data to the non-volatile storage

private:
	Platform* platform;

	FIL file;
	FileWriteBuffer *writeBuffer;
	volatile unsigned int openCount;
	volatile bool closeRequested;

	bool inUse;
	bool writing;
	CRC32 crc;

	static uint32_t longestWriteTime;
};

inline bool FileStore::Write(const uint8_t *s, size_t len) { return Write(reinterpret_cast<const char *>(s), len); }

inline uint32_t FileStore::GetCRC32() const
{
	return crc.Get();
}

#endif
